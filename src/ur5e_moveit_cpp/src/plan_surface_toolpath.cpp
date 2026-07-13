#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;

namespace
{

constexpr double kMillimetersToMeters = 0.001;
constexpr double kPi = 3.14159265358979323846;

struct CsvToolpathPoint
{
  int line_id;
  int point_index;
  geometry_msgs::msg::Pose object_pose;
};

std::vector<std::string> splitCsvLine(
  const std::string& line)
{
  std::vector<std::string> values;
  std::stringstream stream(line);
  std::string value;

  while (std::getline(stream, value, ','))
  {
    values.push_back(value);
  }

  return values;
}

double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

geometry_msgs::msg::Pose transformToPose(
  const tf2::Transform& transform)
{
  geometry_msgs::msg::Pose pose;

  const tf2::Vector3& position =
    transform.getOrigin();

  tf2::Quaternion orientation =
    transform.getRotation();

  orientation.normalize();

  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();

  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();

  return pose;
}

std::map<int, std::vector<CsvToolpathPoint>>
loadToolpathCsv(const std::string& csv_path)
{
  std::ifstream input(csv_path);

  if (!input.is_open())
  {
    throw std::runtime_error(
      "Could not open toolpath CSV: " + csv_path);
  }

  std::string header_line;

  if (!std::getline(input, header_line))
  {
    throw std::runtime_error(
      "Toolpath CSV is empty: " + csv_path);
  }

  if (
    !header_line.empty() &&
    header_line.back() == '\r')
  {
    header_line.pop_back();
  }

  const std::vector<std::string> headers =
    splitCsvLine(header_line);

  std::unordered_map<std::string, std::size_t>
    header_indices;

  for (
    std::size_t index = 0;
    index < headers.size();
    ++index)
  {
    header_indices[headers[index]] = index;
  }

  const std::vector<std::string> required_headers = {
    "line_id",
    "point_index",
    "tcp_x_mm",
    "tcp_y_mm",
    "tcp_z_mm",
    "qx",
    "qy",
    "qz",
    "qw"
  };

  for (const std::string& header : required_headers)
  {
    if (header_indices.count(header) == 0)
    {
      throw std::runtime_error(
        "CSV is missing required column: " + header);
    }
  }

  std::map<int, std::vector<CsvToolpathPoint>>
    points_by_line;

  std::string line;
  std::size_t csv_row = 1;

  while (std::getline(input, line))
  {
    ++csv_row;

    if (
      !line.empty() &&
      line.back() == '\r')
    {
      line.pop_back();
    }

    if (line.empty())
    {
      continue;
    }

    const std::vector<std::string> values =
      splitCsvLine(line);

    if (values.size() != headers.size())
    {
      throw std::runtime_error(
        "CSV row " +
        std::to_string(csv_row) +
        " has an unexpected number of fields");
    }

    auto value =
      [&values, &header_indices](
        const std::string& name) -> const std::string&
      {
        return values.at(header_indices.at(name));
      };

    CsvToolpathPoint point;

    point.line_id =
      std::stoi(value("line_id"));

    point.point_index =
      std::stoi(value("point_index"));

    point.object_pose.position.x =
      std::stod(value("tcp_x_mm")) *
      kMillimetersToMeters;

    point.object_pose.position.y =
      std::stod(value("tcp_y_mm")) *
      kMillimetersToMeters;

    point.object_pose.position.z =
      std::stod(value("tcp_z_mm")) *
      kMillimetersToMeters;

    tf2::Quaternion orientation(
      std::stod(value("qx")),
      std::stod(value("qy")),
      std::stod(value("qz")),
      std::stod(value("qw")));

    if (orientation.length2() < 1e-12)
    {
      throw std::runtime_error(
        "CSV row " +
        std::to_string(csv_row) +
        " contains a zero-length quaternion");
    }

    orientation.normalize();

    point.object_pose.orientation.x =
      orientation.x();

    point.object_pose.orientation.y =
      orientation.y();

    point.object_pose.orientation.z =
      orientation.z();

    point.object_pose.orientation.w =
      orientation.w();

    points_by_line[point.line_id].push_back(
      point);
  }

  if (points_by_line.empty())
  {
    throw std::runtime_error(
      "Toolpath CSV contains no path points");
  }

  for (auto& entry : points_by_line)
  {
    std::vector<CsvToolpathPoint>& points =
      entry.second;

    std::sort(
      points.begin(),
      points.end(),
      [](
        const CsvToolpathPoint& first,
        const CsvToolpathPoint& second)
      {
        return first.point_index <
               second.point_index;
      });
  }

  return points_by_line;
}

std::vector<geometry_msgs::msg::Pose>
transformLineToWorld(
  const std::vector<CsvToolpathPoint>& points,
  const tf2::Transform& object_transform)
{
  std::vector<geometry_msgs::msg::Pose> world_poses;

  world_poses.reserve(points.size());

  for (const CsvToolpathPoint& point : points)
  {
    tf2::Quaternion object_tool_orientation(
      point.object_pose.orientation.x,
      point.object_pose.orientation.y,
      point.object_pose.orientation.z,
      point.object_pose.orientation.w);

    object_tool_orientation.normalize();

    const tf2::Transform object_tool_transform(
      object_tool_orientation,
      tf2::Vector3(
        point.object_pose.position.x,
        point.object_pose.position.y,
        point.object_pose.position.z));

    const tf2::Transform world_tool_transform =
      object_transform *
      object_tool_transform;

    world_poses.push_back(
      transformToPose(world_tool_transform));
  }

  return world_poses;
}

std::shared_ptr<moveit::core::RobotState>
trajectoryEndState(
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::shared_ptr<moveit::core::RobotState>& seed_state,
  const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  const auto& joint_trajectory =
    trajectory.joint_trajectory;

  if (joint_trajectory.points.empty())
  {
    throw std::runtime_error(
      "Trajectory contains no joint points");
  }

  const auto& final_point =
    joint_trajectory.points.back();

  if (
    joint_trajectory.joint_names.size() !=
    final_point.positions.size())
  {
    throw std::runtime_error(
      "Trajectory has mismatched joint names and positions");
  }

  std::shared_ptr<moveit::core::RobotState> state;

  if (seed_state)
  {
    state =
      std::make_shared<moveit::core::RobotState>(
        *seed_state);
  }
  else
  {
    state =
      std::make_shared<moveit::core::RobotState>(
        robot_model);

    state->setToDefaultValues();
  }

  state->setVariablePositions(
    joint_trajectory.joint_names,
    final_point.positions);

  state->update();

  return state;
}

void printPose(
  const rclcpp::Logger& logger,
  const std::string& label,
  const geometry_msgs::msg::Pose& pose)
{
  RCLCPP_INFO(
    logger,
    "%s position xyz=[%.6f, %.6f, %.6f]",
    label.c_str(),
    pose.position.x,
    pose.position.y,
    pose.position.z);

  RCLCPP_INFO(
    logger,
    "%s quaternion xyzw=[%.6f, %.6f, %.6f, %.6f]",
    label.c_str(),
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
}

}  // namespace


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    rclcpp::Node::make_shared(
      "surface_toolpath_planner");

  const auto logger = node->get_logger();

  try
  {
    const std::string package_share =
      ament_index_cpp::get_package_share_directory(
        "ur5e_moveit_cpp");

    const std::string default_csv_path =
      package_share +
      "/data/toolpaths/saddle/raster_45_deg.csv";

    const std::string csv_path =
      node->declare_parameter<std::string>(
        "csv_path",
        default_csv_path);

    const std::string planning_group =
      node->declare_parameter<std::string>(
        "planning_group",
        "ur_manipulator");

    const std::string tcp_link =
      node->declare_parameter<std::string>(
        "tcp_link",
        "probe_tcp");

    const std::string planning_frame =
      node->declare_parameter<std::string>(
        "planning_frame",
        "world");

    const std::int64_t selected_line_id =
      node->declare_parameter<std::int64_t>(
        "line_id",
        0);

    const double object_x =
      node->declare_parameter<double>(
        "object_x",
        0.55);

    const double object_y =
      node->declare_parameter<double>(
        "object_y",
        0.00);

    const double object_z =
      node->declare_parameter<double>(
        "object_z",
        0.10);

    const double object_roll_degrees =
      node->declare_parameter<double>(
        "object_roll_deg",
        0.0);

    const double object_pitch_degrees =
      node->declare_parameter<double>(
        "object_pitch_deg",
        0.0);

    const double object_yaw_degrees =
      node->declare_parameter<double>(
        "object_yaw_deg",
        0.0);

    const double planning_time =
      node->declare_parameter<double>(
        "planning_time",
        15.0);

    const std::int64_t planning_attempts =
      node->declare_parameter<std::int64_t>(
        "planning_attempts",
        20);

    const double velocity_scale =
      node->declare_parameter<double>(
        "velocity_scale",
        0.10);

    const double acceleration_scale =
      node->declare_parameter<double>(
        "acceleration_scale",
        0.10);

    const double eef_step =
      node->declare_parameter<double>(
        "eef_step",
        0.001);

    const double jump_threshold =
      node->declare_parameter<double>(
        "jump_threshold",
        0.0);

    const double minimum_fraction =
      node->declare_parameter<double>(
        "minimum_fraction",
        0.999);

    if (planning_time <= 0.0)
    {
      throw std::runtime_error(
        "planning_time must be positive");
    }

    if (planning_attempts < 1)
    {
      throw std::runtime_error(
        "planning_attempts must be at least 1");
    }

    if (
      velocity_scale <= 0.0 ||
      velocity_scale > 1.0)
    {
      throw std::runtime_error(
        "velocity_scale must be in (0, 1]");
    }

    if (
      acceleration_scale <= 0.0 ||
      acceleration_scale > 1.0)
    {
      throw std::runtime_error(
        "acceleration_scale must be in (0, 1]");
    }

    if (eef_step <= 0.0)
    {
      throw std::runtime_error(
        "eef_step must be positive");
    }

    if (jump_threshold < 0.0)
    {
      throw std::runtime_error(
        "jump_threshold cannot be negative");
    }

    if (
      minimum_fraction <= 0.0 ||
      minimum_fraction > 1.0)
    {
      throw std::runtime_error(
        "minimum_fraction must be in (0, 1]");
    }

    std::map<int, std::vector<CsvToolpathPoint>>
      object_lines =
        loadToolpathCsv(csv_path);

    RCLCPP_INFO(
      logger,
      "Loaded %zu lines from %s",
      object_lines.size(),
      csv_path.c_str());

    std::vector<int> lines_to_plan;

    if (selected_line_id >= 0)
    {
      const int line_id =
        static_cast<int>(selected_line_id);

      if (object_lines.count(line_id) == 0)
      {
        std::stringstream available;

        for (const auto& entry : object_lines)
        {
          available << entry.first << " ";
        }

        throw std::runtime_error(
          "Requested line_id " +
          std::to_string(line_id) +
          " does not exist. Available line IDs: " +
          available.str());
      }

      lines_to_plan.push_back(line_id);
    }
    else
    {
      for (const auto& entry : object_lines)
      {
        lines_to_plan.push_back(entry.first);
      }
    }

    tf2::Quaternion object_orientation;

    object_orientation.setRPY(
      degreesToRadians(object_roll_degrees),
      degreesToRadians(object_pitch_degrees),
      degreesToRadians(object_yaw_degrees));

    object_orientation.normalize();

    const tf2::Transform object_transform(
      object_orientation,
      tf2::Vector3(
        object_x,
        object_y,
        object_z));

    RCLCPP_INFO(
      logger,
      "Object pose in %s: "
      "xyz=[%.3f, %.3f, %.3f], "
      "rpy_deg=[%.1f, %.1f, %.1f]",
      planning_frame.c_str(),
      object_x,
      object_y,
      object_z,
      object_roll_degrees,
      object_pitch_degrees,
      object_yaw_degrees);

    moveit::planning_interface::MoveGroupInterface
      move_group(
        node,
        planning_group);

    move_group.setEndEffectorLink(tcp_link);
    move_group.setPoseReferenceFrame(
      planning_frame);

    move_group.setPlanningTime(planning_time);

    move_group.setNumPlanningAttempts(
      static_cast<int>(planning_attempts));

    move_group.setMaxVelocityScalingFactor(
      velocity_scale);

    move_group.setMaxAccelerationScalingFactor(
      acceleration_scale);

    rclcpp::QoS display_qos(1);
    display_qos.reliable();
    display_qos.transient_local();

    auto display_publisher =
      node->create_publisher<
        moveit_msgs::msg::DisplayTrajectory>(
          "/display_planned_path",
          display_qos);

    moveit_msgs::msg::DisplayTrajectory
      display_trajectory;

    display_trajectory.model_id =
      move_group.getRobotModel()->getName();

    std::shared_ptr<moveit::core::RobotState>
      planned_state;

    std::size_t successful_lines = 0;

    for (const int line_id : lines_to_plan)
    {
      const auto& object_points =
        object_lines.at(line_id);

      if (object_points.size() < 2)
      {
        RCLCPP_ERROR(
          logger,
          "Line %d contains fewer than two points",
          line_id);

        break;
      }

      const std::vector<geometry_msgs::msg::Pose>
        world_waypoints =
          transformLineToWorld(
            object_points,
            object_transform);

      const geometry_msgs::msg::Pose&
        approach_target =
          world_waypoints.front();

      RCLCPP_INFO(
        logger,
        "==================================================");

      RCLCPP_INFO(
        logger,
        "Planning line %d with %zu path points",
        line_id,
        world_waypoints.size());

      printPose(
        logger,
        "Approach target",
        approach_target);

      if (planned_state)
      {
        move_group.setStartState(
          *planned_state);
      }
      else
      {
        move_group.setStartStateToCurrentState();
      }

      move_group.setPoseTarget(
        approach_target,
        tcp_link);

      moveit::planning_interface::
        MoveGroupInterface::Plan approach_plan;

      RCLCPP_INFO(
        logger,
        "Planning collision-aware OMPL transition "
        "to line %d...",
        line_id);

      const auto approach_result =
        move_group.plan(approach_plan);

      move_group.clearPoseTargets();

      if (
        approach_result !=
        moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_ERROR(
          logger,
          "Approach planning failed for line %d",
          line_id);

        break;
      }

      if (
        approach_plan.trajectory
        .joint_trajectory.points.empty())
      {
        RCLCPP_ERROR(
          logger,
          "Approach trajectory for line %d is empty",
          line_id);

        break;
      }

      RCLCPP_INFO(
        logger,
        "Approach succeeded with %zu trajectory points",
        approach_plan.trajectory
          .joint_trajectory.points.size());

      if (display_trajectory.trajectory.empty())
      {
        display_trajectory.trajectory_start =
          approach_plan.start_state;
      }

      display_trajectory.trajectory.push_back(
        approach_plan.trajectory);

      std::shared_ptr<moveit::core::RobotState>
        approach_end_state =
          trajectoryEndState(
            move_group.getRobotModel(),
            planned_state,
            approach_plan.trajectory);

      move_group.setStartState(
        *approach_end_state);

      std::vector<geometry_msgs::msg::Pose>
        cartesian_waypoints;

      cartesian_waypoints.reserve(
        world_waypoints.size() - 1);

      for (
        std::size_t index = 1;
        index < world_waypoints.size();
        ++index)
      {
        cartesian_waypoints.push_back(
          world_waypoints[index]);
      }

      moveit_msgs::msg::RobotTrajectory
        cartesian_trajectory;

      RCLCPP_INFO(
        logger,
        "Computing collision-aware Cartesian path "
        "for line %d with %zu waypoints...",
        line_id,
        cartesian_waypoints.size());

      const double fraction =
        move_group.computeCartesianPath(
          cartesian_waypoints,
          eef_step,
          jump_threshold,
          cartesian_trajectory,
          true);

      RCLCPP_INFO(
        logger,
        "Line %d Cartesian fraction: %.6f",
        line_id,
        fraction);

      RCLCPP_INFO(
        logger,
        "Line %d Cartesian trajectory points: %zu",
        line_id,
        cartesian_trajectory
          .joint_trajectory.points.size());

      if (
        !cartesian_trajectory
          .joint_trajectory.points.empty())
      {
        display_trajectory.trajectory.push_back(
          cartesian_trajectory);
      }

      if (fraction < minimum_fraction)
      {
        RCLCPP_ERROR(
          logger,
          "Line %d was incomplete: %.2f%%. "
          "Stopping without execution.",
          line_id,
          fraction * 100.0);

        break;
      }

      planned_state =
        trajectoryEndState(
          move_group.getRobotModel(),
          approach_end_state,
          cartesian_trajectory);

      ++successful_lines;

      RCLCPP_INFO(
        logger,
        "Line %d planned successfully",
        line_id);
    }

    if (!display_trajectory.trajectory.empty())
    {
      std::this_thread::sleep_for(500ms);

      display_publisher->publish(
        display_trajectory);

      RCLCPP_INFO(
        logger,
        "Published %zu trajectory segments "
        "on /display_planned_path",
        display_trajectory.trajectory.size());

      std::this_thread::sleep_for(2s);
    }

    RCLCPP_INFO(
      logger,
      "==================================================");

    RCLCPP_INFO(
      logger,
      "Successfully planned %zu of %zu requested lines",
      successful_lines,
      lines_to_plan.size());

    RCLCPP_INFO(
      logger,
      "PLAN-ONLY: no robot motion was executed");

    if (successful_lines != lines_to_plan.size())
    {
      rclcpp::shutdown();
      return 1;
    }
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(
      logger,
      "%s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}