#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>


namespace
{

constexpr double kMillimetersToMeters = 0.001;
constexpr double kPi = 3.14159265358979323846;


struct CsvPoint
{
  int line_id{};
  int point_index{};

  tf2::Vector3 tcp_object_m;
  tf2::Quaternion orientation_object;
};


struct LineStatistics
{
  int total{};
  int sequential_success{};
  int recovered_success{};
  int failed{};
  int large_joint_steps{};
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


double radiansToDegrees(double radians)
{
  return radians * 180.0 / kPi;
}


double wrappedAngularDifference(
  double first,
  double second)
{
  const double difference = second - first;

  return std::atan2(
    std::sin(difference),
    std::cos(difference));
}


std::vector<CsvPoint> loadToolpathCsv(
  const std::string& csv_path)
{
  std::ifstream input(csv_path);

  if (!input.is_open())
  {
    throw std::runtime_error(
      "Could not open CSV: " + csv_path);
  }

  std::string header_line;

  if (!std::getline(input, header_line))
  {
    throw std::runtime_error(
      "CSV is empty: " + csv_path);
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

  for (std::size_t index = 0;
       index < headers.size();
       ++index)
  {
    header_indices[headers[index]] = index;
  }

  const std::vector<std::string> required = {
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

  for (const std::string& name : required)
  {
    if (header_indices.count(name) == 0)
    {
      throw std::runtime_error(
        "CSV is missing column: " + name);
    }
  }

  std::vector<CsvPoint> points;
  std::string line;
  std::size_t row_number = 1;

  while (std::getline(input, line))
  {
    ++row_number;

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
        "Unexpected field count on CSV row " +
        std::to_string(row_number));
    }

    auto value =
      [&values, &header_indices](
        const std::string& name) -> const std::string&
      {
        return values.at(header_indices.at(name));
      };

    CsvPoint point;

    point.line_id =
      std::stoi(value("line_id"));

    point.point_index =
      std::stoi(value("point_index"));

    point.tcp_object_m = tf2::Vector3(
      std::stod(value("tcp_x_mm")) *
        kMillimetersToMeters,
      std::stod(value("tcp_y_mm")) *
        kMillimetersToMeters,
      std::stod(value("tcp_z_mm")) *
        kMillimetersToMeters);

    point.orientation_object = tf2::Quaternion(
      std::stod(value("qx")),
      std::stod(value("qy")),
      std::stod(value("qz")),
      std::stod(value("qw")));

    if (
      point.orientation_object.length2() <
      1e-12)
    {
      throw std::runtime_error(
        "Zero-length quaternion on CSV row " +
        std::to_string(row_number));
    }

    point.orientation_object.normalize();

    points.push_back(point);
  }

  if (points.empty())
  {
    throw std::runtime_error(
      "CSV contains no path points.");
  }

  return points;
}


geometry_msgs::msg::Pose transformPointToPose(
  const CsvPoint& point,
  const tf2::Transform& planning_from_object)
{
  const tf2::Transform object_from_tcp(
    point.orientation_object,
    point.tcp_object_m);

  const tf2::Transform planning_from_tcp =
    planning_from_object * object_from_tcp;

  const tf2::Vector3 translation =
    planning_from_tcp.getOrigin();

  tf2::Quaternion orientation =
    planning_from_tcp.getRotation();

  orientation.normalize();

  geometry_msgs::msg::Pose pose;

  pose.position.x = translation.x();
  pose.position.y = translation.y();
  pose.position.z = translation.z();

  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();

  return pose;
}


void calculateFkResidual(
  const moveit::core::RobotState& state,
  const std::string& tcp_link,
  const geometry_msgs::msg::Pose& target_pose,
  double& position_error,
  double& orientation_error)
{
  const Eigen::Isometry3d& achieved =
    state.getGlobalLinkTransform(tcp_link);

  const Eigen::Vector3d target_position(
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z);

  position_error =
    (achieved.translation() - target_position).norm();

  Eigen::Quaterniond achieved_orientation(
    achieved.rotation());

  Eigen::Quaterniond target_orientation(
    target_pose.orientation.w,
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z);

  achieved_orientation.normalize();
  target_orientation.normalize();

  Eigen::Quaterniond difference =
    target_orientation.conjugate() *
    achieved_orientation;

  difference.normalize();

  orientation_error =
    Eigen::AngleAxisd(difference).angle();

  if (orientation_error > kPi)
  {
    orientation_error =
      2.0 * kPi - orientation_error;
  }
}


}  // namespace


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "check_surface_toolpath_ik");

  const auto logger = node->get_logger();

  const std::string package_share =
    ament_index_cpp::get_package_share_directory(
      "ur5e_moveit_cpp");

  const std::string default_csv =
    package_share +
    "/data/toolpaths/front_raster.csv";

  const std::string csv_path =
    node->declare_parameter<std::string>(
      "csv_path",
      default_csv);

  const std::string planning_group =
    node->declare_parameter<std::string>(
      "planning_group",
      "ur_manipulator");

  const std::string tcp_link =
    node->declare_parameter<std::string>(
      "tcp_link",
      "probe_tcp");

  const double object_x =
    node->declare_parameter<double>(
      "object_x",
      0.60);

  const double object_y =
    node->declare_parameter<double>(
      "object_y",
      0.00);

  const double object_z =
    node->declare_parameter<double>(
      "object_z",
      0.10);

  const double object_roll =
    degreesToRadians(
      node->declare_parameter<double>(
        "object_roll_deg",
        0.0));

  const double object_pitch =
    degreesToRadians(
      node->declare_parameter<double>(
        "object_pitch_deg",
        0.0));

  const double object_yaw =
    degreesToRadians(
      node->declare_parameter<double>(
        "object_yaw_deg",
        90.0));

  const double ik_timeout =
    node->declare_parameter<double>(
      "ik_timeout",
      0.10);

  const double maximum_joint_step =
    node->declare_parameter<double>(
      "max_joint_step",
      0.35);

  if (ik_timeout <= 0.0)
  {
    RCLCPP_ERROR(
      logger,
      "ik_timeout must be positive.");

    rclcpp::shutdown();
    return 1;
  }

  if (maximum_joint_step <= 0.0)
  {
    RCLCPP_ERROR(
      logger,
      "max_joint_step must be positive.");

    rclcpp::shutdown();
    return 1;
  }

  std::vector<CsvPoint> path_points;

  try
  {
    path_points = loadToolpathCsv(csv_path);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(
      logger,
      "%s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }

  tf2::Quaternion object_orientation;
  object_orientation.setRPY(
    object_roll,
    object_pitch,
    object_yaw);
  object_orientation.normalize();

  const tf2::Transform planning_from_object(
    object_orientation,
    tf2::Vector3(
      object_x,
      object_y,
      object_z));

  RCLCPP_INFO(
    logger,
    "Loaded %zu path poses from %s",
    path_points.size(),
    csv_path.c_str());

  RCLCPP_INFO(
    logger,
    "Object pose: xyz=[%.3f, %.3f, %.3f]",
    object_x,
    object_y,
    object_z);

  RCLCPP_INFO(
    logger,
    "Planning group: %s",
    planning_group.c_str());

  RCLCPP_INFO(
    logger,
    "Requested TCP link: %s",
    tcp_link.c_str());

  RCLCPP_INFO(
    logger,
    "IK timeout per attempt: %.3f seconds",
    ik_timeout);

  RCLCPP_INFO(
    logger,
    "Maximum allowed within-line joint step: "
    "%.3f rad (%.1f deg)",
    maximum_joint_step,
    radiansToDegrees(maximum_joint_step));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner(
    [&executor]()
    {
      executor.spin();
    });

  int exit_code = 1;

  try
  {
    moveit::planning_interface::MoveGroupInterface
      move_group(
        node,
        planning_group);

    const moveit::core::RobotModelConstPtr robot_model =
      move_group.getRobotModel();

    if (!robot_model)
    {
      throw std::runtime_error(
        "MoveIt did not provide a robot model.");
    }

    if (!robot_model->hasLinkModel(tcp_link))
    {
      throw std::runtime_error(
        "Robot model does not contain TCP link: " +
        tcp_link);
    }

    const moveit::core::JointModelGroup*
      joint_model_group =
        robot_model->getJointModelGroup(
          planning_group);

    if (joint_model_group == nullptr)
    {
      throw std::runtime_error(
        "Robot model does not contain planning group: " +
        planning_group);
    }

    move_group.setEndEffectorLink(tcp_link);

    RCLCPP_INFO(
      logger,
      "MoveIt planning frame: %s",
      move_group.getPlanningFrame().c_str());

    RCLCPP_INFO(
      logger,
      "MoveIt end-effector link: %s",
      move_group.getEndEffectorLink().c_str());

    RCLCPP_INFO(
      logger,
      "Waiting for the current mock robot state...");

    moveit::core::RobotStatePtr current_state =
      move_group.getCurrentState(10.0);

    if (!current_state)
    {
      throw std::runtime_error(
        "Could not obtain the current robot state.");
    }

    current_state->update();

    moveit::core::RobotState initial_state(
      *current_state);

    moveit::core::RobotState sequential_seed(
      *current_state);

    std::vector<double> previous_joint_values;
    bool have_previous_within_line = false;
    int previous_line_id = -1;

    int sequential_successes = 0;
    int recovered_successes = 0;
    int complete_failures = 0;
    int large_joint_steps = 0;

    double maximum_observed_joint_step = 0.0;
    double maximum_position_residual = 0.0;
    double maximum_orientation_residual = 0.0;

    std::map<int, LineStatistics> line_statistics;

    for (const CsvPoint& point : path_points)
    {
      LineStatistics& line_stats =
        line_statistics[point.line_id];

      ++line_stats.total;

      if (point.line_id != previous_line_id)
      {
        RCLCPP_INFO(
          logger,
          "Checking raster line %d...",
          point.line_id);

        previous_line_id = point.line_id;
        have_previous_within_line = false;
        previous_joint_values.clear();
      }

      const geometry_msgs::msg::Pose target_pose =
        transformPointToPose(
          point,
          planning_from_object);

      moveit::core::RobotState candidate(
        sequential_seed);

      bool sequential_success =
        candidate.setFromIK(
          joint_model_group,
          target_pose,
          tcp_link,
          ik_timeout);

      bool recovered_from_initial = false;

      if (!sequential_success)
      {
        moveit::core::RobotState fallback_candidate(
          initial_state);

        const bool fallback_success =
          fallback_candidate.setFromIK(
            joint_model_group,
            target_pose,
            tcp_link,
            ik_timeout);

        if (fallback_success)
        {
          candidate = fallback_candidate;
          recovered_from_initial = true;

          ++recovered_successes;
          ++line_stats.recovered_success;

          RCLCPP_WARN(
            logger,
            "Line %d point %d: sequential seed failed, "
            "but IK succeeded from the initial state. "
            "Possible IK-branch discontinuity.",
            point.line_id,
            point.point_index);
        }
        else
        {
          ++complete_failures;
          ++line_stats.failed;

          RCLCPP_ERROR(
            logger,
            "Line %d point %d: no IK solution found.",
            point.line_id,
            point.point_index);

          have_previous_within_line = false;
          previous_joint_values.clear();
          continue;
        }
      }
      else
      {
        ++sequential_successes;
        ++line_stats.sequential_success;
      }

      candidate.update();

      std::vector<double> joint_values;
      candidate.copyJointGroupPositions(
        joint_model_group,
        joint_values);

      if (
        have_previous_within_line &&
        previous_joint_values.size() ==
          joint_values.size())
      {
        double largest_step_for_pose = 0.0;
        std::size_t largest_step_joint = 0;

        for (std::size_t joint_index = 0;
             joint_index < joint_values.size();
             ++joint_index)
        {
          const double step = std::abs(
            wrappedAngularDifference(
              previous_joint_values[joint_index],
              joint_values[joint_index]));

          if (step > largest_step_for_pose)
          {
            largest_step_for_pose = step;
            largest_step_joint = joint_index;
          }
        }

        maximum_observed_joint_step =
          std::max(
            maximum_observed_joint_step,
            largest_step_for_pose);

        if (
          largest_step_for_pose >
          maximum_joint_step)
        {
          ++large_joint_steps;
          ++line_stats.large_joint_steps;

          const std::vector<std::string>&
            active_joint_names =
              joint_model_group
              ->getActiveJointModelNames();

          const std::string joint_name =
            largest_step_joint <
              active_joint_names.size()
            ? active_joint_names[largest_step_joint]
            : "unknown_joint";

          RCLCPP_WARN(
            logger,
            "Line %d point %d: large joint step "
            "%.4f rad (%.2f deg) on %s.",
            point.line_id,
            point.point_index,
            largest_step_for_pose,
            radiansToDegrees(
              largest_step_for_pose),
            joint_name.c_str());
        }
      }

      double position_residual = 0.0;
      double orientation_residual = 0.0;

      calculateFkResidual(
        candidate,
        tcp_link,
        target_pose,
        position_residual,
        orientation_residual);

      maximum_position_residual =
        std::max(
          maximum_position_residual,
          position_residual);

      maximum_orientation_residual =
        std::max(
          maximum_orientation_residual,
          orientation_residual);

      previous_joint_values = joint_values;
      have_previous_within_line = true;
      sequential_seed = candidate;

      if (recovered_from_initial)
      {
        // The fallback solution becomes the seed for subsequent
        // points so the diagnostic can continue.
        sequential_seed = candidate;
      }
    }

    RCLCPP_INFO(
      logger,
      "================================================");

    RCLCPP_INFO(
      logger,
      "SEQUENTIAL IK SUMMARY");

    RCLCPP_INFO(
      logger,
      "Total poses: %zu",
      path_points.size());

    RCLCPP_INFO(
      logger,
      "Sequential IK successes: %d",
      sequential_successes);

    RCLCPP_INFO(
      logger,
      "Recovered from initial-state seed: %d",
      recovered_successes);

    RCLCPP_INFO(
      logger,
      "Complete IK failures: %d",
      complete_failures);

    RCLCPP_INFO(
      logger,
      "Large within-line joint steps: %d",
      large_joint_steps);

    RCLCPP_INFO(
      logger,
      "Maximum observed joint step: %.6f rad "
      "(%.3f deg)",
      maximum_observed_joint_step,
      radiansToDegrees(
        maximum_observed_joint_step));

    RCLCPP_INFO(
      logger,
      "Maximum FK position residual: %.9f m",
      maximum_position_residual);

    RCLCPP_INFO(
      logger,
      "Maximum FK orientation residual: "
      "%.9f rad (%.6f deg)",
      maximum_orientation_residual,
      radiansToDegrees(
        maximum_orientation_residual));

    for (const auto& entry : line_statistics)
    {
      const int line_id = entry.first;
      const LineStatistics& stats = entry.second;

      RCLCPP_INFO(
        logger,
        "Line %d: total=%d sequential=%d "
        "recovered=%d failed=%d large_steps=%d",
        line_id,
        stats.total,
        stats.sequential_success,
        stats.recovered_success,
        stats.failed,
        stats.large_joint_steps);
    }

    if (
      complete_failures == 0 &&
      recovered_successes == 0 &&
      large_joint_steps == 0)
    {
      RCLCPP_INFO(
        logger,
        "PASSED: every pose has continuous sequential IK.");

      exit_code = 0;
    }
    else
    {
      RCLCPP_WARN(
        logger,
        "DIAGNOSTIC DID NOT FULLY PASS. "
        "No motion was executed.");

      exit_code = 2;
    }
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(
      logger,
      "%s",
      error.what());

    exit_code = 1;
  }

  executor.cancel();

  if (spinner.joinable())
  {
    spinner.join();
  }

  rclcpp::shutdown();
  return exit_code;
}