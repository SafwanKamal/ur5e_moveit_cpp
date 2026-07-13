#include <boost/variant/get.hpp>

#include <Eigen/Core>

#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

namespace
{

constexpr double kPi = 3.14159265358979323846;

double degreesToRadians(double degrees)
{
  return degrees * kPi / 180.0;
}

geometry_msgs::msg::Pose makePose(
  double x,
  double y,
  double z,
  double roll_degrees,
  double pitch_degrees,
  double yaw_degrees)
{
  tf2::Quaternion orientation;

  orientation.setRPY(
    degreesToRadians(roll_degrees),
    degreesToRadians(pitch_degrees),
    degreesToRadians(yaw_degrees));

  orientation.normalize();

  geometry_msgs::msg::Pose pose;

  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();

  return pose;
}

}  // namespace


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "mesh_collision_object_adder");

  try
  {
    node->declare_parameter<std::string>(
      "object_id",
      "surface_object");

    node->declare_parameter<std::string>(
      "frame_id",
      "world");

    node->declare_parameter<std::string>(
      "mesh_resource",
      "package://ur5e_moveit_cpp/meshes/saddle/"
      "saddle_block_100x100.stl");

    node->declare_parameter<double>(
      "mesh_scale",
      0.001);

    node->declare_parameter<double>(
      "object_x",
      0.55);

    node->declare_parameter<double>(
      "object_y",
      0.00);

    node->declare_parameter<double>(
      "object_z",
      0.10);

    node->declare_parameter<double>(
      "object_roll_deg",
      0.0);

    node->declare_parameter<double>(
      "object_pitch_deg",
      0.0);

    node->declare_parameter<double>(
      "object_yaw_deg",
      0.0);

    node->declare_parameter<double>(
      "service_timeout_seconds",
      10.0);

    const std::string object_id =
      node->get_parameter("object_id").as_string();

    const std::string frame_id =
      node->get_parameter("frame_id").as_string();

    const std::string mesh_resource =
      node->get_parameter("mesh_resource").as_string();

    const double mesh_scale =
      node->get_parameter("mesh_scale").as_double();

    const double object_x =
      node->get_parameter("object_x").as_double();

    const double object_y =
      node->get_parameter("object_y").as_double();

    const double object_z =
      node->get_parameter("object_z").as_double();

    const double object_roll_degrees =
      node->get_parameter(
        "object_roll_deg").as_double();

    const double object_pitch_degrees =
      node->get_parameter(
        "object_pitch_deg").as_double();

    const double object_yaw_degrees =
      node->get_parameter(
        "object_yaw_deg").as_double();

    const double service_timeout_seconds =
      node->get_parameter(
        "service_timeout_seconds").as_double();

    if (object_id.empty())
    {
      throw std::runtime_error(
        "object_id cannot be empty");
    }

    if (frame_id.empty())
    {
      throw std::runtime_error(
        "frame_id cannot be empty");
    }

    if (mesh_resource.empty())
    {
      throw std::runtime_error(
        "mesh_resource cannot be empty");
    }

    if (
      !std::isfinite(mesh_scale) ||
      mesh_scale <= 0.0)
    {
      throw std::runtime_error(
        "mesh_scale must be finite and positive");
    }

    if (
      !std::isfinite(service_timeout_seconds) ||
      service_timeout_seconds <= 0.0)
    {
      throw std::runtime_error(
        "service_timeout_seconds must be finite and positive");
    }

    RCLCPP_INFO(
      node->get_logger(),
      "Loading collision mesh: %s",
      mesh_resource.c_str());

    RCLCPP_INFO(
      node->get_logger(),
      "Mesh scale: %.9f",
      mesh_scale);

    const Eigen::Vector3d scale(
      mesh_scale,
      mesh_scale,
      mesh_scale);

    std::unique_ptr<shapes::Mesh> collision_mesh(
      shapes::createMeshFromResource(
        mesh_resource,
        scale));

    if (!collision_mesh)
    {
      throw std::runtime_error(
        "Could not load collision mesh resource: " +
        mesh_resource);
    }

    shapes::ShapeMsg shape_message;

    const bool conversion_success =
      shapes::constructMsgFromShape(
        collision_mesh.get(),
        shape_message);

    if (!conversion_success)
    {
      throw std::runtime_error(
        "Could not convert collision mesh into a ROS message");
    }

    const shape_msgs::msg::Mesh mesh_message =
      boost::get<shape_msgs::msg::Mesh>(
        shape_message);

    const geometry_msgs::msg::Pose object_pose =
      makePose(
        object_x,
        object_y,
        object_z,
        object_roll_degrees,
        object_pitch_degrees,
        object_yaw_degrees);

    moveit_msgs::msg::CollisionObject collision_object;

    collision_object.header.frame_id = frame_id;
    collision_object.id = object_id;

    collision_object.meshes.push_back(
      mesh_message);

    collision_object.mesh_poses.push_back(
      object_pose);

    collision_object.operation =
      moveit_msgs::msg::CollisionObject::ADD;

    moveit_msgs::msg::PlanningScene planning_scene;

    planning_scene.is_diff = true;

    planning_scene.world.collision_objects.push_back(
      collision_object);

    auto apply_scene_client =
      node->create_client<
        moveit_msgs::srv::ApplyPlanningScene>(
          "/apply_planning_scene");

    const auto service_timeout =
      std::chrono::duration_cast<
        std::chrono::nanoseconds>(
          std::chrono::duration<double>(
            service_timeout_seconds));

    RCLCPP_INFO(
      node->get_logger(),
      "Waiting for /apply_planning_scene...");

    if (
      !apply_scene_client->wait_for_service(
        service_timeout))
    {
      throw std::runtime_error(
        "/apply_planning_scene was not available within " +
        std::to_string(service_timeout_seconds) +
        " seconds");
    }

    auto request =
      std::make_shared<
        moveit_msgs::srv::ApplyPlanningScene::Request>();

    request->scene = planning_scene;

    auto future =
      apply_scene_client->async_send_request(
        request);

    const rclcpp::FutureReturnCode return_code =
      rclcpp::spin_until_future_complete(
        node,
        future,
        service_timeout);

    if (
      return_code !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error(
        "Timed out while applying the planning scene");
    }

    const auto response = future.get();

    if (!response->success)
    {
      throw std::runtime_error(
        "MoveIt rejected the planning-scene update");
    }

    RCLCPP_INFO(
      node->get_logger(),
      "Added collision object '%s' in frame '%s'",
      object_id.c_str(),
      frame_id.c_str());

    RCLCPP_INFO(
      node->get_logger(),
      "Pose xyz=[%.3f, %.3f, %.3f], "
      "rpy_deg=[%.1f, %.1f, %.1f]",
      object_x,
      object_y,
      object_z,
      object_roll_degrees,
      object_pitch_degrees,
      object_yaw_degrees);

    RCLCPP_INFO(
      node->get_logger(),
      "Mesh vertices: %u, triangles: %u",
      collision_mesh->vertex_count,
      collision_mesh->triangle_count);
  }
  catch (const boost::bad_get& error)
  {
    RCLCPP_FATAL(
      node->get_logger(),
      "Loaded resource did not convert to "
      "shape_msgs/msg/Mesh: %s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }
  catch (const std::exception& error)
  {
    RCLCPP_FATAL(
      node->get_logger(),
      "%s",
      error.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}