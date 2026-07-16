#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class ObjectMeasurement : public rclcpp::Node
{
public:
  ObjectMeasurement()
  : Node("object_measurement")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic",
      "/perception/object_cloud");

    center_topic_ = declare_parameter<std::string>(
      "center_topic",
      "/perception/object_center");

    marker_topic_ = declare_parameter<std::string>(
      "marker_topic",
      "/perception/object_bounding_box");

    const auto qos =
      rclcpp::QoS(rclcpp::KeepLast(5)).reliable();

    center_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(
        center_topic_,
        qos);

    marker_publisher_ =
      create_publisher<visualization_msgs::msg::Marker>(
        marker_topic_,
        qos);

    subscription_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_,
        qos,
        std::bind(
          &ObjectMeasurement::cloudCallback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Object measurement node started");
    RCLCPP_INFO(get_logger(), "Input: %s", input_topic_.c_str());
  }

private:
  using Point = pcl::PointXYZRGB;
  using Cloud = pcl::PointCloud<Point>;

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    auto cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(*message, *cloud);

    if (cloud->empty()) {
      return;
    }

    Point minimum;
    Point maximum;

    pcl::getMinMax3D(*cloud, minimum, maximum);

    const double center_x =
      0.5 * (minimum.x + maximum.x);
    const double center_y =
      0.5 * (minimum.y + maximum.y);
    const double center_z =
      0.5 * (minimum.z + maximum.z);

    const double size_x =
      maximum.x - minimum.x;
    const double size_y =
      maximum.y - minimum.y;
    const double size_z =
      maximum.z - minimum.z;

    // Publish the approximate center of the visible object bounds.
    geometry_msgs::msg::PoseStamped center_message;
    center_message.header = message->header;

    center_message.pose.position.x = center_x;
    center_message.pose.position.y = center_y;
    center_message.pose.position.z = center_z;

    // Orientation is intentionally left as identity.
    // Full object orientation will come from mesh registration.
    center_message.pose.orientation.x = 0.0;
    center_message.pose.orientation.y = 0.0;
    center_message.pose.orientation.z = 0.0;
    center_message.pose.orientation.w = 1.0;

    center_publisher_->publish(center_message);

    // Publish an axis-aligned bounding box for RViz.
    visualization_msgs::msg::Marker marker;
    marker.header = message->header;
    marker.ns = "segmented_object";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = center_message.pose;

    // RViz does not accept zero-sized marker dimensions.
    marker.scale.x = std::max(size_x, 0.001);
    marker.scale.y = std::max(size_y, 0.001);
    marker.scale.z = std::max(size_z, 0.001);

    marker.color.r = 0.1F;
    marker.color.g = 1.0F;
    marker.color.b = 0.1F;
    marker.color.a = 0.30F;

    marker.lifetime = rclcpp::Duration::from_seconds(0.25);

    marker_publisher_->publish(marker);

    frame_count_++;

    if (frame_count_ % 30 == 0) {
      Eigen::Vector4f surface_centroid;
      pcl::compute3DCentroid(*cloud, surface_centroid);

      RCLCPP_INFO(
        get_logger(),
        "Object bounds center [m]: x=%.4f, y=%.4f, z=%.4f",
        center_x,
        center_y,
        center_z);

      RCLCPP_INFO(
        get_logger(),
        "Visible dimensions [m]: x=%.4f, y=%.4f, z=%.4f",
        size_x,
        size_y,
        size_z);

      RCLCPP_INFO(
        get_logger(),
        "Visible surface centroid [m]: x=%.4f, y=%.4f, z=%.4f",
        surface_centroid.x(),
        surface_centroid.y(),
        surface_centroid.z());
    }
  }

  std::string input_topic_;
  std::string center_topic_;
  std::string marker_topic_;

  std::size_t frame_count_{0};

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr center_publisher_;

  rclcpp::Publisher<
    visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectMeasurement>());
  rclcpp::shutdown();
  return 0;
}