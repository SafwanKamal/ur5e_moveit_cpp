#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudPreprocessor : public rclcpp::Node
{
public:
  PointCloudPreprocessor()
  : Node("pointcloud_preprocessor")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic",
      "/camera/camera/depth/color/points");

    output_topic_ = declare_parameter<std::string>(
      "output_topic",
      "/perception/cleaned_cloud");

    // In an optical frame:
    // x points right, y points down, z points forward.
    min_depth_ = declare_parameter<double>("min_depth", 0.30);
    max_depth_ = declare_parameter<double>("max_depth", 1.00);

    x_min_ = declare_parameter<double>("x_min", -0.40);
    x_max_ = declare_parameter<double>("x_max", 0.40);

    y_min_ = declare_parameter<double>("y_min", -0.30);
    y_max_ = declare_parameter<double>("y_max", 0.30);

    voxel_size_ = declare_parameter<double>("voxel_size", 0.003);

    mean_k_ = declare_parameter<int>("mean_k", 20);
    stddev_multiplier_ =
      declare_parameter<double>("stddev_multiplier", 1.0);

    publisher_ =
    create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_,
        rclcpp::QoS(rclcpp::KeepLast(5)).reliable());

    subscription_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
          &PointCloudPreprocessor::cloudCallback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Point-cloud preprocessor started");
    RCLCPP_INFO(get_logger(), "Input:  %s", input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output: %s", output_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Depth range: %.3f to %.3f m",
      min_depth_,
      max_depth_);
    RCLCPP_INFO(
      get_logger(),
      "Voxel size: %.4f m",
      voxel_size_);
  }

private:
  using Point = pcl::PointXYZRGB;
  using Cloud = pcl::PointCloud<Point>;

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    auto input_cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(*message, *input_cloud);

    if (input_cloud->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Received an empty point cloud");
      return;
    }

    // Remove NaN and infinite points.
    auto finite_cloud = std::make_shared<Cloud>();
    std::vector<int> valid_indices;

    pcl::removeNaNFromPointCloud(
      *input_cloud,
      *finite_cloud,
      valid_indices);

    if (finite_cloud->empty()) {
      return;
    }

    // Crop in camera optical coordinates.
    pcl::CropBox<Point> crop_filter;
    crop_filter.setInputCloud(finite_cloud);

    crop_filter.setMin(
      Eigen::Vector4f(
        static_cast<float>(x_min_),
        static_cast<float>(y_min_),
        static_cast<float>(min_depth_),
        1.0F));

    crop_filter.setMax(
      Eigen::Vector4f(
        static_cast<float>(x_max_),
        static_cast<float>(y_max_),
        static_cast<float>(max_depth_),
        1.0F));

    auto cropped_cloud = std::make_shared<Cloud>();
    crop_filter.filter(*cropped_cloud);

    if (cropped_cloud->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "No points remain after workspace cropping");
      return;
    }

    // Downsample the point cloud.
    pcl::VoxelGrid<Point> voxel_filter;
    voxel_filter.setInputCloud(cropped_cloud);

    const float leaf = static_cast<float>(voxel_size_);
    voxel_filter.setLeafSize(leaf, leaf, leaf);

    auto downsampled_cloud = std::make_shared<Cloud>();
    voxel_filter.filter(*downsampled_cloud);

    if (downsampled_cloud->empty()) {
      return;
    }

    // Remove isolated points.
    auto cleaned_cloud = std::make_shared<Cloud>();

    if (
      mean_k_ > 1 &&
      downsampled_cloud->size() >
      static_cast<std::size_t>(mean_k_))
    {
      pcl::StatisticalOutlierRemoval<Point> outlier_filter;
      outlier_filter.setInputCloud(downsampled_cloud);
      outlier_filter.setMeanK(mean_k_);
      outlier_filter.setStddevMulThresh(stddev_multiplier_);
      outlier_filter.filter(*cleaned_cloud);
    } else {
      *cleaned_cloud = *downsampled_cloud;
    }

    if (cleaned_cloud->empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud2 output_message;
    pcl::toROSMsg(*cleaned_cloud, output_message);

    // Preserve the input coordinate frame and timestamp.
    output_message.header = message->header;

    publisher_->publish(output_message);

    frame_count_++;

    if (frame_count_ % 30 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "Points: input=%zu, finite=%zu, cropped=%zu, "
        "downsampled=%zu, cleaned=%zu",
        input_cloud->size(),
        finite_cloud->size(),
        cropped_cloud->size(),
        downsampled_cloud->size(),
        cleaned_cloud->size());
    }
  }

  std::string input_topic_;
  std::string output_topic_;

  double min_depth_;
  double max_depth_;

  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;

  double voxel_size_;
  int mean_k_;
  double stddev_multiplier_;

  std::size_t frame_count_{0};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    publisher_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<PointCloudPreprocessor>());

  rclcpp::shutdown();
  return 0;
}