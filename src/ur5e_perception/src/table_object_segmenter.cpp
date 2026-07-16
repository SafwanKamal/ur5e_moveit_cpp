#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

class TableObjectSegmenter : public rclcpp::Node
{
public:
  TableObjectSegmenter()
  : Node("table_object_segmenter")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic",
      "/perception/cleaned_cloud");

    table_topic_ = declare_parameter<std::string>(
      "table_topic",
      "/perception/table_cloud");

    no_table_topic_ = declare_parameter<std::string>(
      "no_table_topic",
      "/perception/no_table_cloud");

    object_topic_ = declare_parameter<std::string>(
      "object_topic",
      "/perception/object_cloud");

    plane_distance_threshold_ = declare_parameter<double>(
      "plane_distance_threshold",
      0.005);

    plane_max_iterations_ = declare_parameter<int>(
      "plane_max_iterations",
      200);

    minimum_plane_points_ = declare_parameter<int>(
      "minimum_plane_points",
      300);

    cluster_tolerance_ = declare_parameter<double>(
      "cluster_tolerance",
      0.015);

    minimum_cluster_points_ = declare_parameter<int>(
      "minimum_cluster_points",
      100);

    maximum_cluster_points_ = declare_parameter<int>(
      "maximum_cluster_points",
      100000);

    const auto output_qos =
      rclcpp::QoS(rclcpp::KeepLast(5)).reliable();

    table_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        table_topic_,
        output_qos);

    no_table_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        no_table_topic_,
        output_qos);

    object_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        object_topic_,
        output_qos);

    subscription_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
          &TableObjectSegmenter::cloudCallback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Table/object segmenter started");
    RCLCPP_INFO(get_logger(), "Input: %s", input_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Plane threshold: %.4f m",
      plane_distance_threshold_);
    RCLCPP_INFO(
      get_logger(),
      "Cluster tolerance: %.4f m",
      cluster_tolerance_);
  }

private:
  using Point = pcl::PointXYZRGB;
  using Cloud = pcl::PointCloud<Point>;

  void publishCloud(
    const Cloud & cloud,
    const std_msgs::msg::Header & header,
    const rclcpp::Publisher<
      sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
  {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header = header;
    publisher->publish(message);
  }

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    auto input_cloud = std::make_shared<Cloud>();
    pcl::fromROSMsg(*message, *input_cloud);

    if (input_cloud->empty()) {
      return;
    }

    // Find the largest planar surface, expected to be the table.
    auto plane_inliers = std::make_shared<pcl::PointIndices>();
    auto plane_coefficients =
      std::make_shared<pcl::ModelCoefficients>();

    pcl::SACSegmentation<Point> plane_segmenter;
    plane_segmenter.setOptimizeCoefficients(true);
    plane_segmenter.setModelType(pcl::SACMODEL_PLANE);
    plane_segmenter.setMethodType(pcl::SAC_RANSAC);
    plane_segmenter.setMaxIterations(plane_max_iterations_);
    plane_segmenter.setDistanceThreshold(
      plane_distance_threshold_);
    plane_segmenter.setInputCloud(input_cloud);

    plane_segmenter.segment(
      *plane_inliers,
      *plane_coefficients);

    if (
      plane_inliers->indices.size() <
      static_cast<std::size_t>(minimum_plane_points_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "No sufficiently large table plane found: %zu inliers",
        plane_inliers->indices.size());
      return;
    }

    pcl::ExtractIndices<Point> extractor;
    extractor.setInputCloud(input_cloud);
    extractor.setIndices(plane_inliers);

    // Extract the table the itself.
    auto table_cloud = std::make_shared<Cloud>();
    extractor.setNegative(false);
    extractor.filter(*table_cloud);

    // Extract everything except the table.
    auto no_table_cloud = std::make_shared<Cloud>();
    extractor.setNegative(true);
    extractor.filter(*no_table_cloud);

    publishCloud(
      *table_cloud,
      message->header,
      table_publisher_);

    publishCloud(
      *no_table_cloud,
      message->header,
      no_table_publisher_);

    if (no_table_cloud->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "No points remain after table removal");
      return;
    }

    // Find connected object clusters.
    auto search_tree =
      std::make_shared<pcl::search::KdTree<Point>>();

    search_tree->setInputCloud(no_table_cloud);

    std::vector<pcl::PointIndices> clusters;

    pcl::EuclideanClusterExtraction<Point> cluster_extractor;
    cluster_extractor.setClusterTolerance(cluster_tolerance_);
    cluster_extractor.setMinClusterSize(minimum_cluster_points_);
    cluster_extractor.setMaxClusterSize(maximum_cluster_points_);
    cluster_extractor.setSearchMethod(search_tree);
    cluster_extractor.setInputCloud(no_table_cloud);
    cluster_extractor.extract(clusters);

    if (clusters.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "No object cluster satisfied the configured limits");
      return;
    }

    // Select the largest cluster for the initial single-object test.
    const auto largest_cluster =
      std::max_element(
        clusters.begin(),
        clusters.end(),
        [](const pcl::PointIndices & first,
           const pcl::PointIndices & second)
        {
          return first.indices.size() < second.indices.size();
        });

    auto object_cloud = std::make_shared<Cloud>();
    object_cloud->reserve(largest_cluster->indices.size());

    for (const int index : largest_cluster->indices) {
      object_cloud->push_back(no_table_cloud->points[index]);
    }

    object_cloud->width =
      static_cast<std::uint32_t>(object_cloud->size());
    object_cloud->height = 1;
    object_cloud->is_dense = true;

    publishCloud(
      *object_cloud,
      message->header,
      object_publisher_);

    frame_count_++;

    if (frame_count_ % 30 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "Points: input=%zu, table=%zu, no_table=%zu, "
        "clusters=%zu, selected_object=%zu",
        input_cloud->size(),
        table_cloud->size(),
        no_table_cloud->size(),
        clusters.size(),
        object_cloud->size());

      if (plane_coefficients->values.size() >= 4) {
        RCLCPP_INFO(
          get_logger(),
          "Table plane: %.4fx + %.4fy + %.4fz + %.4f = 0",
          plane_coefficients->values[0],
          plane_coefficients->values[1],
          plane_coefficients->values[2],
          plane_coefficients->values[3]);
      }
    }
  }

  std::string input_topic_;
  std::string table_topic_;
  std::string no_table_topic_;
  std::string object_topic_;

  double plane_distance_threshold_;
  int plane_max_iterations_;
  int minimum_plane_points_;

  double cluster_tolerance_;
  int minimum_cluster_points_;
  int maximum_cluster_points_;

  std::size_t frame_count_{0};

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr table_publisher_;

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr no_table_publisher_;

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr object_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TableObjectSegmenter>());
  rclcpp::shutdown();
  return 0;
}