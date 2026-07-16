cmake_minimum_required(VERSION 3.8)
project(ur5e_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(PCL REQUIRED COMPONENTS
  common
  filters
  search
  segmentation
)

add_executable(
  pointcloud_preprocessor
  src/pointcloud_preprocessor.cpp
)

target_compile_features(
  pointcloud_preprocessor
  PUBLIC cxx_std_17
)

target_include_directories(
  pointcloud_preprocessor
  PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(
  pointcloud_preprocessor
  ${PCL_LIBRARIES}
)

target_compile_definitions(
  pointcloud_preprocessor
  PRIVATE
  ${PCL_DEFINITIONS}
)

ament_target_dependencies(
  pointcloud_preprocessor
  rclcpp
  sensor_msgs
  pcl_conversions
)

add_executable(
  table_object_segmenter
  src/table_object_segmenter.cpp
)

target_compile_features(
  table_object_segmenter
  PUBLIC cxx_std_17
)

target_include_directories(
  table_object_segmenter
  PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(
  table_object_segmenter
  ${PCL_LIBRARIES}
)

target_compile_definitions(
  table_object_segmenter
  PRIVATE
  ${PCL_DEFINITIONS}
)

ament_target_dependencies(
  table_object_segmenter
  rclcpp
  sensor_msgs
  pcl_conversions
)

add_executable(
  object_measurement
  src/object_measurement.cpp
)

target_compile_features(
  object_measurement
  PUBLIC cxx_std_17
)

target_include_directories(
  object_measurement
  PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(
  object_measurement
  ${PCL_LIBRARIES}
)

target_compile_definitions(
  object_measurement
  PRIVATE
  ${PCL_DEFINITIONS}
)

ament_target_dependencies(
  object_measurement
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
)

install(
  TARGETS 
    pointcloud_preprocessor 
    table_object_segmenter
    object_measurement
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()