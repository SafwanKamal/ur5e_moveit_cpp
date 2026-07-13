cmake_minimum_required(VERSION 3.8)
project(ur5e_moveit_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometric_shapes REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



add_executable(simple_joint_motion src/simple_joint_motion.cpp)
ament_target_dependencies(simple_joint_motion
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

add_executable(simple_pose_motion src/simple_pose_motion.cpp)
ament_target_dependencies(simple_pose_motion
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

add_executable(plan_only_trajectory src/plan_only_trajectory.cpp)
ament_target_dependencies(plan_only_trajectory
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

add_executable(save_trajectory_csv src/save_trajectory_csv.cpp)
ament_target_dependencies(save_trajectory_csv
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

add_executable(analyze_trajectory src/analyze_trajectory.cpp)
ament_target_dependencies(analyze_trajectory
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

add_executable(compare_pose_candidates src/compare_pose_candidates.cpp)
ament_target_dependencies(compare_pose_candidates
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

add_executable(add_box_only src/add_box_only.cpp)
ament_target_dependencies(add_box_only
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

add_executable(collision_object_demo src/collision_object_demo.cpp)
ament_target_dependencies(collision_object_demo
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

add_executable(collision_aware_candidate_planner src/collision_aware_candidate_planner.cpp)
ament_target_dependencies(collision_aware_candidate_planner
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

add_executable(remove_all_objects src/remove_all_objects.cpp)
ament_target_dependencies(remove_all_objects
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

add_executable(parameterized_shape_adder src/parameterized_shape_adder.cpp)
ament_target_dependencies(parameterized_shape_adder
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  shape_msgs
)

add_executable(hemisphere_surface_demo src/hemisphere_surface_demo.cpp)
ament_target_dependencies(hemisphere_surface_demo
  rclcpp
  geometry_msgs
  moveit_msgs
  shape_msgs
  moveit_ros_planning_interface
  moveit_core
)

add_executable(
  visualize_surface_toolpath
  src/visualize_surface_toolpath.cpp
)

ament_target_dependencies(
  visualize_surface_toolpath
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  ament_index_cpp
)

add_executable(
  check_surface_toolpath_ik
  src/check_surface_toolpath_ik.cpp
)

ament_target_dependencies(
  check_surface_toolpath_ik
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  tf2
  ament_index_cpp
)

add_executable(
  add_mesh_collision_object
  src/add_mesh_collision_object.cpp
)

ament_target_dependencies(
  add_mesh_collision_object
  rclcpp
  geometry_msgs
  moveit_msgs
  shape_msgs
  tf2
  geometric_shapes
)

add_executable(
  plan_surface_toolpath
  src/plan_surface_toolpath.cpp
)

ament_target_dependencies(
  plan_surface_toolpath
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_core
  tf2
  ament_index_cpp
)

add_executable(
  plan_execute_surface_toolpath
  src/plan_execute_surface_toolpath.cpp
)

ament_target_dependencies(
  plan_execute_surface_toolpath
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_core
  tf2
  ament_index_cpp
)

install(TARGETS
  simple_joint_motion
  simple_pose_motion
  plan_only_trajectory
  save_trajectory_csv
  analyze_trajectory
  compare_pose_candidates
  add_box_only
  collision_object_demo
  collision_aware_candidate_planner
  remove_all_objects
  parameterized_shape_adder
  hemisphere_surface_demo
  visualize_surface_toolpath
  check_surface_toolpath_ik
  add_mesh_collision_object
  plan_surface_toolpath
  plan_execute_surface_toolpath
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY data meshes
  DESTINATION share/${PROJECT_NAME}
)