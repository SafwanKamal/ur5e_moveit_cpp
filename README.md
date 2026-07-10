# UR5e MoveIt C++ Experiments

ROS 2 Jazzy and MoveIt 2 experiments for planning, analyzing, visualizing, and eventually executing UR5e motions with collision objects and a custom probe/tool TCP.

The repository records the progression from basic joint-space motion to programmatic surface-following paths on simple geometry. The current surface-planning checkpoint uses a spherical workpiece, generated TCP poses, a collision-aware OMPL approach, and a Cartesian ring path.

> **Safety:** several early demonstration nodes execute trajectories immediately after planning. Use mock hardware/RViz first. Review every target, collision object, frame, and speed before connecting to a real robot.

---

## Repository structure

```text
ur5e_moveit_cpp/
├── README.md
├── paths/
│   ├── generate_hemisphere_ring.py
│   ├── hemisphere_ring_start_00.csv
│   ├── hemisphere_ring_start_01.csv
│   └── ...
└── src/
    └── ur5e_moveit_cpp/
        ├── CMakeLists.txt
        ├── package.xml
        ├── data/
        │   └── toolpaths/
        ├── meshes/
        └── src/
            ├── simple_joint_motion.cpp
            ├── simple_pose_motion.cpp
            ├── plan_only_trajectory.cpp
            ├── save_trajectory_csv.cpp
            ├── analyze_trajectory.cpp
            ├── compare_pose_candidates.cpp
            ├── add_box_only.cpp
            ├── collision_object_demo.cpp
            ├── collision_aware_candidate_planner.cpp
            ├── remove_all_objects.cpp
            ├── parameterized_shape_adder.cpp
            ├── hemisphere_surface_demo.cpp
            └── visualize_surface_toolpath.cpp
```

The ROS package itself is located at:

```text
src/ur5e_moveit_cpp
```

---

## Project goals

Immediate workflow:

```text
simple object geometry
→ generated surface path
→ collision object in MoveIt
→ collision-aware IK validation
→ OMPL approach planning
→ Cartesian surface-following path
→ mock execution
→ later real-robot execution
```

Longer-term workflow:

```text
RealSense or 3D scanner data
→ object localization
→ point-cloud registration / mesh generation
→ path transformation into the robot frame
→ collision-aware planning and execution
```

---

## Environment

Current development setup:

- Ubuntu 24.04
- ROS 2 Jazzy
- MoveIt 2
- UR5e
- RViz
- C++ planning nodes
- Python path generation

Workspace used during development:

```bash
~/ur5e_ws
```

Source ROS and the workspace in every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur5e_ws/install/setup.bash
```

---

## Required MoveIt configuration and custom tool

This repository contains the C++ nodes and path data, but it depends on a separate MoveIt configuration package for the complete UR5e model, mock controllers, RViz configuration, and custom probe tool.

The package used during development is:

```text
ur5e_probe_moveit_config
```

Expected location:

```text
~/ur5e_ws/src/ur5e_probe_moveit_config
```

Launch the mock MoveIt/RViz environment:

```bash
ros2 launch ur5e_probe_moveit_config demo.launch.py
```

Keep that terminal running while using the nodes in this repository.

### Tool frame chain

```text
flange → probe_tool_link → probe_tcp
```

The surface-planning code uses:

```text
planning group: ur_manipulator
planning frame: world
TCP link: probe_tcp
```

The older examples generally use the default end effector or explicitly use `tool0`.

### Verify the custom TCP

```bash
ros2 run tf2_ros tf2_echo world probe_tcp
```

A valid transform confirms that the custom TCP is present in the robot model.

### Include the tool in RViz and MoveIt

The tool must be added to the URDF/xacro and loaded by the MoveIt config package. The C++ nodes do not modify the robot model.

Required steps:

1. Add the tool visual and collision geometry to the robot description.
2. Add a fixed joint from the UR5e flange to `probe_tool_link`.
3. Add a fixed `probe_tcp` frame at the real tool center point.
4. Ensure the MoveIt config loads the updated robot description.
5. Launch `ur5e_probe_moveit_config demo.launch.py`.
6. Confirm that the visual and collision models align in RViz.
7. Confirm that `world -> probe_tcp` exists in TF.

---

## Build

From the workspace root:

```bash
cd ~/ur5e_ws
source /opt/ros/jazzy/setup.bash

colcon build   --packages-select ur5e_moveit_cpp   --symlink-install

source install/setup.bash
```

Install missing dependencies when needed:

```bash
cd ~/ur5e_ws
rosdep install --from-paths src --ignore-src -r -y
```

Clean rebuild:

```bash
cd ~/ur5e_ws
rm -rf build/ur5e_moveit_cpp install/ur5e_moveit_cpp

colcon build   --packages-select ur5e_moveit_cpp   --symlink-install

source install/setup.bash
```

---

## Executables

### `simple_joint_motion`

Basic joint-space planning and execution test.

What it does:

- Reads the current joint values.
- Adds `0.8` radians to the first returned joint.
- Plans to the new joint target.
- Executes the plan immediately.
- Uses velocity and acceleration scaling of `0.1`.

```bash
ros2 run ur5e_moveit_cpp simple_joint_motion
```

> Executes by default.

---

### `simple_pose_motion`

Basic pose-target example using `tool0`.

What it does:

- Reads the current `tool0` pose.
- Adds `0.02 m` to its X position.
- Uses `setApproximateJointValueTarget()` for IK.
- Plans and executes the result.

```bash
ros2 run ur5e_moveit_cpp simple_pose_motion
```

> Executes by default and targets `tool0`, not `probe_tcp`.

---

### `plan_only_trajectory`

Plans a small joint-space motion without execution.

What it does:

- Adds `0.2` radians to the first joint.
- Prints joint names, timestamps, positions, and velocities for every trajectory point.

```bash
ros2 run ur5e_moveit_cpp plan_only_trajectory
```

---

### `save_trajectory_csv`

Plans a small joint-space motion and exports it to CSV.

Current hard-coded output:

```text
/home/safwan/ur5e_planned_trajectory.csv
```

The file includes:

- trajectory point index
- time from start
- joint positions
- joint velocities

```bash
ros2 run ur5e_moveit_cpp save_trajectory_csv
```

> The hard-coded output path should eventually become a ROS parameter.

---

### `analyze_trajectory`

Plans without execution and reports:

- total duration
- total joint-space distance
- average joint-space speed
- total absolute movement per joint
- maximum absolute velocity per joint
- maximum single-step joint change per joint

```bash
ros2 run ur5e_moveit_cpp analyze_trajectory
```

---

### `compare_pose_candidates`

Tests pose offsets around the current `tool0` pose:

```text
+X 10 cm
-X 10 cm
+Y 10 cm
-Y 10 cm
+Z 8 cm
-Z 8 cm
```

Successful plans are scored by total joint-space distance. The shortest plan is executed.

```bash
ros2 run ur5e_moveit_cpp compare_pose_candidates
```

> Executes the selected plan and uses `tool0`.

---

### `add_box_only`

Adds a box collision object and keeps it visible for RViz inspection.

Actual configured dimensions:

```text
0.20 × 0.20 × 0.20 m
```

Pose:

```text
x = 0.80
y = 0.20
z = 0.40
```

The node also publishes a planning-scene diff to `/monitored_planning_scene` and waits for 60 seconds.

```bash
ros2 run ur5e_moveit_cpp add_box_only
```

> Some source comments/logs still describe an `0.80 m` cube, but the actual primitive is `0.20 m` per side.

---

### `collision_object_demo`

Adds a box obstacle and then plans and executes a joint-space motion.

Box:

```text
size = 0.60 × 0.60 × 0.60 m
position = (0.70, 0.00, 0.30)
```

Motion:

```text
first joint += 0.4 rad
second joint += 0.2 rad
```

```bash
ros2 run ur5e_moveit_cpp collision_object_demo
```

> Executes by default.

---

### `collision_aware_candidate_planner`

Adds a box and tests collision-aware pose candidates using `tool0`.

Actual box:

```text
size = 0.30 × 0.30 × 0.30 m
position = (0.80, 0.00, 0.30)
```

Candidates:

```text
±X 12 cm
±Y 12 cm
±Z 10 cm
```

Successful plans are scored by total joint-space distance, and the best plan is executed.

```bash
ros2 run ur5e_moveit_cpp collision_aware_candidate_planner
```

> Executes the selected plan.

---

### `remove_all_objects`

Removes every collision object known to MoveIt and publishes explicit `REMOVE` operations so RViz clears stale geometry.

```bash
ros2 run ur5e_moveit_cpp remove_all_objects
```

---

### `parameterized_shape_adder`

Adds a box, cylinder, or sphere using ROS parameters.

Supported parameters:

```text
shape_type: box | cylinder | sphere
object_id
frame_id
center_x, center_y, center_z
box_x, box_y, box_z
radius
height
```

Sphere example:

```bash
ros2 run ur5e_moveit_cpp parameterized_shape_adder   --ros-args   -p shape_type:=sphere   -p object_id:=test_sphere   -p frame_id:=world   -p center_x:=0.55   -p center_y:=0.00   -p center_z:=0.25   -p radius:=0.075
```

Cylinder example:

```bash
ros2 run ur5e_moveit_cpp parameterized_shape_adder   --ros-args   -p shape_type:=cylinder   -p object_id:=test_cylinder   -p frame_id:=world   -p center_x:=0.60   -p center_y:=0.00   -p center_z:=0.25   -p radius:=0.10   -p height:=0.30
```

This node does not plan or execute motion.

---

### `hemisphere_surface_demo`

Main sphere surface-path planning diagnostic.

Default CSV:

```text
/home/desktop/ur5e_ws/paths/hemisphere_ring_start_00.csv
```

CSV format:

```text
x,y,z,qx,qy,qz,qw,nx,ny,nz
```

The node:

1. Loads the ring waypoints.
2. Adds a sphere collision object.
3. Plans a collision-aware OMPL approach to the first pose.
4. Uses `probe_tcp` as the end-effector link.
5. Reconstructs the approach-end joint state from the planned trajectory.
6. Computes a Cartesian path through the remaining waypoints.
7. Reports the completion fraction.
8. Executes only when `execute:=true`.

Sphere:

```text
center = (0.55, 0.00, 0.25)
radius = 0.075 m
object ID = hemisphere_demo_sphere
```

Default parameters:

```text
execute = false
eef_step = 0.002
jump_threshold = 0.0
```

Planning-only run:

```bash
ros2 run ur5e_moveit_cpp hemisphere_surface_demo
```

Explicit run:

```bash
ros2 run ur5e_moveit_cpp hemisphere_surface_demo   --ros-args   -p csv_path:=/home/desktop/ur5e_ws/paths/hemisphere_ring_start_00.csv   -p execute:=false   -p eef_step:=0.002   -p jump_threshold:=0.0
```

Keep execution disabled until the full path is validated.

---

### `visualize_surface_toolpath`

Visualization-only node for a raster toolpath and mesh.

Default resources:

```text
CSV: package share/data/toolpaths/front_raster.csv
mesh: package://ur5e_moveit_cpp/meshes/apple_surface_z_up.stl
```

Default object pose:

```text
position = (0.60, 0.00, 0.10)
RPY = (0°, 0°, 90°)
mesh scale = 0.001
```

Expected CSV columns:

```text
line_id
point_index
tcp_x_mm
tcp_y_mm
tcp_z_mm
qx
qy
qz
qw
```

The node:

- reads poses in object coordinates
- converts millimeters to meters
- applies the object-to-world transform
- publishes the mesh
- publishes line-strip path markers
- publishes sampled tool `+Z` arrows
- publishes a `PoseArray`
- does not plan or execute

Published topics:

```text
/surface_toolpath/markers
/surface_toolpath/poses
```

Run:

```bash
ros2 run ur5e_moveit_cpp visualize_surface_toolpath
```

Add these displays in RViz:

- `MarkerArray` on `/surface_toolpath/markers`
- `PoseArray` on `/surface_toolpath/poses`

> The mesh marker is visualization only. It is not automatically a MoveIt collision object.

---

## Hemisphere path generator

File:

```text
paths/generate_hemisphere_ring.py
```

Current geometry:

```python
CENTER_X = 0.55
CENTER_Y = 0.00
CENTER_Z = 0.25
SPHERE_RADIUS = 0.075
STANDOFF = 0.015
PHI = math.radians(70.4)
WAYPOINT_COUNT = 72
START_SAMPLE_COUNT = 10
```

Orientation convention:

```text
TCP local +Z points toward the sphere center.
TCP local +X follows the ring tangent.
TCP local +Y completes the right-handed frame.
```

Run:

```bash
cd ~/ur5e_ws
python3 paths/generate_hemisphere_ring.py
```

Generated output is written to:

```text
~/ur5e_ws/paths
```

---

## Surface-path debugging history

Programmatic IK initially failed even though manually selected RViz poses appeared reachable.

The debugging process separated:

```text
collision object insertion
→ geometric IK
→ collision-aware IK
→ state validity
→ tool-roll scan
→ multiple ring starts
→ OMPL approach
→ Cartesian continuity
```

The main bug was an extra local-Y rotation in the generator. It reversed the tool's local `+Z` axis so the tool pointed away from the sphere.

Correct convention:

```text
probe_tcp local +Z points toward the sphere center.
```

A known-good RViz pose was measured with:

```bash
ros2 run tf2_ros tf2_echo world probe_tcp
```

Approximate pose:

```text
translation = (0.565, -0.083, 0.280)
quaternion = (-0.809, 0.000, 0.000, 0.588)
```

This informed:

```text
PHI ≈ 70.4°
STANDOFF ≈ 0.015 m
```

Validation results after the correction:

```text
12 / 12 tool-roll samples had collision-free IK
10 / 10 sampled ring starts had valid collision-free target states
OMPL approach to Start 00 succeeded
```

Latest Cartesian result:

```text
Cartesian completion fraction: 0.859
Cartesian trajectory points: 257
```

The sampled poses are valid individually, but the full loop does not yet maintain one continuous collision-free IK branch.

---

## Current checkpoint

Confirmed:

- custom tool and `probe_tcp`
- programmatic collision objects
- parameterized primitive insertion
- generated sphere-ring TCP poses
- correct inward tool orientation
- collision-aware IK at sampled targets
- OMPL approach to Start 00
- most of the Cartesian ring
- apple mesh and raster path visualization

Current unresolved issue:

```text
The closed Cartesian ring completes approximately 85.9%.
```

Next diagnostic:

```text
Start 00 → waypoint 01
waypoint 01 → waypoint 02
waypoint 02 → waypoint 03
...
```

Each segment should use the previous segment's final joint state. The first failure can then be checked for:

- collision
- joint limits
- wrist wrapping
- singularity
- IK branch switching
- seam discontinuity

Do not change the confirmed sphere pose, radius, `PHI`, standoff, or TCP orientation before this diagnostic.

---

## Known limitations and cleanup items

- Older examples use `tool0`; current surface work uses `probe_tcp`.
- Several early examples execute automatically.
- `save_trajectory_csv` has a hard-coded `/home/safwan` path.
- `hemisphere_surface_demo` has a hard-coded `/home/desktop` CSV path.
- `computeCartesianPath()` uses a deprecated overload with `jump_threshold`.
- `add_box_only.cpp` comments/logs do not match the actual `0.20 m` dimensions.
- `collision_aware_candidate_planner.cpp` contains an outdated box-height comment.
- The visualizer mesh is not automatically a MoveIt collision object.
- A standalone client may warn that no kinematics plugin is defined even while `/move_group` has a working solver.

---

## Dependencies

Current package dependencies:

```text
rclcpp
moveit_ros_planning_interface
geometry_msgs
moveit_msgs
shape_msgs
sensor_msgs
moveit_core
tf2
tf2_geometry_msgs
visualization_msgs
ament_index_cpp
```

The package installs all executables plus the `data` and `meshes` directories into the package share directory.
