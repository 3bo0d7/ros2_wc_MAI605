# MAI605 QR Sorting Manipulation Demo

ROS 2 Humble project for a Panda-based pick, scan, and place sorting pipeline. The demo uses MoveIt 2 and MoveIt Task Constructor for manipulation, `zbar_ros` for QR decoding, and RViz markers for a presentable workcell.

The project objective from the MAI605 brief is:

`fixed pick station -> pick object -> move to scan pose -> decode QR -> choose bin -> place object`

## Packages

- `qr_sorting_bringup`: launch file and shared parameters.
- `qr_sorting_logic`: simulated QR image publisher, zbar decision logic, and RViz workcell markers.
- `qr_sorting_mtc`: MoveIt Task Constructor pick-scan-place task.

## Dependencies

Tested on Ubuntu 22.04 / ROS 2 Humble in WSL.

Required ROS packages include:

- `moveit_resources_panda_moveit_config`
- `moveit_ros_planning_interface`
- `moveit_task_constructor_core`
- `zbar_ros`
- `sensor_msgs`, `visualization_msgs`, `geometry_msgs`, `std_msgs`

The simulated QR camera uses Python `qrcode`, `PIL`, and `numpy`.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select qr_sorting_mtc qr_sorting_logic qr_sorting_bringup
source install/setup.bash
```

## Launch

Clean stale nodes before a demo:

```bash
pkill -INT -f move_group || true
pkill -INT -f rviz2 || true
pkill -INT -f robot_state_publisher || true
pkill -INT -f joint_state_publisher || true
pkill -INT -f pick_scan_place_mtc || true
pkill -INT -f barcode_reader || true
pkill -INT -f qr_decision_node || true
pkill -INT -f scene_visualizer_node || true
pkill -INT -f sim_qr_camera_node || true
sleep 3
ros2 daemon stop
ros2 daemon start
```

Run the demo:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qr_sorting_bringup qr_sorting_demo.launch.py qr_text:=BIN_A launch_rviz:=true
```

In RViz:

- Add `MarkerArray` topic `/qr_sort/visualization`.
- In Motion Planning Tasks, set Task Solution Topic to `/solution`.

## Current Behavior

The MTC node first plans a pick-to-scan phase and publishes `/qr_sort/scan_ready`. The simulated QR camera only publishes the QR image after that trigger. `zbar_ros` decodes the image to `/barcode`, `qr_decision_node` maps the QR text to `/qr_sort/bin_pose`, then the full MTC task plans:

`current -> ready -> open gripper -> allow contact -> pick -> approach -> close gripper -> attach workpiece -> lift -> scan -> selected bin -> lower -> open gripper -> detach workpiece -> retreat -> return home -> restore collision checking`

The workpiece is a real MoveIt collision object named `workpiece` and is attached/detached using `stages::ModifyPlanningScene`. A small pick-station support collision object is also inserted so the planning scene contains static environment geometry without blocking the simplified Panda gripper motion. The Panda hand uses configurable finger joint widths (`gripper_open_width` and `gripper_grasp_width`) and a `grasp_frame_offset` so the fingers clamp the exterior of the 6 cm cube while the palm/finger base stays outside the object. The open-top bins are also represented as MoveIt collision objects, and the place motion approaches from above, lowers through the opening, releases, and retreats upward. The larger table, scanner, camera, QR panel, open-top square bins, route, and selected-bin result are RViz markers on `/qr_sort/visualization`.

## Requirements Trace

| Project requirement | Current implementation |
| --- | --- |
| ROS 2 Humble+ | ROS 2 Humble workspace in `~/ros2_ws` |
| At least 5 DOF robot | Panda arm from `moveit_resources_panda_moveit_config` |
| MoveIt 2 planning | `move_group` plus MoveIt Task Constructor |
| Collision-free trajectories | MTC stages with planning-scene `workpiece`, pick support, and open-top bin wall collision objects |
| QR integration with `zbar_ros` | Simulated QR image on `/qr_sort/sim_camera/image_raw`, decoded by `zbar_ros` to `/barcode` |
| QR decision logic | `qr_decision_node` maps QR text to configured bin pose |
| Modular packages | `qr_sorting_bringup`, `qr_sorting_logic`, `qr_sorting_mtc` |
| Parameters and launch files | Shared YAML config and `qr_sorting_demo.launch.py` |
| Simulation evidence | RViz/MoveIt visualization, MTC solution on `/solution`, MarkerArray workcell |

## Verified Evidence

Verified with:

```bash
colcon build --symlink-install --packages-select qr_sorting_mtc qr_sorting_logic qr_sorting_bringup
timeout 120s ros2 launch qr_sorting_bringup qr_sorting_demo.launch.py qr_text:=BIN_A launch_rviz:=false
```

Observed:

- Pick-to-scan planning succeeded with 1 solution.
- `/qr_sort/scan_ready` was published after the scan-stage solution.
- zbar/decision path mapped `BIN_A` to `bin_a`.
- Full MTC pick-scan-place planning succeeded with 1 solution.
- MTC solution was republished for RViz visualization.
- After collision tuning, the pick-station support collision object no longer collides with Panda ready/pick states.
- The Panda gripper stages use width targets and a grasp-center IK offset so the fingers close around the exterior of the workpiece without the palm entering the cube.
- RViz bins are rendered as square open-top containers with visible side walls and top rims.
- MoveIt planning includes open-top bin wall/floor collision objects, so the place motion goes above the selected bin and lowers vertically into the opening.
- Verified `BIN_A`, `BIN_B`, `BIN_C`, and `UNKNOWN` all map and plan successfully with the collision-aware bin layout.

## Troubleshooting

- If RViz shows no MTC tree, set Task Solution Topic to `/solution`.
- If Motion Planning Tasks shows `Status: Warn` only under `Solution`, check that the task has published on `/solution` and select or animate a solution. `Robot Model: Successfully loaded` and `Task Monitor: OK` mean the display is connected.
- If QR output is missing, echo `/qr_sort/scan_ready`, `/barcode`, `/qr_sort/bin_id`, and `/qr_sort/bin_pose`.
- If duplicate nodes appear, run the cleanup commands above.
- `trajopt`/`lerp` planner plugin warnings from the Panda config are non-blocking when OMPL loads.
- This is an RViz/MoveIt simulation. It does not claim Gazebo physics, a real camera, or a real robot controller.
- `/qr_sort/scan_ready` is a planning-level trigger. With `execute:=false`, it confirms the planned scan phase rather than a physical sensor event.
# ros2_wc_MAI605
# ros2_wc_MAI605
# ros2_wc_MAI605
