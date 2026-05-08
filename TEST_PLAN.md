# Test Plan

## Build Test

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select qr_sorting_mtc qr_sorting_logic qr_sorting_bringup
source install/setup.bash
```

Expected: all three packages build successfully.

## QR Mapping Tests

Run each case after cleaning stale nodes:

```bash
ros2 launch qr_sorting_bringup qr_sorting_demo.launch.py qr_text:=BIN_A launch_rviz:=true
ros2 launch qr_sorting_bringup qr_sorting_demo.launch.py qr_text:=BIN_B launch_rviz:=true
ros2 launch qr_sorting_bringup qr_sorting_demo.launch.py qr_text:=BIN_C launch_rviz:=true
ros2 launch qr_sorting_bringup qr_sorting_demo.launch.py qr_text:=UNKNOWN launch_rviz:=true
```

Expected mapping:

| QR text | Expected bin |
| --- | --- |
| `BIN_A` | `bin_a` |
| `BIN_B` | `bin_b` |
| `BIN_C` | `bin_c` |
| `UNKNOWN` | `reject_bin` |

Evidence topics:

```bash
ros2 topic echo /barcode
ros2 topic echo /qr_sort/bin_id
ros2 topic echo /qr_sort/bin_pose
ros2 topic echo /qr_sort/scan_ready
```

## MTC Planning Test

Expected log evidence:

- `Pick-to-scan planning succeeded. Solutions: 1`
- `Added planning scene objects: support 'pick_station_support' and workpiece 'workpiece'`
- `Published /qr_sort/scan_ready`
- `Scan trigger received. Publishing QR images for zbar_ros decode.`
- `QR "... " from /barcode mapped to ...`
- `Planning QR pick-scan-place task. execute=false`
- `Planning succeeded. Solutions: 1`

Expected task stages:

- current state
- move to ready
- open gripper before pick
- allow gripper-object contact
- move to pick pose
- approach object
- close gripper on object
- attach workpiece to gripper
- lift attached object
- move attached object to QR scan pose
- move attached object above selected bin opening
- lower attached object into bin
- open gripper at bin
- detach workpiece in bin
- retreat from bin
- return home
- forbid gripper-object contact after release

## RViz Test

Expected:

- Panda robot is visible.
- Workpiece, scanner/camera, square open-top bins, route line, and selected-bin marker are visible on `/qr_sort/visualization`.
- After `/qr_sort/scan_ready`, the static pick-station object marker is hidden so it does not duplicate the attached MoveIt object.
- Motion Planning Tasks displays the MTC solution on `/solution`.
- The attached object appears in the planning scene during the planned carry motion.
- The Panda fingers close to a configured grasp width around the outside of the cube instead of using the fully closed hand state.
- The palm/finger base remains outside the cube because pose targets use a grasp-center IK offset.
- The selected bin is entered from above; the vertical lower/retreat stages should not pass through bin walls.
- Text labels do not overlap the robot; the status line is offset from the workcell.
- The QR-like panel on the workpiece and the scanner/camera/frustum markers are visible.

## Execution Test

This code leaves `execute:=false` by default. Use RViz Motion Planning Tasks animation first. Only enable execution after confirming a valid controller/action setup:

```bash
ros2 action list | grep -E "follow_joint|execute"
```

Expected available actions include `/execute_trajectory` and the Panda arm trajectory action.
