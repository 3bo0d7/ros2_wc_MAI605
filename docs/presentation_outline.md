# Presentation Outline

## Slide 1: Project Goal

QR-based robotic sorting with ROS 2 Humble, MoveIt 2, MoveIt Task Constructor, zbar_ros, and RViz simulation.

## Slide 2: System Architecture

Show node graph:

`sim_qr_camera -> zbar_ros -> qr_decision_node -> pick_scan_place_mtc -> MoveIt/RViz`

Mention that `/qr_sort/scan_ready` gates the simulated camera so scanning happens after the planned pick-to-scan stage.

## Slide 3: Workcell

Show Panda robot, fixed pick station, workpiece with QR panel, QR scanner/camera frustum, bins, route line, and selected-bin marker.

## Slide 4: Manipulation Pipeline

List key MTC stages:

`ready -> open -> pick -> approach -> close -> attach -> lift -> scan -> bin -> lower -> open -> detach -> retreat -> home`

## Slide 5: QR Perception Pipeline

Explain generated QR image, zbar decode, `/barcode`, bin mapping, and `/qr_sort/bin_pose`.

## Slide 6: Object Attachment

Show that the workpiece is a MoveIt collision object and is attached to `panda_hand` during the carry motion.

## Slide 7: Test Evidence

Include terminal evidence:

- Pick-to-scan solution count.
- `/qr_sort/scan_ready` trigger.
- `zbar_ros` `/barcode` decode.
- `BIN_A/B/C/UNKNOWN` mapping.
- Full MTC solution count.

## Slide 8: Limitations And Future Work

Current limitation: RViz/MoveIt simulation only. Future work: Gazebo/Isaac camera, physics-enabled grasping, and controller execution validation.
