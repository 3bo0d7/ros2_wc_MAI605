/**
 * @file pick_scan_place_mtc.cpp
 * @brief MoveIt Task Constructor implementation for QR-based pick-scan-place sorting.
 *
 * Demonstrated sequence:
 *   current -> ready -> open gripper -> pick pose -> approach -> close gripper
 *   -> attach workpiece -> lift -> scan pose -> selected bin -> lower
 *   -> open gripper -> detach workpiece -> retreat -> home
 *
 * Notes:
 * - The workpiece is inserted as a MoveIt collision object.
 * - The object is attached to the Panda hand after grasp and detached at placement.
 * - QR decision is received from /qr_sort/bin_pose.
 */

#include <chrono>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>

using moveit::task_constructor::InitStageException;
using moveit::task_constructor::Stage;
using moveit::task_constructor::Task;
namespace stages = moveit::task_constructor::stages;
namespace solvers = moveit::task_constructor::solvers;

class PickScanPlaceMtcNode : public rclcpp::Node {
public:
  PickScanPlaceMtcNode() : Node("pick_scan_place_mtc") {
    planning_group_ = declare_parameter<std::string>("planning_group", "panda_arm");
    gripper_group_ = declare_parameter<std::string>("gripper_group", "hand");
    eef_frame_ = declare_parameter<std::string>("eef_frame", "panda_hand");
    base_frame_ = declare_parameter<std::string>("base_frame", "panda_link0");

    execute_ = declare_parameter<bool>("execute", false);

    home_named_target_ = declare_parameter<std::string>("home_named_target", "ready");
    ready_named_target_ = declare_parameter<std::string>("ready_named_target", "ready");

    use_gripper_stages_ = declare_parameter<bool>("use_gripper_stages", true);
    use_gripper_width_targets_ = declare_parameter<bool>("use_gripper_width_targets", true);
    gripper_open_named_target_ = declare_parameter<std::string>("gripper_open_named_target", "open");
    gripper_closed_named_target_ = declare_parameter<std::string>("gripper_closed_named_target", "close");
    gripper_open_width_ = declare_parameter<double>("gripper_open_width", 0.04);
    gripper_grasp_width_ = declare_parameter<double>("gripper_grasp_width", 0.028);
    grasp_frame_offset_ =
      declare_parameter<std::vector<double>>("grasp_frame_offset", {0.0, 0.0, 0.1034});

    object_id_ = declare_parameter<std::string>("object_id", "workpiece");
    object_size_ = declare_parameter<std::vector<double>>("object_size", {0.06, 0.06, 0.06});
    table_id_ = declare_parameter<std::string>("table_id", "pick_station_support");
    table_size_ = declare_parameter<std::vector<double>>("table_size", {0.16, 0.16, 0.02});
    table_pose_ = declare_parameter<std::vector<double>>("table_pose", {0.45, 0.0, 0.20});
    bin_outer_size_ = declare_parameter<double>("bin_outer_size", 0.20);
    bin_wall_thickness_ = declare_parameter<double>("bin_wall_thickness", 0.010);
    bin_wall_height_ = declare_parameter<double>("bin_wall_height", 0.10);
    bin_floor_thickness_ = declare_parameter<double>("bin_floor_thickness", 0.012);
    bin_bottom_offset_ = declare_parameter<double>("bin_bottom_offset", 0.055);
    bin_place_z_offset_ = declare_parameter<double>("bin_place_z_offset", 0.04);
    bin_a_pose_ = poseFromParameter("bin_a.pose", {0.30, 0.50, 0.35, 0.0, 1.0, 0.0, 0.0});
    bin_b_pose_ = poseFromParameter("bin_b.pose", {0.50, 0.28, 0.35, 0.0, 1.0, 0.0, 0.0});
    bin_c_pose_ = poseFromParameter("bin_c.pose", {0.30, -0.50, 0.35, 0.0, 1.0, 0.0, 0.0});
    reject_bin_pose_ = poseFromParameter("reject_bin.pose", {0.50, -0.28, 0.35, 0.0, 1.0, 0.0, 0.0});

    approach_distance_ = declare_parameter<double>("approach_distance", 0.02);
    lift_distance_ = declare_parameter<double>("lift_distance", 0.05);
    lower_distance_ = declare_parameter<double>("lower_distance", 0.10);
    retreat_distance_ = declare_parameter<double>("retreat_distance", 0.10);

    pick_pose_ = poseFromParameter("pick_pose", {0.45, 0.00, 0.37, 0.0, 1.0, 0.0, 0.0});
    scan_pose_ = poseFromParameter("scan_pose", {0.35, -0.25, 0.50, 0.0, 1.0, 0.0, 0.0});
    selected_bin_pose_ = poseFromVector({0.45, 0.25, 0.35, 0.0, 1.0, 0.0, 0.0});

    bin_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/qr_sort/bin_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        selected_bin_pose_ = *msg;
        have_bin_pose_ = true;
        RCLCPP_INFO(get_logger(),
                    "Received selected bin pose: x=%.3f y=%.3f z=%.3f",
                    msg->pose.position.x,
                    msg->pose.position.y,
                    msg->pose.position.z);
      });

    scan_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/qr_sort/scan_ready", 10);

    plan_timer_ = create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        if (planned_) {
          return;
        }

        if (!pick_to_scan_planned_) {
          pick_to_scan_planned_ = true;
          planPickToScanTask();
          return;
        }

        if (!have_bin_pose_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Pick-to-scan phase is ready. Waiting for zbar/decision output on /qr_sort/bin_pose.");
          return;
        }

        planned_ = true;
        planTask();
      });
  }

private:
  geometry_msgs::msg::PoseStamped poseFromParameter(
      const std::string& name,
      const std::vector<double>& fallback) {
    const auto raw = declare_parameter<std::vector<double>>(name, fallback);
    return poseFromVector(raw);
  }

  geometry_msgs::msg::PoseStamped poseFromVector(const std::vector<double>& raw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = base_frame_;

    if (raw.size() != 7) {
      RCLCPP_WARN(get_logger(), "Pose parameter must contain 7 values. Using identity fallback.");
      pose.pose.orientation.w = 1.0;
      return pose;
    }

    pose.pose.position.x = raw[0];
    pose.pose.position.y = raw[1];
    pose.pose.position.z = raw[2];
    pose.pose.orientation.x = raw[3];
    pose.pose.orientation.y = raw[4];
    pose.pose.orientation.z = raw[5];
    pose.pose.orientation.w = raw[6];
    return pose;
  }

  std::unique_ptr<stages::MoveTo> makeMoveToPose(
      const std::string& name,
      const geometry_msgs::msg::PoseStamped& pose,
      const solvers::PlannerInterfacePtr& planner) {
    auto stage = std::make_unique<stages::MoveTo>(name, planner);
    stage->setGroup(planning_group_);

    // Pose targets are expressed as the desired cube/grasp center. The IK frame
    // offset moves the Panda hand frame back to the palm, keeping the hand body
    // above/outside the cube while the fingers close on the cube sides.
    Eigen::Isometry3d grasp_frame = Eigen::Isometry3d::Identity();
    if (grasp_frame_offset_.size() == 3) {
      grasp_frame.translation().x() = grasp_frame_offset_[0];
      grasp_frame.translation().y() = grasp_frame_offset_[1];
      grasp_frame.translation().z() = grasp_frame_offset_[2];
    } else {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "grasp_frame_offset must contain 3 values. Using Panda default [0, 0, 0.1034].");
      grasp_frame.translation().z() = 0.1034;
    }
    stage->setIKFrame(grasp_frame, eef_frame_);
    stage->setGoal(pose);
    return stage;
  }

  geometry_msgs::msg::PoseStamped poseWithZOffset(
      const geometry_msgs::msg::PoseStamped& pose,
      double dz) const {
    auto out = pose;
    out.pose.position.z += dz;
    return out;
  }

  std::unique_ptr<stages::MoveRelative> makeCartesianMove(
      const std::string& name,
      double dx,
      double dy,
      double dz,
      const std::shared_ptr<solvers::CartesianPath>& cartesian) {
    auto stage = std::make_unique<stages::MoveRelative>(name, cartesian);
    stage->properties().configureInitFrom(Stage::PARENT, {"group"});

    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = base_frame_;
    direction.vector.x = dx;
    direction.vector.y = dy;
    direction.vector.z = dz;

    stage->setDirection(direction);
    return stage;
  }

  double clampedFingerWidth(double width) const {
    if (width < 0.0) {
      return 0.0;
    }
    if (width > 0.04) {
      return 0.04;
    }
    return width;
  }

  std::unique_ptr<stages::MoveTo> makeGripperMove(
      const std::string& name,
      const std::string& named_target,
      const solvers::PlannerInterfacePtr& planner) {
    auto stage = std::make_unique<stages::MoveTo>(name, planner);
    stage->setGroup(gripper_group_);
    stage->setGoal(named_target);
    return stage;
  }

  std::unique_ptr<stages::MoveTo> makePandaGripperWidthMove(
      const std::string& name,
      double finger_width,
      const solvers::PlannerInterfacePtr& planner) {
    auto stage = std::make_unique<stages::MoveTo>(name, planner);
    stage->setGroup(gripper_group_);

    const double target = clampedFingerWidth(finger_width);
    stage->setGoal(std::map<std::string, double>{
      {"panda_finger_joint1", target},
      {"panda_finger_joint2", target},
    });
    return stage;
  }

  std::unique_ptr<stages::MoveTo> makeOpenGripperMove(
      const solvers::PlannerInterfacePtr& planner) {
    if (use_gripper_width_targets_) {
      return makePandaGripperWidthMove("open gripper before pick", gripper_open_width_, planner);
    }
    return makeGripperMove("open gripper before pick", gripper_open_named_target_, planner);
  }

  std::unique_ptr<stages::MoveTo> makeCloseGripperMove(
      const solvers::PlannerInterfacePtr& planner) {
    if (use_gripper_width_targets_) {
      return makePandaGripperWidthMove("close gripper on object", gripper_grasp_width_, planner);
    }
    return makeGripperMove("close gripper on object", gripper_closed_named_target_, planner);
  }

  std::unique_ptr<stages::MoveTo> makeOpenGripperAtBinMove(
      const solvers::PlannerInterfacePtr& planner) {
    if (use_gripper_width_targets_) {
      return makePandaGripperWidthMove("open gripper at bin", gripper_open_width_, planner);
    }
    return makeGripperMove("open gripper at bin", gripper_open_named_target_, planner);
  }

  std::vector<std::string> gripperCollisionLinks(const Task& task) const {
    std::vector<std::string> links;

    const auto* jmg = task.getRobotModel()->getJointModelGroup(gripper_group_);
    if (jmg) {
      links = jmg->getLinkModelNamesWithCollisionGeometry();
    }

    if (links.empty()) {
      links.push_back(eef_frame_);
    }

    return links;
  }

  moveit_msgs::msg::CollisionObject makeBoxCollisionObject(
      const std::string& id,
      const std::vector<double>& size,
      const geometry_msgs::msg::Pose& pose) const {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = base_frame_;
    object.id = id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = size[0];
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = size[1];
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = size[2];

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    return object;
  }

  void appendOpenBinCollisionObjects(
      const std::string& bin_id,
      const geometry_msgs::msg::PoseStamped& bin_pose,
      std::vector<moveit_msgs::msg::CollisionObject>& objects) const {
    const double outer = bin_outer_size_;
    const double wall = bin_wall_thickness_;
    const double floor = bin_floor_thickness_;
    const double height = bin_wall_height_;
    const double bottom_z = bin_pose.pose.position.z - bin_bottom_offset_;
    const double wall_z = bottom_z + floor + (height * 0.5);

    auto pose_at = [](double x, double y, double z) {
      geometry_msgs::msg::Pose pose;
      pose.orientation.w = 1.0;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;
      return pose;
    };

    const double x = bin_pose.pose.position.x;
    const double y = bin_pose.pose.position.y;
    objects.push_back(makeBoxCollisionObject(
      bin_id + "_floor", {outer, outer, floor}, pose_at(x, y, bottom_z + floor * 0.5)));
    objects.push_back(makeBoxCollisionObject(
      bin_id + "_wall_px", {wall, outer, height}, pose_at(x + outer * 0.5 - wall * 0.5, y, wall_z)));
    objects.push_back(makeBoxCollisionObject(
      bin_id + "_wall_nx", {wall, outer, height}, pose_at(x - outer * 0.5 + wall * 0.5, y, wall_z)));
    objects.push_back(makeBoxCollisionObject(
      bin_id + "_wall_py", {outer, wall, height}, pose_at(x, y + outer * 0.5 - wall * 0.5, wall_z)));
    objects.push_back(makeBoxCollisionObject(
      bin_id + "_wall_ny", {outer, wall, height}, pose_at(x, y - outer * 0.5 + wall * 0.5, wall_z)));
  }

  void addPlanningSceneObjects() {
    if (object_size_.size() != 3) {
      RCLCPP_WARN(get_logger(), "object_size must contain 3 values. Using 0.06 m cube.");
      object_size_ = {0.06, 0.06, 0.06};
    }

    if (table_size_.size() != 3 || table_pose_.size() != 3) {
      RCLCPP_WARN(get_logger(), "table_size/table_pose must contain 3 values. Using default pick-station support.");
      table_size_ = {0.16, 0.16, 0.02};
      table_pose_ = {0.45, 0.0, 0.20};
    }

    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = table_pose_[0];
    table_pose.position.y = table_pose_[1];
    table_pose.position.z = table_pose_[2];

    geometry_msgs::msg::Pose object_pose;
    object_pose.orientation.w = 1.0;

    object_pose.position.x = pick_pose_.pose.position.x;
    object_pose.position.y = pick_pose_.pose.position.y;

    // The pick pose targets the cube/grasp center before the final vertical
    // approach. After the approach, the grasp center coincides with the object
    // center while the hand frame stays offset above the cube.
    object_pose.position.z = pick_pose_.pose.position.z - approach_distance_;

    const auto table = makeBoxCollisionObject(table_id_, table_size_, table_pose);
    const auto object = makeBoxCollisionObject(object_id_, object_size_, object_pose);

    std::vector<moveit_msgs::msg::CollisionObject> objects{table, object};
    appendOpenBinCollisionObjects("bin_a", bin_a_pose_, objects);
    appendOpenBinCollisionObjects("bin_b", bin_b_pose_, objects);
    appendOpenBinCollisionObjects("bin_c", bin_c_pose_, objects);
    appendOpenBinCollisionObjects("reject_bin", reject_bin_pose_, objects);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObjects(objects);

    RCLCPP_INFO(get_logger(),
                "Added planning scene objects: support '%s', workpiece '%s', and open-top bin walls at x=%.3f y=%.3f z=%.3f",
                table_id_.c_str(),
                object_id_.c_str(),
                object_pose.position.x,
                object_pose.position.y,
                object_pose.position.z);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  Task createTask() {
    Task task;
    task.stages()->setName("QR Pick Scan Place - grasp scan sort");
    task.loadRobotModel(shared_from_this());

    task.setProperty("group", planning_group_);
    task.setProperty("eef", gripper_group_);
    task.setProperty("ik_frame", eef_frame_);

    auto cartesian = std::make_shared<solvers::CartesianPath>();
    cartesian->setMaxVelocityScalingFactor(0.2);
    cartesian->setMaxAccelerationScalingFactor(0.2);

    auto joint = std::make_shared<solvers::JointInterpolationPlanner>();

    task.add(std::make_unique<stages::CurrentState>("current state"));

    {
      auto stage = std::make_unique<stages::MoveTo>("move to ready", joint);
      stage->setGroup(planning_group_);
      stage->setGoal(ready_named_target_);
      task.add(std::move(stage));
    }

    if (use_gripper_stages_) {
      task.add(makeOpenGripperMove(joint));
    }

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow gripper-object contact");
      stage->allowCollisions(object_id_, gripperCollisionLinks(task), true);
      task.add(std::move(stage));
    }

    task.add(makeMoveToPose("move to pick pose", pick_pose_, joint));
    task.add(makeCartesianMove("approach object", 0.0, 0.0, -approach_distance_, cartesian));

    if (use_gripper_stages_) {
      task.add(makeCloseGripperMove(joint));
    }

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("attach workpiece to gripper");
      stage->attachObject(object_id_, eef_frame_);
      task.add(std::move(stage));
    }

    task.add(makeCartesianMove("lift attached object", 0.0, 0.0, lift_distance_, cartesian));
    task.add(makeMoveToPose("move attached object to QR scan pose", scan_pose_, joint));

    // In the final report/demo, this is the scanning station step:
    // zbar_ros decodes the QR image, qr_decision_node maps it to /qr_sort/bin_pose,
    // and this task uses selected_bin_pose_ as the target placement bin.
    const auto place_pose = poseWithZOffset(selected_bin_pose_, bin_place_z_offset_);
    const auto above_bin_pose = poseWithZOffset(place_pose, lower_distance_);
    task.add(makeMoveToPose("move attached object above selected bin opening", above_bin_pose, joint));
    task.add(makeCartesianMove("lower attached object into bin", 0.0, 0.0, -lower_distance_, cartesian));

    if (use_gripper_stages_) {
      task.add(makeOpenGripperAtBinMove(joint));
    }

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("detach workpiece in bin");
      stage->detachObject(object_id_, eef_frame_);
      task.add(std::move(stage));
    }

    task.add(makeCartesianMove("retreat from bin", 0.0, 0.0, retreat_distance_, cartesian));

    {
      auto stage = std::make_unique<stages::MoveTo>("return home", joint);
      stage->setGroup(planning_group_);
      stage->setGoal(home_named_target_);
      task.add(std::move(stage));
    }

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid gripper-object contact after release");
      stage->allowCollisions(object_id_, gripperCollisionLinks(task), false);
      task.add(std::move(stage));
    }

    return task;
  }

  Task createPickToScanTask() {
    Task task;
    task.stages()->setName("QR Pick To Scan - trigger QR decode");
    task.loadRobotModel(shared_from_this());

    task.setProperty("group", planning_group_);
    task.setProperty("eef", gripper_group_);
    task.setProperty("ik_frame", eef_frame_);

    auto cartesian = std::make_shared<solvers::CartesianPath>();
    cartesian->setMaxVelocityScalingFactor(0.2);
    cartesian->setMaxAccelerationScalingFactor(0.2);

    auto joint = std::make_shared<solvers::JointInterpolationPlanner>();

    task.add(std::make_unique<stages::CurrentState>("current state"));

    {
      auto stage = std::make_unique<stages::MoveTo>("move to ready", joint);
      stage->setGroup(planning_group_);
      stage->setGoal(ready_named_target_);
      task.add(std::move(stage));
    }

    if (use_gripper_stages_) {
      task.add(makeOpenGripperMove(joint));
    }

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("allow gripper-object contact");
      stage->allowCollisions(object_id_, gripperCollisionLinks(task), true);
      task.add(std::move(stage));
    }

    task.add(makeMoveToPose("move to pick pose", pick_pose_, joint));
    task.add(makeCartesianMove("approach object", 0.0, 0.0, -approach_distance_, cartesian));

    if (use_gripper_stages_) {
      task.add(makeCloseGripperMove(joint));
    }

    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>("attach workpiece to gripper");
      stage->attachObject(object_id_, eef_frame_);
      task.add(std::move(stage));
    }

    task.add(makeCartesianMove("lift attached object", 0.0, 0.0, lift_distance_, cartesian));
    task.add(makeMoveToPose("move attached object to QR scan pose", scan_pose_, joint));

    return task;
  }

  void startSolutionRepublisher() {
    solution_republish_timer_ = create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        if (active_task_ && active_task_->numSolutions() > 0) {
          active_task_->introspection().publishSolution(*active_task_->solutions().front());
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 10000,
            "Republished MTC solution for RViz visualization.");
        }
      });
  }

  void publishScanReady() {
    if (scan_ready_sent_) {
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data = true;
    scan_ready_pub_->publish(msg);
    scan_ready_sent_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Pick-to-scan solution reached the QR scan stage. Published /qr_sort/scan_ready for simulated zbar input.");
  }

  void planPickToScanTask() {
    RCLCPP_INFO(get_logger(), "Planning pick-to-scan phase before QR decode.");

    addPlanningSceneObjects();

    active_task_ = std::make_unique<Task>(createPickToScanTask());
    auto& task = *active_task_;

    try {
      if (task.plan(5) && task.numSolutions() > 0) {
        RCLCPP_INFO(get_logger(), "Pick-to-scan planning succeeded. Solutions: %zu", task.numSolutions());
        task.introspection().publishSolution(*task.solutions().front());
        startSolutionRepublisher();
        publishScanReady();
      } else {
        RCLCPP_ERROR(get_logger(), "Pick-to-scan planning failed or produced zero complete solutions.");
        std::ostringstream failures;
        task.explainFailure(failures);
        RCLCPP_ERROR(get_logger(), "Failure explanation:\n%s", failures.str().c_str());
      }
    } catch (const InitStageException& ex) {
      RCLCPP_ERROR(get_logger(), "Pick-to-scan MTC initialization error: %s", ex.what());
      std::ostringstream details;
      details << task;
      RCLCPP_ERROR(get_logger(), "Task details:\n%s", details.str().c_str());
    }
  }

  void planTask() {
    RCLCPP_INFO(get_logger(), "Planning QR pick-scan-place task. execute=%s", execute_ ? "true" : "false");

    addPlanningSceneObjects();

    active_task_ = std::make_unique<Task>(createTask());
    auto& task = *active_task_;

    try {
      if (task.plan(10) && task.numSolutions() > 0) {
        RCLCPP_INFO(get_logger(), "Planning succeeded. Solutions: %zu", task.numSolutions());
        RCLCPP_INFO(get_logger(), "The task includes object approach, gripper close, attach, lift, scan pose, bin placement, detach, and retreat.");

        task.introspection().publishSolution(*task.solutions().front());
        startSolutionRepublisher();

        if (execute_) {
          RCLCPP_WARN(get_logger(), "Execution requested. Only enable with a valid fake/real trajectory controller.");
          task.execute(*task.solutions().front());
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Planning failed or produced zero complete solutions.");
        std::ostringstream failures;
        task.explainFailure(failures);
        RCLCPP_ERROR(get_logger(), "Failure explanation:\n%s", failures.str().c_str());
      }
    } catch (const InitStageException& ex) {
      RCLCPP_ERROR(get_logger(), "MTC initialization error: %s", ex.what());
      std::ostringstream details;
      details << task;
      RCLCPP_ERROR(get_logger(), "Task details:\n%s", details.str().c_str());
    }
  }

  std::string planning_group_;
  std::string gripper_group_;
  std::string eef_frame_;
  std::string base_frame_;
  std::string home_named_target_;
  std::string ready_named_target_;
  std::string gripper_open_named_target_;
  std::string gripper_closed_named_target_;
  std::string object_id_;
  std::string table_id_;

  bool execute_{false};
  bool use_gripper_stages_{true};
  bool use_gripper_width_targets_{true};
  bool have_bin_pose_{false};
  bool pick_to_scan_planned_{false};
  bool scan_ready_sent_{false};
  bool planned_{false};

  double approach_distance_{0.02};
  double lift_distance_{0.05};
  double gripper_open_width_{0.04};
  double gripper_grasp_width_{0.028};
  double lower_distance_{0.10};
  double retreat_distance_{0.10};
  double bin_outer_size_{0.20};
  double bin_wall_thickness_{0.010};
  double bin_wall_height_{0.10};
  double bin_floor_thickness_{0.012};
  double bin_bottom_offset_{0.055};
  double bin_place_z_offset_{0.04};

  std::vector<double> object_size_{0.06, 0.06, 0.06};
  std::vector<double> table_size_{0.16, 0.16, 0.02};
  std::vector<double> table_pose_{0.45, 0.0, 0.20};
  std::vector<double> grasp_frame_offset_{0.0, 0.0, 0.1034};

  geometry_msgs::msg::PoseStamped pick_pose_;
  geometry_msgs::msg::PoseStamped scan_pose_;
  geometry_msgs::msg::PoseStamped selected_bin_pose_;
  geometry_msgs::msg::PoseStamped bin_a_pose_;
  geometry_msgs::msg::PoseStamped bin_b_pose_;
  geometry_msgs::msg::PoseStamped bin_c_pose_;
  geometry_msgs::msg::PoseStamped reject_bin_pose_;

  std::unique_ptr<Task> active_task_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr bin_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr scan_ready_pub_;
  rclcpp::TimerBase::SharedPtr plan_timer_;
  rclcpp::TimerBase::SharedPtr solution_republish_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickScanPlaceMtcNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
