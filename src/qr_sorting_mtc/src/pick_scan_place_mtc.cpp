/**
 * @file pick_scan_place_mtc.cpp
 * @brief MoveIt Task Constructor scaffold for a QR-based pick-scan-place workflow.
 *
 * This file intentionally keeps all robot-specific names as ROS parameters so it can be
 * adapted to the professor's tutorial robot. It plans the high-level sequence:
 *   current -> ready -> pick -> lift -> scan -> selected bin -> lower -> retreat -> home
 *
 * Important local work:
 *   1. Confirm planning_group, gripper_group, eef_frame, and named SRDF states.
 *   2. Add real gripper open/close stages if your robot exposes gripper named states.
 *   3. Add collision objects for table, object, and bins in your planning scene.
 */

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/planning_scene/planning_scene.h>

using moveit::task_constructor::InitStageException;
using moveit::task_constructor::Stage;
using moveit::task_constructor::Task;
namespace stages = moveit::task_constructor::stages;
namespace solvers = moveit::task_constructor::solvers;

class PickScanPlaceMtcNode : public rclcpp::Node {
public:
  PickScanPlaceMtcNode() : Node("pick_scan_place_mtc") {
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");
    gripper_group_ = declare_parameter<std::string>("gripper_group", "gripper");
    eef_frame_ = declare_parameter<std::string>("eef_frame", "link6_flange");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    execute_ = declare_parameter<bool>("execute", false);
    home_named_target_ = declare_parameter<std::string>("home_named_target", "home");
    ready_named_target_ = declare_parameter<std::string>("ready_named_target", "ready");
    approach_distance_ = declare_parameter<double>("approach_distance", 0.08);
    lift_distance_ = declare_parameter<double>("lift_distance", 0.10);
    lower_distance_ = declare_parameter<double>("lower_distance", 0.08);
    retreat_distance_ = declare_parameter<double>("retreat_distance", 0.08);

    pick_pose_ = poseFromParameter("pick_pose", {0.25, 0.00, 0.14, 0.0, 1.0, 0.0, 0.0});
    scan_pose_ = poseFromParameter("scan_pose", {0.18, -0.30, 0.32, 0.0, 1.0, 0.0, 0.0});
    selected_bin_pose_ = poseFromVector({0.35, 0.25, 0.18, 0.0, 0.0, 0.0, 1.0});

    bin_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/qr_sort/bin_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        selected_bin_pose_ = *msg;
        have_bin_pose_ = true;
        RCLCPP_INFO(get_logger(), "Received selected bin pose: x=%.3f y=%.3f z=%.3f",
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
      });

    plan_timer_ = create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        if (planned_) return;
        if (!have_bin_pose_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Waiting for /qr_sort/bin_pose before planning full pick-scan-place task.");
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
    stage->setIKFrame(eef_frame_);
    stage->setGoal(pose);
    return stage;
  }

  std::unique_ptr<stages::MoveRelative> makeCartesianMove(
      const std::string& name,
      double dx, double dy, double dz,
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

  Task createTask() {
    Task task;
    task.stages()->setName("QR Pick Scan Place");
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

    task.add(makeMoveToPose("move to pick pose", pick_pose_, joint));
    task.add(makeCartesianMove("approach object", 0.0, 0.0, -approach_distance_, cartesian));

    // TODO: Replace with robot-specific gripper close stage if available.
    // Example: MoveTo("close gripper", joint)->setGroup(gripper_group_)->setGoal("closed");
    // For a stronger demo, attach the object to the end effector in the planning scene here.

    task.add(makeCartesianMove("lift object", 0.0, 0.0, lift_distance_, cartesian));
    task.add(makeMoveToPose("move to QR scan pose", scan_pose_, joint));

    // The QR decision is handled by qr_decision_node. This node waits for /qr_sort/bin_pose before planning.
    task.add(makeMoveToPose("move to selected bin", selected_bin_pose_, joint));
    task.add(makeCartesianMove("lower object", 0.0, 0.0, -lower_distance_, cartesian));

    // TODO: Replace with robot-specific gripper open stage and object detach.

    task.add(makeCartesianMove("retreat from bin", 0.0, 0.0, retreat_distance_, cartesian));

    {
      auto stage = std::make_unique<stages::MoveTo>("return home", joint);
      stage->setGroup(planning_group_);
      stage->setGoal(home_named_target_);
      task.add(std::move(stage));
    }

    return task;
  }

  void planTask() {
    RCLCPP_INFO(get_logger(), "Planning QR pick-scan-place task. execute=%s", execute_ ? "true" : "false");
    auto task = createTask();

    try {
      if (task.plan(5) && task.numSolutions() > 0) {
        RCLCPP_INFO(get_logger(), "Planning succeeded. Solutions: %zu", task.numSolutions());
        task.introspection().publishSolution(*task.solutions().front());
        if (execute_) {
          RCLCPP_WARN(get_logger(), "Execution requested. Confirm simulation/controller safety before enabling this in final demo.");
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
  bool execute_{false};
  bool have_bin_pose_{false};
  bool planned_{false};
  double approach_distance_{0.08};
  double lift_distance_{0.10};
  double lower_distance_{0.08};
  double retreat_distance_{0.08};
  geometry_msgs::msg::PoseStamped pick_pose_;
  geometry_msgs::msg::PoseStamped scan_pose_;
  geometry_msgs::msg::PoseStamped selected_bin_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr bin_pose_sub_;
  rclcpp::TimerBase::SharedPtr plan_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickScanPlaceMtcNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
