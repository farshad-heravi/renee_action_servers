// ArmJointMotionPlan action server: plans and executes to a joint-space target via MoveIt.
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_ros/buffer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "renee_action_servers/action/arm_joint_motion_plan.hpp"

using ArmJointMotionPlan = renee_action_servers::action::ArmJointMotionPlan;
using GoalHandleArmJointMotionPlan = rclcpp_action::ServerGoalHandle<ArmJointMotionPlan>;

class ArmJointMotionPlanServer : public rclcpp::Node
{
public:
  ArmJointMotionPlanServer()
  : rclcpp::Node("arm_joint_motion_plan_server")
  {
    planning_group_ = this->declare_parameter<std::string>("planning_group", "arm");
    move_group_ns_ = this->declare_parameter<std::string>("move_group_namespace", "robot");
    default_velocity_scaling_ = this->declare_parameter<double>("default_velocity_scaling", 0.25);
    default_acceleration_scaling_ = this->declare_parameter<double>("default_acceleration_scaling", 0.25);

    action_server_ = rclcpp_action::create_server<ArmJointMotionPlan>(
      this,
      "moveit_arm_joint_motion_plan",
      std::bind(&ArmJointMotionPlanServer::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArmJointMotionPlanServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ArmJointMotionPlanServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "ArmJointMotionPlan server on 'moveit_arm_joint_motion_plan' (ns='%s', group='%s')",
      move_group_ns_.c_str(), planning_group_.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ArmJointMotionPlan::Goal> goal)
  {
    if (goal->joint_names.empty() || goal->joint_positions.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: joint_names and joint_positions must be non-empty");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->joint_names.size() != goal->joint_positions.size()) {
      RCLCPP_WARN(get_logger(),
        "Rejecting goal: joint_names size (%zu) != joint_positions size (%zu)",
        goal->joint_names.size(), goal->joint_positions.size());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->planning_pipeline_id.empty() || goal->planner_id.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: planning_pipeline_id and planner_id must be non-empty");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleArmJointMotionPlan>)
  {
    std::lock_guard<std::mutex> lock(mgi_mutex_);
    if (active_mgi_) {
      active_mgi_->stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleArmJointMotionPlan> goal_handle)
  {
    std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleArmJointMotionPlan> goal_handle)
  {
    rclcpp::Node::SharedPtr node = shared_from_this();
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ArmJointMotionPlan::Result>();

    moveit::planning_interface::MoveGroupInterface::Options opt(
      planning_group_, moveit::planning_interface::MoveGroupInterface::ROBOT_DESCRIPTION,
      move_group_ns_);

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi;
    try {
      mgi = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node, opt, std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration::from_seconds(30.0));
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "MoveGroupInterface failed: %s", ex.what());
      result->success = false;
      result->message = std::string("MoveGroupInterface: ") + ex.what();
      goal_handle->abort(result);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mgi_mutex_);
      active_mgi_ = mgi;
    }

    mgi->setPlanningPipelineId(goal->planning_pipeline_id);
    mgi->setPlannerId(goal->planner_id);

    const double v = (goal->velocity_scaling > 0.0) ? goal->velocity_scaling : default_velocity_scaling_;
    const double a = (goal->acceleration_scaling > 0.0) ? goal->acceleration_scaling : default_acceleration_scaling_;
    mgi->setMaxVelocityScalingFactor(v);
    mgi->setMaxAccelerationScalingFactor(a);

    std::map<std::string, double> joint_target;
    for (size_t i = 0; i < goal->joint_names.size(); ++i) {
      joint_target[goal->joint_names[i]] = goal->joint_positions[i];
    }
    mgi->setJointValueTarget(joint_target);

    RCLCPP_INFO(get_logger(), "Planning [%s / %s] to %zu joint targets",
      goal->planning_pipeline_id.c_str(), goal->planner_id.c_str(),
      goal->joint_names.size());

    const moveit::core::MoveItErrorCode err = mgi->move();
    const bool ok = static_cast<bool>(err);

    {
      std::lock_guard<std::mutex> lock(mgi_mutex_);
      if (active_mgi_ == mgi) {
        active_mgi_.reset();
      }
    }

    result->success = ok;
    result->message = ok ? std::string("OK") : std::string("move() failed: ") + std::to_string(err.val);

    if (ok) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  rclcpp_action::Server<ArmJointMotionPlan>::SharedPtr action_server_;
  std::string planning_group_;
  std::string move_group_ns_;
  double default_velocity_scaling_{0.25};
  double default_acceleration_scaling_{0.25};

  std::mutex mgi_mutex_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> active_mgi_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmJointMotionPlanServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
