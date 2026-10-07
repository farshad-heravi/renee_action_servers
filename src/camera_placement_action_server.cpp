/**
 * @file camera_placement_action_server.cpp
 * @brief Computes camera placement and exposes it through /camera_placement.
 */

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <set>
#include <sstream>
#include <utility>
#include <vector>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <thread>

#include "renee_action_servers/action/camera_placement.hpp"

namespace renee_action_servers
{

namespace
{

constexpr char kZed2iCameraLink[] =
  "robot_arm_rgbd_camera_left_camera_optical_frame";

}  // namespace


/** @brief Input required to solve one camera placement. */
struct PlanningRequest
{
  geometry_msgs::msg::PoseStamped camera_pose;
  std::string camera_link;
  bool lock_current_base{false};
  bool allow_long_arm_motion{false};
};

/** @brief Result of the coordinated base and arm placement search. */
struct PlanningSolution
{
  bool success{false};
  std::string message;
  geometry_msgs::msg::PoseStamped base_pose;
  geometry_msgs::msg::PoseStamped end_effector_pose;
  sensor_msgs::msg::JointState arm_solution;
  double score{0.0};
};

/** @brief Progress information emitted while candidates are evaluated. */
struct PlanningFeedback
{
  std::string phase;
  std::uint32_t candidates_evaluated{0};
  std::uint32_t valid_candidates{0};
};

using FeedbackCallback = std::function<void(const PlanningFeedback &)>;
using CancelCallback = std::function<bool()>;

/**
 * @brief Finds a navigable base pose and a collision-free UR5 IK solution.
 *
 * Positions are expressed in metres, angles in radians, and global poses in
 * the configured planning frame (normally `robot_map`).
 */
class CameraPlacementPlanner
{
public:
  /**
   * @brief Constructs the planner and its MoveIt, Nav2, and TF clients.
   * @param node ROS node that owns parameters and clients.
   * @param tf_buffer Shared TF buffer used to resolve robot and camera frames.
   */
  CameraPlacementPlanner(
    rclcpp::Node & node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  /**
   * @brief Computes the best valid base and arm placement.
   * @param request Desired optical-camera pose and planning options.
   * @param feedback Callback used to report planning progress.
   * @param cancelled Callback used to detect action cancellation.
   * @return Best valid placement, or a failed solution with an explanatory message.
   */
  PlanningSolution plan(
    const PlanningRequest & request,
    const FeedbackCallback & feedback,
    const CancelCallback & cancelled);

  /** @brief Resolves an optional goal override against server configuration and fallback. */
  std::string resolveCameraLink(const std::string & requested_camera_link) const;

private:
  struct MoveItDiagnostics
  {
    std::uint32_t collision_disabled_ik_successes{0};
    std::uint32_t collision_disabled_ik_failures{0};
    std::uint32_t collision_rejections{0};
    std::uint32_t valid_collision_disabled_solutions{0};
    std::uint32_t joint_path_rejections{0};
    std::uint32_t long_motion_rejections{0};
    std::uint32_t elbow_down_rejections{0};
    std::map<std::int32_t, std::uint32_t> collision_aware_error_codes;
    std::map<std::int32_t, std::uint32_t> collision_disabled_error_codes;
    std::set<std::string> collision_pairs;
  };

  struct CandidateSolution
  {
    geometry_msgs::msg::PoseStamped base_pose;
    sensor_msgs::msg::JointState arm_solution;
    double arm_cost{0.0};
    double navigation_cost{0.0};
    double score{0.0};
  };

  void validateRequest(const PlanningRequest & request) const;
  geometry_msgs::msg::PoseStamped cameraPoseToTool0(
    const PlanningRequest & request) const;
  geometry_msgs::msg::PoseStamped currentBasePose() const;
  std::vector<geometry_msgs::msg::PoseStamped> generateBaseCandidates(
    const PlanningRequest & request) const;
  std::vector<double> currentArmJoints() const;
  moveit_msgs::msg::RobotState baseRobotState(
    const geometry_msgs::msg::PoseStamped & base_pose) const;
  /** @brief Camera, shoulder, elbow and wrist positions of one arm state (global frame). */
  struct ArmPoints
  {
    tf2::Vector3 camera;
    tf2::Vector3 shoulder;
    tf2::Vector3 elbow;
    tf2::Vector3 wrist;
  };

  bool evaluateWithMoveIt(
    const geometry_msgs::msg::PoseStamped & base_pose,
    const geometry_msgs::msg::PoseStamped & tool0_pose,
    const std::string & camera_link,
    const std::optional<std::vector<double>> & ptp_start,
    bool allow_long_arm_motion,
    CandidateSolution & candidate,
    MoveItDiagnostics & diagnostics,
    const CancelCallback & cancelled);
  void unwrapTowards(const std::vector<double> & reference, std::vector<double> & positions) const;
  std::optional<ArmPoints> computeArmPoints(
    const geometry_msgs::msg::PoseStamped & base_pose,
    const std::vector<double> & positions,
    const std::string & camera_link,
    const CancelCallback & cancelled);
  bool isElbowUp(const ArmPoints & points) const;
  bool isArmMotionShort(
    const geometry_msgs::msg::PoseStamped & base_pose,
    const std::vector<double> & start,
    const std::vector<double> & goal,
    const std::string & camera_link,
    bool allow_long_arm_motion,
    double & camera_path_length,
    MoveItDiagnostics & diagnostics,
    const CancelCallback & cancelled);
  bool solveIk(
    const geometry_msgs::msg::PoseStamped & base_pose,
    const geometry_msgs::msg::PoseStamped & tool0_pose,
    const std::optional<std::vector<double>> & seed,
    CandidateSolution & candidate,
    MoveItDiagnostics & diagnostics,
    const CancelCallback & cancelled);
  bool isJointPathValid(
    const geometry_msgs::msg::PoseStamped & base_pose,
    const std::vector<double> & start,
    const std::vector<double> & goal,
    const CancelCallback & cancelled);
  bool evaluateWithNav2(
    CandidateSolution & candidate,
    const CancelCallback & cancelled);
  double calculateScore(const CandidateSolution & candidate) const;
  void publishFeedback(
    const FeedbackCallback & callback,
    const std::string & phase,
    std::uint32_t evaluated,
    std::uint32_t valid) const;

  rclcpp::Node & node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Client<moveit_msgs::srv::GetPositionFK>::SharedPtr fk_client_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr validity_client_;
  rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr navigation_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  mutable std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  std::mt19937 rng_{std::random_device{}()};

  std::string global_frame_;
  std::string base_frame_;
  std::string tool0_link_;
  std::string default_camera_link_;
  std::string virtual_joint_name_;
  std::string planning_group_;
  std::vector<std::string> arm_joint_names_;
  double min_base_radius_;
  double max_base_radius_;
  double base_radius_step_;
  int angular_samples_;
  double yaw_offset_;
  std::string base_yaw_mode_;
  double base_yaw_tolerance_;
  double max_base_yaw_change_rad_;
  int max_navigation_candidates_;
  double service_timeout_sec_;
  double ik_timeout_sec_;
  bool diagnose_ik_failures_;
  int ik_attempts_;
  double joint_path_resolution_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  double max_joint_delta_rad_;
  double max_ee_path_ratio_;
  double max_ee_path_slack_m_;
  bool require_elbow_up_;
  std::string shoulder_link_;
  std::string elbow_link_;
  std::string wrist_link_;
};


namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kQuaternionEpsilon = 1e-9;

bool isFinitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

double quaternionNorm(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
}

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

double pathLength(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (std::size_t index = 1; index < path.poses.size(); ++index) {
    const auto & previous = path.poses[index - 1].pose.position;
    const auto & current = path.poses[index].pose.position;
    length += std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return length;
}

}  // namespace

CameraPlacementPlanner::CameraPlacementPlanner(
  rclcpp::Node & node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
: node_(node), tf_buffer_(std::move(tf_buffer))
{
  global_frame_ = node_.declare_parameter<std::string>(
    "camera_placement.global_frame", "robot_map");
  base_frame_ = node_.declare_parameter<std::string>(
    "camera_placement.base_frame", "robot_base_footprint");
  tool0_link_ = node_.declare_parameter<std::string>(
    "camera_placement.tool0_link", "robot_arm_tool0");
  default_camera_link_ = node_.declare_parameter<std::string>(
    "camera_placement.camera_link", "");
  virtual_joint_name_ = node_.declare_parameter<std::string>(
    "camera_placement.virtual_joint", "virtual_joint");
  planning_group_ = node_.declare_parameter<std::string>(
    "camera_placement.planning_group", "arm");
  arm_joint_names_ = node_.declare_parameter<std::vector<std::string>>(
    "camera_placement.arm_joint_names",
    {"robot_arm_shoulder_pan_joint", "robot_arm_shoulder_lift_joint",
      "robot_arm_elbow_joint", "robot_arm_wrist_1_joint",
      "robot_arm_wrist_2_joint", "robot_arm_wrist_3_joint"});
  min_base_radius_ = node_.declare_parameter<double>(
    "camera_placement.min_base_radius", 0.4);
  max_base_radius_ = node_.declare_parameter<double>(
    "camera_placement.max_base_radius", 0.8);
  base_radius_step_ = node_.declare_parameter<double>(
    "camera_placement.base_radius_step", 0.2);
  angular_samples_ = node_.declare_parameter<int>(
    "camera_placement.angular_samples", 12);
  yaw_offset_ = node_.declare_parameter<double>(
    "camera_placement.yaw_offset", 0.35);
  // Base candidate heading. face_camera: towards the camera target
  // (+-yaw_offset). keep_current: the current base heading
  // (+-base_yaw_tolerance), so the base only translates along the machine
  // (sideways, forward or in reverse) instead of rotating.
  base_yaw_mode_ = node_.declare_parameter<std::string>(
    "camera_placement.base_yaw_mode", "keep_current");
  if (base_yaw_mode_ != "face_camera" && base_yaw_mode_ != "keep_current") {
    throw std::invalid_argument(
            "camera_placement.base_yaw_mode must be 'face_camera' or 'keep_current'");
  }
  base_yaw_tolerance_ = node_.declare_parameter<double>(
    "camera_placement.base_yaw_tolerance", 0.1);
  // Candidates whose heading differs more than this from the current base
  // heading are dropped, in both modes (<= 0 disables).
  max_base_yaw_change_rad_ = node_.declare_parameter<double>(
    "camera_placement.max_base_yaw_change_rad", 0.785);
  max_navigation_candidates_ = node_.declare_parameter<int>(
    "camera_placement.max_navigation_candidates", 12);
  service_timeout_sec_ = node_.declare_parameter<double>(
    "camera_placement.service_timeout_sec", 10.0);
  ik_timeout_sec_ = node_.declare_parameter<double>(
    "camera_placement.ik_timeout_sec", 0.15);
  diagnose_ik_failures_ = node_.declare_parameter<bool>(
    "camera_placement.diagnose_ik_failures", false);
  ik_attempts_ = std::max(
    1, static_cast<int>(node_.declare_parameter<int>("camera_placement.ik_attempts", 8)));
  joint_path_resolution_ = node_.declare_parameter<double>(
    "camera_placement.joint_path_resolution", 0.05);
  // UR joint limits (rad), in arm_joint_names order; used to pick the 2*pi
  // equivalent of each IK joint value closest to the current arm state.
  joint_lower_limits_ = node_.declare_parameter<std::vector<double>>(
    "camera_placement.joint_lower_limits",
    {-2.0 * kPi, -2.0 * kPi, -kPi, -2.0 * kPi, -2.0 * kPi, -2.0 * kPi});
  joint_upper_limits_ = node_.declare_parameter<std::vector<double>>(
    "camera_placement.joint_upper_limits",
    {2.0 * kPi, 2.0 * kPi, kPi, 2.0 * kPi, 2.0 * kPi, 2.0 * kPi});
  if (joint_lower_limits_.size() != arm_joint_names_.size() ||
    joint_upper_limits_.size() != arm_joint_names_.size())
  {
    throw std::invalid_argument(
            "camera_placement.joint_lower_limits/joint_upper_limits must have one value per arm joint");
  }
  // Short arm motions between targets (with a locked base): no joint turns
  // more than max_joint_delta_rad, and the camera path is at most
  // max_ee_path_ratio * straight-line distance + max_ee_path_slack_m.
  // A value <= 0 disables that limit.
  max_joint_delta_rad_ = node_.declare_parameter<double>(
    "camera_placement.max_joint_delta_rad", 1.57);
  max_ee_path_ratio_ = node_.declare_parameter<double>(
    "camera_placement.max_ee_path_ratio", 1.5);
  max_ee_path_slack_m_ = node_.declare_parameter<double>(
    "camera_placement.max_ee_path_slack_m", 0.15);
  // Elbow up: the elbow joint above the shoulder-wrist line, in the global frame.
  require_elbow_up_ = node_.declare_parameter<bool>(
    "camera_placement.require_elbow_up", true);
  shoulder_link_ = node_.declare_parameter<std::string>(
    "camera_placement.shoulder_link", "robot_arm_upper_arm_link");
  elbow_link_ = node_.declare_parameter<std::string>(
    "camera_placement.elbow_link", "robot_arm_forearm_link");
  wrist_link_ = node_.declare_parameter<std::string>(
    "camera_placement.wrist_link", "robot_arm_wrist_1_link");

  const auto ik_service = node_.declare_parameter<std::string>(
    "camera_placement.ik_service", "/robot/compute_ik");
  const auto fk_service = node_.declare_parameter<std::string>(
    "camera_placement.fk_service", "/robot/compute_fk");
  const auto validity_service = node_.declare_parameter<std::string>(
    "camera_placement.validity_service", "/robot/check_state_validity");
  const auto navigation_action = node_.declare_parameter<std::string>(
    "camera_placement.navigation_action", "/robot/compute_path_to_pose");
  const auto joint_states_topic = node_.declare_parameter<std::string>(
    "camera_placement.joint_states_topic", "/robot/joint_states");

  ik_client_ = node_.create_client<moveit_msgs::srv::GetPositionIK>(ik_service);
  fk_client_ = node_.create_client<moveit_msgs::srv::GetPositionFK>(fk_service);
  validity_client_ = node_.create_client<moveit_msgs::srv::GetStateValidity>(validity_service);
  navigation_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
    &node_, navigation_action);
  joint_state_sub_ = node_.create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr message) {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      latest_joint_state_ = *message;
    });
}

std::string CameraPlacementPlanner::resolveCameraLink(
  const std::string & requested_camera_link) const
{
  if (!requested_camera_link.empty()) {
    return requested_camera_link;
  }
  if (!default_camera_link_.empty()) {
    return default_camera_link_;
  }

  RCLCPP_WARN(
    node_.get_logger(),
    "No camera link supplied by the goal or camera_placement.camera_link; "
    "using Stereolabs ZED2i fallback '%s'",
    kZed2iCameraLink);
  return kZed2iCameraLink;
}

PlanningSolution CameraPlacementPlanner::plan(
  const PlanningRequest & request,
  const FeedbackCallback & feedback,
  const CancelCallback & cancelled)
{
  PlanningSolution result;
  try {
    validateRequest(request);
    publishFeedback(feedback, "transforming_camera_pose", 0, 0);
    result.end_effector_pose = cameraPoseToTool0(request);

    if (cancelled && cancelled()) {
      result.message = "Camera placement cancelled";
      return result;
    }

    auto base_candidates = request.lock_current_base ?
      std::vector<geometry_msgs::msg::PoseStamped>{currentBasePose()} :
      generateBaseCandidates(request);

    // With a locked base the returned joints are executed directly with a
    // joint-space PTP motion, so the straight joint path from the current arm
    // state must also be collision-free, not just the final state.
    std::optional<std::vector<double>> ptp_start;
    if (request.lock_current_base) {
      ptp_start = currentArmJoints();
    }

    std::vector<CandidateSolution> arm_candidates;
    MoveItDiagnostics moveit_diagnostics;
    std::uint32_t evaluated = 0;
    for (const auto & base_pose : base_candidates) {
      if (cancelled && cancelled()) {
        result.message = "Camera placement cancelled";
        return result;
      }

      CandidateSolution candidate;
      candidate.base_pose = base_pose;
      if (evaluateWithMoveIt(
          base_pose, result.end_effector_pose, request.camera_link, ptp_start,
          request.allow_long_arm_motion, candidate, moveit_diagnostics, cancelled))
      {
        arm_candidates.push_back(std::move(candidate));
      }
      ++evaluated;
      publishFeedback(
        feedback, "checking_moveit", evaluated,
        static_cast<std::uint32_t>(arm_candidates.size()));
    }

    if (arm_candidates.empty()) {
      std::ostringstream message;
      message << "MoveIt found no collision-free IK solution for any base candidate";
      if (moveit_diagnostics.joint_path_rejections > 0) {
        message << " (" << moveit_diagnostics.joint_path_rejections
                << " IK solution(s) rejected because the PTP joint path from the "
                   "current arm state collides)";
      }
      if (moveit_diagnostics.long_motion_rejections > 0) {
        message << " (" << moveit_diagnostics.long_motion_rejections
                << " IK solution(s) rejected because the arm motion from the current "
                   "state is too long)";
      }
      if (moveit_diagnostics.elbow_down_rejections > 0) {
        message << " (" << moveit_diagnostics.elbow_down_rejections
                << " IK solution(s) rejected because the elbow is not up)";
      }
      if (diagnose_ik_failures_) {
        message << ". Diagnostics: collision-disabled IK succeeded for "
                << moveit_diagnostics.collision_disabled_ik_successes
                << " candidate(s) and failed for "
                << moveit_diagnostics.collision_disabled_ik_failures << " candidate(s)";
        if (moveit_diagnostics.collision_rejections > 0) {
          message << "; state validity rejected "
                  << moveit_diagnostics.collision_rejections << " candidate(s)";
        }
        if (moveit_diagnostics.valid_collision_disabled_solutions > 0) {
          message << "; " << moveit_diagnostics.valid_collision_disabled_solutions
                  << " collision-disabled solution(s) were state-valid, indicating an "
                     "IK timeout/search issue rather than collision";
        }
        if (!moveit_diagnostics.collision_pairs.empty()) {
          message << "; contacts: ";
          bool first = true;
          for (const auto & pair : moveit_diagnostics.collision_pairs) {
            if (!first) {
              message << ", ";
            }
            message << pair;
            first = false;
          }
        }
        if (!moveit_diagnostics.collision_disabled_error_codes.empty()) {
          message << "; collision-disabled MoveIt codes: ";
          bool first = true;
          for (const auto & entry : moveit_diagnostics.collision_disabled_error_codes)
          {
            if (!first) {
              message << ", ";
            }
            message << entry.first << " (" << entry.second << ')';
            first = false;
          }
        }
      }
      result.message = message.str();
      return result;
    }

    std::sort(
      arm_candidates.begin(), arm_candidates.end(),
      [](const CandidateSolution & left, const CandidateSolution & right) {
        return left.arm_cost < right.arm_cost;
      });

    if (!request.lock_current_base &&
      arm_candidates.size() > static_cast<std::size_t>(max_navigation_candidates_))
    {
      arm_candidates.resize(static_cast<std::size_t>(max_navigation_candidates_));
    }

    std::vector<CandidateSolution> valid_candidates;
    evaluated = 0;
    for (auto & candidate : arm_candidates) {
      if (cancelled && cancelled()) {
        result.message = "Camera placement cancelled";
        return result;
      }

      const bool navigation_valid = request.lock_current_base ||
        evaluateWithNav2(candidate, cancelled);
      if (navigation_valid) {
        candidate.score = calculateScore(candidate);
        valid_candidates.push_back(std::move(candidate));
      }
      ++evaluated;
      publishFeedback(
        feedback, request.lock_current_base ? "validating_current_base" : "checking_nav2",
        evaluated, static_cast<std::uint32_t>(valid_candidates.size()));
    }

    if (valid_candidates.empty()) {
      result.message = "Nav2 found no path to any kinematically valid base candidate";
      return result;
    }

    const auto best = std::min_element(
      valid_candidates.begin(), valid_candidates.end(),
      [](const CandidateSolution & left, const CandidateSolution & right) {
        return left.score < right.score;
      });

    result.success = true;
    result.message = "Camera placement found";
    result.base_pose = best->base_pose;
    result.arm_solution = best->arm_solution;
    result.score = best->score;
    publishFeedback(
      feedback, "completed", evaluated,
      static_cast<std::uint32_t>(valid_candidates.size()));
  } catch (const std::exception & exception) {
    result.message = exception.what();
  }
  return result;
}

void CameraPlacementPlanner::validateRequest(const PlanningRequest & request) const
{
  if (request.camera_pose.header.frame_id != global_frame_) {
    throw std::invalid_argument(
            "camera_pose must use global frame '" + global_frame_ + "'");
  }
  if (request.camera_link.empty()) {
    throw std::invalid_argument("camera_link cannot be empty");
  }
  if (!isFinitePose(request.camera_pose.pose)) {
    throw std::invalid_argument("camera_pose contains non-finite values");
  }
  if (quaternionNorm(request.camera_pose.pose.orientation) <= kQuaternionEpsilon) {
    throw std::invalid_argument("camera_pose orientation quaternion cannot be zero");
  }
  if (min_base_radius_ <= 0.0 || max_base_radius_ < min_base_radius_ ||
    base_radius_step_ <= 0.0 || angular_samples_ <= 0 || max_navigation_candidates_ <= 0)
  {
    throw std::invalid_argument("camera placement sampling parameters are invalid");
  }
}

geometry_msgs::msg::PoseStamped CameraPlacementPlanner::cameraPoseToTool0(
  const PlanningRequest & request) const
{
  const auto tool0_to_camera = tf_buffer_->lookupTransform(
    tool0_link_, request.camera_link, tf2::TimePointZero,
    tf2::durationFromSec(service_timeout_sec_));

  tf2::Transform map_to_camera;
  tf2::fromMsg(request.camera_pose.pose, map_to_camera);
  map_to_camera.getRotation().normalize();

  tf2::Transform tool0_to_camera_tf;
  tf2::fromMsg(tool0_to_camera.transform, tool0_to_camera_tf);

  const tf2::Transform map_to_tool0 = map_to_camera * tool0_to_camera_tf.inverse();
  geometry_msgs::msg::PoseStamped tool0_pose;
  tool0_pose.header.frame_id = global_frame_;
  tool0_pose.header.stamp = node_.now();
  tool0_pose.pose.position.x = map_to_tool0.getOrigin().x();
  tool0_pose.pose.position.y = map_to_tool0.getOrigin().y();
  tool0_pose.pose.position.z = map_to_tool0.getOrigin().z();
  tool0_pose.pose.orientation = tf2::toMsg(map_to_tool0.getRotation());
  return tool0_pose;
}

geometry_msgs::msg::PoseStamped CameraPlacementPlanner::currentBasePose() const
{
  const auto transform = tf_buffer_->lookupTransform(
    global_frame_, base_frame_, tf2::TimePointZero,
    tf2::durationFromSec(service_timeout_sec_));
  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;


  RCLCPP_INFO(
    node_.get_logger(), "Current base pose: (%.3f, %.3f, %.3f) orientation (%.3f, %.3f, %.3f, %.3f)",
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
    pose.pose.orientation.x, pose.pose.orientation.y,
    pose.pose.orientation.z, pose.pose.orientation.w);
  return pose;
}

std::vector<geometry_msgs::msg::PoseStamped>
CameraPlacementPlanner::generateBaseCandidates(const PlanningRequest & request) const
{
  std::vector<geometry_msgs::msg::PoseStamped> candidates;
  const std::vector<double> yaw_offsets{-yaw_offset_, 0.0, yaw_offset_};

  const auto current = currentBasePose().pose.orientation;
  const double current_yaw = std::atan2(
    2.0 * (current.w * current.z + current.x * current.y),
    1.0 - 2.0 * (current.y * current.y + current.z * current.z));
  const auto within_yaw_change = [this, current_yaw](const double yaw) {
      const double change = std::abs(std::remainder(yaw - current_yaw, 2.0 * kPi));
      return max_base_yaw_change_rad_ <= 0.0 || change <= max_base_yaw_change_rad_;
    };

  // keep_current: the same headings for every position.
  std::vector<double> fixed_yaws;
  if (base_yaw_mode_ == "keep_current") {
    fixed_yaws.push_back(current_yaw);
    if (base_yaw_tolerance_ > 0.0) {
      fixed_yaws.push_back(current_yaw - base_yaw_tolerance_);
      fixed_yaws.push_back(current_yaw + base_yaw_tolerance_);
    }
  }

  for (double radius = min_base_radius_;
    radius <= max_base_radius_ + 1e-9; radius += base_radius_step_)
  {
    for (int index = 0; index < angular_samples_; ++index) {
      const double angle = 2.0 * kPi * static_cast<double>(index) /
        static_cast<double>(angular_samples_);
      const double x = request.camera_pose.pose.position.x + radius * std::cos(angle);
      const double y = request.camera_pose.pose.position.y + radius * std::sin(angle);
      const double face_camera_yaw = std::atan2(
        request.camera_pose.pose.position.y - y,
        request.camera_pose.pose.position.x - x);

      std::vector<double> yaws = fixed_yaws;
      if (yaws.empty()) {
        for (const double offset : yaw_offsets) {
          yaws.push_back(face_camera_yaw + offset);
        }
      }
      for (const double yaw : yaws) {
        if (!within_yaw_change(yaw)) {
          continue;
        }
        geometry_msgs::msg::PoseStamped candidate;
        candidate.header.frame_id = global_frame_;
        candidate.header.stamp = node_.now();
        candidate.pose.position.x = x;
        candidate.pose.position.y = y;
        candidate.pose.position.z = 0.0;
        candidate.pose.orientation = yawToQuaternion(yaw);
        candidates.push_back(std::move(candidate));
      }
    }
  }
  return candidates;
}

std::vector<double> CameraPlacementPlanner::currentArmJoints() const
{
  sensor_msgs::msg::JointState joint_state;
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    joint_state = latest_joint_state_;
  }

  std::vector<double> positions;
  for (const auto & joint_name : arm_joint_names_) {
    const auto found = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
    const auto index = static_cast<std::size_t>(std::distance(joint_state.name.begin(), found));
    if (found == joint_state.name.end() || index >= joint_state.position.size()) {
      throw std::runtime_error(
              "No joint state received for arm joint '" + joint_name + "'");
    }
    positions.push_back(joint_state.position[index]);
  }
  return positions;
}

moveit_msgs::msg::RobotState CameraPlacementPlanner::baseRobotState(
  const geometry_msgs::msg::PoseStamped & base_pose) const
{
  moveit_msgs::msg::RobotState state;
  state.is_diff = true;
  auto & multi_dof = state.multi_dof_joint_state;
  multi_dof.header.frame_id = global_frame_;
  multi_dof.header.stamp = node_.now();
  multi_dof.joint_names.push_back(virtual_joint_name_);
  geometry_msgs::msg::Transform base_transform;
  base_transform.translation.x = base_pose.pose.position.x;
  base_transform.translation.y = base_pose.pose.position.y;
  base_transform.translation.z = base_pose.pose.position.z;
  base_transform.rotation = base_pose.pose.orientation;
  multi_dof.transforms.push_back(base_transform);
  return state;
}

bool CameraPlacementPlanner::evaluateWithMoveIt(
  const geometry_msgs::msg::PoseStamped & base_pose,
  const geometry_msgs::msg::PoseStamped & tool0_pose,
  const std::string & camera_link,
  const std::optional<std::vector<double>> & ptp_start,
  bool allow_long_arm_motion,
  CandidateSolution & candidate,
  MoveItDiagnostics & diagnostics,
  const CancelCallback & cancelled)
{
  const auto timeout = std::chrono::duration<double>(service_timeout_sec_);
  if (!ik_client_->wait_for_service(timeout) || !validity_client_->wait_for_service(timeout) ||
    ((ptp_start || require_elbow_up_) && !fk_client_->wait_for_service(timeout)))
  {
    throw std::runtime_error("MoveIt IK, FK or state-validity service is unavailable");
  }

  // Without a PTP start (base candidates) the first IK solution with the
  // elbow up is enough: the arm motion is planned again from the reached base.
  // With one, the first attempt is seeded from the current arm state (nearest
  // IK branch) and kept if its motion is short and collision-free; otherwise
  // the remaining attempts use random seeds and the solution with the
  // shortest camera path is kept.
  const int attempts = (ptp_start || require_elbow_up_) ? ik_attempts_ : 1;
  std::uniform_real_distribution<double> random_joint(-kPi, kPi);
  std::optional<CandidateSolution> best;
  for (int attempt = 0; attempt < attempts; ++attempt) {
    if (cancelled && cancelled()) {
      return false;
    }
    std::optional<std::vector<double>> seed;
    if (ptp_start) {
      seed = *ptp_start;
    }
    if (attempt > 0) {
      seed = std::vector<double>(arm_joint_names_.size());
      for (auto & position : *seed) {
        position = random_joint(rng_);
      }
    }

    CandidateSolution attempt_candidate;
    attempt_candidate.base_pose = candidate.base_pose;
    if (!solveIk(base_pose, tool0_pose, seed, attempt_candidate, diagnostics, cancelled)) {
      continue;
    }
    auto & positions = attempt_candidate.arm_solution.position;
    if (!ptp_start) {
      if (require_elbow_up_) {
        const auto points = computeArmPoints(base_pose, positions, camera_link, cancelled);
        if (!points) {
          continue;
        }
        if (!isElbowUp(*points)) {
          ++diagnostics.elbow_down_rejections;
          continue;
        }
      }
      candidate = std::move(attempt_candidate);
      return true;
    }

    unwrapTowards(*ptp_start, positions);
    double camera_path_length = 0.0;
    if (!isArmMotionShort(
        base_pose, *ptp_start, positions, camera_link, allow_long_arm_motion,
        camera_path_length, diagnostics, cancelled))
    {
      continue;
    }
    if (!isJointPathValid(base_pose, *ptp_start, positions, cancelled)) {
      ++diagnostics.joint_path_rejections;
      continue;
    }
    attempt_candidate.arm_cost = camera_path_length;
    if (attempt == 0) {
      candidate = std::move(attempt_candidate);
      return true;
    }
    if (!best || attempt_candidate.arm_cost < best->arm_cost) {
      best = std::move(attempt_candidate);
    }
  }
  if (!best) {
    return false;
  }
  candidate = std::move(*best);
  return true;
}

void CameraPlacementPlanner::unwrapTowards(
  const std::vector<double> & reference, std::vector<double> & positions) const
{
  // UR joints span +-2*pi, so an IK value and the same value +-2*pi reach the
  // same pose; keep the one closest to the reference (no full turns).
  for (std::size_t index = 0; index < positions.size() && index < reference.size(); ++index) {
    double best = positions[index];
    for (int turns = -2; turns <= 2; ++turns) {
      const double value = positions[index] + 2.0 * kPi * static_cast<double>(turns);
      if (value >= joint_lower_limits_[index] && value <= joint_upper_limits_[index] &&
        std::abs(value - reference[index]) < std::abs(best - reference[index]))
      {
        best = value;
      }
    }
    positions[index] = best;
  }
}

std::optional<CameraPlacementPlanner::ArmPoints> CameraPlacementPlanner::computeArmPoints(
  const geometry_msgs::msg::PoseStamped & base_pose,
  const std::vector<double> & positions,
  const std::string & camera_link,
  const CancelCallback & cancelled)
{
  using namespace std::chrono_literals;
  auto request = std::make_shared<moveit_msgs::srv::GetPositionFK::Request>();
  request->header.frame_id = global_frame_;
  request->header.stamp = node_.now();
  request->fk_link_names = {camera_link, shoulder_link_, elbow_link_, wrist_link_};
  request->robot_state = baseRobotState(base_pose);
  request->robot_state.joint_state.name = arm_joint_names_;
  request->robot_state.joint_state.position = positions;

  auto future = fk_client_->async_send_request(request);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(service_timeout_sec_);
  while (future.wait_for(50ms) != std::future_status::ready) {
    if ((cancelled && cancelled()) || std::chrono::steady_clock::now() >= deadline) {
      return std::nullopt;
    }
  }
  const auto response = future.get();
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS ||
    response->pose_stamped.size() != request->fk_link_names.size())
  {
    RCLCPP_WARN(
      node_.get_logger(), "MoveIt FK failed (code %d) for the camera/arm links",
      response->error_code.val);
    return std::nullopt;
  }
  const auto point = [&response](const std::size_t index) {
      const auto & position = response->pose_stamped[index].pose.position;
      return tf2::Vector3(position.x, position.y, position.z);
    };
  return ArmPoints{point(0), point(1), point(2), point(3)};
}

bool CameraPlacementPlanner::isElbowUp(const ArmPoints & points) const
{
  // The elbow is up when it lies above the shoulder-wrist line (global z up).
  const tf2::Vector3 line = points.wrist - points.shoulder;
  const double length_squared = line.length2();
  if (length_squared < 1e-9) {
    return true;
  }
  const double fraction = (points.elbow - points.shoulder).dot(line) / length_squared;
  const tf2::Vector3 on_line = points.shoulder + line * fraction;
  return points.elbow.z() > on_line.z();
}

bool CameraPlacementPlanner::isArmMotionShort(
  const geometry_msgs::msg::PoseStamped & base_pose,
  const std::vector<double> & start,
  const std::vector<double> & goal,
  const std::string & camera_link,
  bool allow_long_arm_motion,
  double & camera_path_length,
  MoveItDiagnostics & diagnostics,
  const CancelCallback & cancelled)
{
  double max_delta = 0.0;
  for (std::size_t index = 0; index < start.size(); ++index) {
    max_delta = std::max(max_delta, std::abs(goal[index] - start[index]));
  }
  const bool limit_motion = !allow_long_arm_motion;
  if (limit_motion && max_joint_delta_rad_ > 0.0 && max_delta > max_joint_delta_rad_) {
    ++diagnostics.long_motion_rejections;
    return false;
  }
  camera_path_length = 0.0;
  if ((!limit_motion || max_ee_path_ratio_ <= 0.0) && !require_elbow_up_) {
    return true;
  }

  // Same samples as the PTP collision check (straight line in joint space).
  // The elbow must end up and, if it starts up, stay up along the motion.
  const int steps = std::max(
    1, static_cast<int>(std::ceil(max_delta / std::max(joint_path_resolution_, 1e-3))));
  std::vector<double> positions(start.size());
  tf2::Vector3 first_camera;
  tf2::Vector3 previous_camera;
  bool start_elbow_up = true;
  for (int step = 0; step <= steps; ++step) {
    const double fraction = static_cast<double>(step) / static_cast<double>(steps);
    for (std::size_t index = 0; index < start.size(); ++index) {
      positions[index] = start[index] + fraction * (goal[index] - start[index]);
    }
    const auto points = computeArmPoints(base_pose, positions, camera_link, cancelled);
    if (!points) {
      return false;
    }
    if (require_elbow_up_) {
      const bool elbow_up = isElbowUp(*points);
      if (step == 0) {
        start_elbow_up = elbow_up;
      } else if (!elbow_up && (start_elbow_up || step == steps)) {
        ++diagnostics.elbow_down_rejections;
        return false;
      }
    }
    if (step == 0) {
      first_camera = points->camera;
    } else {
      camera_path_length += (points->camera - previous_camera).length();
    }
    previous_camera = points->camera;
  }

  if (limit_motion && max_ee_path_ratio_ > 0.0) {
    const double straight = (previous_camera - first_camera).length();
    if (camera_path_length > max_ee_path_ratio_ * straight + max_ee_path_slack_m_) {
      ++diagnostics.long_motion_rejections;
      return false;
    }
  }
  return true;
}

bool CameraPlacementPlanner::isJointPathValid(
  const geometry_msgs::msg::PoseStamped & base_pose,
  const std::vector<double> & start,
  const std::vector<double> & goal,
  const CancelCallback & cancelled)
{
  using namespace std::chrono_literals;
  // Pilz PTP interpolates all joints synchronously, so its geometric path is
  // the straight line between start and goal in joint space.
  double max_delta = 0.0;
  for (std::size_t index = 0; index < start.size(); ++index) {
    max_delta = std::max(max_delta, std::abs(goal[index] - start[index]));
  }
  const int steps = std::max(
    1, static_cast<int>(std::ceil(max_delta / std::max(joint_path_resolution_, 1e-3))));

  auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  request->group_name = planning_group_;
  request->robot_state = baseRobotState(base_pose);
  request->robot_state.joint_state.name = arm_joint_names_;
  request->robot_state.joint_state.position.resize(start.size());
  // The end points are already known to be valid (current state and the
  // validated IK solution), so only the interior samples are checked.
  for (int step = 1; step < steps; ++step) {
    const double fraction = static_cast<double>(step) / static_cast<double>(steps);
    for (std::size_t index = 0; index < start.size(); ++index) {
      request->robot_state.joint_state.position[index] =
        start[index] + fraction * (goal[index] - start[index]);
    }
    auto future = validity_client_->async_send_request(request);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(service_timeout_sec_);
    while (future.wait_for(50ms) != std::future_status::ready) {
      if ((cancelled && cancelled()) || std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
    }
    if (!future.get()->valid) {
      return false;
    }
  }
  return true;
}

bool CameraPlacementPlanner::solveIk(
  const geometry_msgs::msg::PoseStamped & base_pose,
  const geometry_msgs::msg::PoseStamped & tool0_pose,
  const std::optional<std::vector<double>> & seed,
  CandidateSolution & candidate,
  MoveItDiagnostics & diagnostics,
  const CancelCallback & cancelled)
{
  using namespace std::chrono_literals;
  auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  request->ik_request.group_name = planning_group_;
  request->ik_request.ik_link_name = tool0_link_;
  request->ik_request.pose_stamped = tool0_pose;
  request->ik_request.avoid_collisions = true;
  request->ik_request.timeout = rclcpp::Duration::from_seconds(ik_timeout_sec_);
  request->ik_request.robot_state = baseRobotState(base_pose);
  if (seed) {
    request->ik_request.robot_state.joint_state.name = arm_joint_names_;
    request->ik_request.robot_state.joint_state.position = *seed;
  }

  auto ik_future = ik_client_->async_send_request(request);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(service_timeout_sec_);
  while (ik_future.wait_for(50ms) != std::future_status::ready) {
    if ((cancelled && cancelled()) || std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
  }
  const auto ik_response = ik_future.get();
  if (ik_response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    ++diagnostics.collision_aware_error_codes[ik_response->error_code.val];
    if (!diagnose_ik_failures_) {
      return false;
    }

    // A collision-aware IK failure does not say whether the pose is unreachable
    // or every IK solution was rejected by collision checking. Retry only for
    // diagnostics, without changing the acceptance result of the normal search.
    auto diagnostic_request =
      std::make_shared<moveit_msgs::srv::GetPositionIK::Request>(*request);
    diagnostic_request->ik_request.avoid_collisions = false;
    auto diagnostic_future = ik_client_->async_send_request(diagnostic_request);
    const auto diagnostic_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(service_timeout_sec_);
    while (diagnostic_future.wait_for(50ms) != std::future_status::ready) {
      if ((cancelled && cancelled()) ||
        std::chrono::steady_clock::now() >= diagnostic_deadline)
      {
        ++diagnostics.collision_disabled_ik_failures;
        return false;
      }
    }

    const auto diagnostic_response = diagnostic_future.get();
    if (diagnostic_response->error_code.val !=
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      ++diagnostics.collision_disabled_ik_failures;
      ++diagnostics.collision_disabled_error_codes[
        diagnostic_response->error_code.val];
      return false;
    }

    ++diagnostics.collision_disabled_ik_successes;
    auto diagnostic_validity_request =
      std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
    diagnostic_validity_request->robot_state = diagnostic_response->solution;
    diagnostic_validity_request->group_name = planning_group_;
    auto diagnostic_validity_future =
      validity_client_->async_send_request(diagnostic_validity_request);
    const auto validity_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(service_timeout_sec_);
    while (diagnostic_validity_future.wait_for(50ms) != std::future_status::ready) {
      if ((cancelled && cancelled()) ||
        std::chrono::steady_clock::now() >= validity_deadline)
      {
        return false;
      }
    }

    const auto diagnostic_validity_response = diagnostic_validity_future.get();
    if (diagnostic_validity_response->valid) {
      ++diagnostics.valid_collision_disabled_solutions;
    } else {
      ++diagnostics.collision_rejections;
      for (const auto & contact : diagnostic_validity_response->contacts) {
        diagnostics.collision_pairs.insert(
          contact.contact_body_1 + "<->" + contact.contact_body_2);
      }
    }
    return false;
  }

  auto validity_request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  validity_request->robot_state = ik_response->solution;
  validity_request->group_name = planning_group_;
  auto validity_future = validity_client_->async_send_request(validity_request);
  while (validity_future.wait_for(50ms) != std::future_status::ready) {
    if ((cancelled && cancelled()) || std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
  }
  const auto validity_response = validity_future.get();
  if (!validity_response->valid) {
    ++diagnostics.collision_rejections;
    for (const auto & contact : validity_response->contacts) {
      diagnostics.collision_pairs.insert(
        contact.contact_body_1 + "<->" + contact.contact_body_2);
    }
    return false;
  }

  candidate.arm_solution.header = ik_response->solution.joint_state.header;
  double cost = 0.0;
  for (const auto & desired_name : arm_joint_names_) {
    const auto found = std::find(
      ik_response->solution.joint_state.name.begin(),
      ik_response->solution.joint_state.name.end(), desired_name);
    if (found == ik_response->solution.joint_state.name.end()) {
      return false;
    }
    const auto position_index = static_cast<std::size_t>(
      std::distance(ik_response->solution.joint_state.name.begin(), found));
    if (position_index >= ik_response->solution.joint_state.position.size()) {
      return false;
    }
    const double position = ik_response->solution.joint_state.position[position_index];
    candidate.arm_solution.name.push_back(desired_name);
    candidate.arm_solution.position.push_back(position);
    cost += std::abs(position);
  }
  candidate.arm_cost = cost / static_cast<double>(arm_joint_names_.size());
  return true;
}

bool CameraPlacementPlanner::evaluateWithNav2(
  CandidateSolution & candidate,
  const CancelCallback & cancelled)
{
  using namespace std::chrono_literals;
  if (!navigation_client_->wait_for_action_server(
      std::chrono::duration<double>(service_timeout_sec_)))
  {
    throw std::runtime_error("Nav2 ComputePathToPose action is unavailable");
  }

  nav2_msgs::action::ComputePathToPose::Goal goal;
  goal.goal = candidate.base_pose;
  goal.use_start = false;
  auto goal_future = navigation_client_->async_send_goal(goal);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(service_timeout_sec_);
  while (goal_future.wait_for(50ms) != std::future_status::ready) {
    if ((cancelled && cancelled()) || std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
  }

  const auto goal_handle = goal_future.get();
  if (!goal_handle) {
    return false;
  }
  auto result_future = navigation_client_->async_get_result(goal_handle);
  while (result_future.wait_for(50ms) != std::future_status::ready) {
    if (cancelled && cancelled()) {
      navigation_client_->async_cancel_goal(goal_handle);
      return false;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      navigation_client_->async_cancel_goal(goal_handle);
      return false;
    }
  }

  const auto wrapped_result = result_future.get();
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED ||
    !wrapped_result.result || wrapped_result.result->path.poses.empty())
  {
    return false;
  }
  candidate.navigation_cost = pathLength(wrapped_result.result->path);
  return true;
}

double CameraPlacementPlanner::calculateScore(const CandidateSolution & candidate) const
{
  return candidate.navigation_cost + 0.25 * candidate.arm_cost;
}

void CameraPlacementPlanner::publishFeedback(
  const FeedbackCallback & callback,
  const std::string & phase,
  const std::uint32_t evaluated,
  const std::uint32_t valid) const
{
  if (callback) {
    callback(PlanningFeedback{phase, evaluated, valid});
  }
}



/** @brief ROS action server with its private camera-placement implementation. */
class CameraPlacementActionServer : public rclcpp::Node
{
public:
  /** @brief Creates the planning clients and starts `/camera_placement`. */
  CameraPlacementActionServer()
  : rclcpp::Node("camera_placement_action_server")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    planner_ = std::make_unique<CameraPlacementPlanner>(*this, tf_buffer_);

    action_server_ = rclcpp_action::create_server<Action>(
      this,
      "camera_placement",
      std::bind(
        &CameraPlacementActionServer::handleGoal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &CameraPlacementActionServer::handleCancel, this,
        std::placeholders::_1),
      std::bind(
        &CameraPlacementActionServer::handleAccepted, this,
        std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Camera placement action ready on '/camera_placement'");
  }

private:
  using Action = renee_action_servers::action::CameraPlacement;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Action::Goal> goal)
  {
    bool expected = false;
    if (!active_goal_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Rejecting camera placement: another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(
      get_logger(),
      "Accepted camera placement: frame='%s' camera_link='%s' lock_current_base=%s "
      "allow_long_arm_motion=%s",
      goal->camera_pose.header.frame_id.c_str(), goal->camera_link.c_str(),
      goal->lock_current_base ? "true" : "false",
      goal->allow_long_arm_motion ? "true" : "false");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "Camera placement cancellation requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread([this, goal_handle]() {execute(goal_handle);}).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    PlanningRequest request;
    request.camera_pose = goal->camera_pose;
    request.camera_link = planner_->resolveCameraLink(goal->camera_link);
    request.lock_current_base = goal->lock_current_base;
    request.allow_long_arm_motion = goal->allow_long_arm_motion;

    RCLCPP_INFO(
      get_logger(), "Planning camera pose: position=(%.3f, %.3f, %.3f) orientation=(%.4f, %.4f, %.4f, %.4f)",
      request.camera_pose.pose.position.x, request.camera_pose.pose.position.y,
      request.camera_pose.pose.position.z, request.camera_pose.pose.orientation.x,
      request.camera_pose.pose.orientation.y, request.camera_pose.pose.orientation.z,
      request.camera_pose.pose.orientation.w);

    const auto solution = planner_->plan(
      request,
      [this, goal_handle](const PlanningFeedback & update) {
        auto feedback = std::make_shared<Action::Feedback>();
        feedback->phase = update.phase;
        feedback->candidates_evaluated = update.candidates_evaluated;
        feedback->valid_candidates = update.valid_candidates;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(
          get_logger(), "Camera placement phase='%s' evaluated=%u valid=%u",
          update.phase.c_str(), update.candidates_evaluated, update.valid_candidates);
      },
      [goal_handle]() {return goal_handle->is_canceling();});

    auto result = std::make_shared<Action::Result>();
    result->success = solution.success;
    result->message = solution.message;
    result->resolved_camera_link = request.camera_link;
    result->base_pose = solution.base_pose;
    result->end_effector_pose = solution.end_effector_pose;
    result->arm_solution = solution.arm_solution;
    result->score = solution.score;

    if (goal_handle->is_canceling()) {
      RCLCPP_WARN(get_logger(), "Camera placement cancelled: %s", solution.message.c_str());
      goal_handle->canceled(result);
    } else if (solution.success) {
      RCLCPP_INFO(
        get_logger(),
        "Camera placement succeeded: base=(%.3f, %.3f) tool0=(%.3f, %.3f, %.3f) score=%.3f",
        solution.base_pose.pose.position.x, solution.base_pose.pose.position.y,
        solution.end_effector_pose.pose.position.x, solution.end_effector_pose.pose.position.y,
        solution.end_effector_pose.pose.position.z, solution.score);
      goal_handle->succeed(result);
    } else {
      RCLCPP_ERROR(get_logger(), "Camera placement failed: %s", solution.message.c_str());
      goal_handle->abort(result);
    }
    active_goal_.store(false);
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<CameraPlacementPlanner> planner_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
  std::atomic<bool> active_goal_{false};
};

}  // namespace renee_action_servers

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<renee_action_servers::CameraPlacementActionServer>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
