/**
 * @file capture_rgbd_action_server.cpp
 * @brief Captures synchronized RGB-D frames and persists their pose metadata.
 */

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "renee_action_servers/action/capture_rgbd.hpp"
#include "renee_perception/rgbd_dataset_writer.hpp"

namespace
{

geometry_msgs::msg::PoseStamped transformToPoseStamped(
  const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

bool hasNonZeroStamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec != 0 || stamp.nanosec != 0;
}

double stampDeltaMs(
  const builtin_interfaces::msg::Time & lhs,
  const builtin_interfaces::msg::Time & rhs)
{
  return std::abs((rclcpp::Time(lhs) - rclcpp::Time(rhs)).nanoseconds()) / 1.0e6;
}

}  // namespace

/** @brief Implements the `/capture_rgbd` action server. */
class CaptureRGBDActionServer : public rclcpp::Node
{
public:
  using Action = renee_action_servers::action::CaptureRGBD;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  /** @brief Creates camera subscriptions, TF clients, and the capture action. */
  CaptureRGBDActionServer()
  : rclcpp::Node("capture_rgbd_action_server")
  {
    rgb_topic_ = declare_parameter<std::string>(
      "rgb_topic", "/robot/arm_rgbd_camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/robot/arm_rgbd_camera/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/robot/arm_rgbd_camera/color/camera_info");
    default_session_dir_ = declare_parameter<std::string>(
      "session_dir", "/tmp/renee_scan_session");
    world_frame_ = declare_parameter<std::string>("world_frame", "robot_map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "robot_base_link");
    camera_frame_ = declare_parameter<std::string>(
      "camera_frame", "robot_arm_rgbd_camera_left_camera_optical_frame");
    sync_tolerance_ms_ = declare_parameter<double>("sync_tolerance_ms", 1000.0);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.5);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      rgb_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CaptureRGBDActionServer::onRgbImage, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CaptureRGBDActionServer::onDepthImage, this, std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CaptureRGBDActionServer::onCameraInfo, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<Action>(
      this,
      "capture_rgbd",
      std::bind(
        &CaptureRGBDActionServer::handleGoal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&CaptureRGBDActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&CaptureRGBDActionServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "CaptureRGBD action ready on '/capture_rgbd'");
    RCLCPP_INFO(get_logger(), "RGB topic: %s", rgb_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Depth topic: %s", depth_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Camera frame: %s", camera_frame_.c_str());
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Action::Goal> goal)
  {
    if (goal->waypoint_id.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting RGB-D capture without waypoint_id");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_goal_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Rejecting RGB-D capture: another goal is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{[this, goal_handle]() {execute(goal_handle);}}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result = std::make_shared<Action::Result>();
    const auto goal = goal_handle->get_goal();

    const auto finish = [this]() {active_goal_ = false;};
    const auto fail = [this, &finish, &goal_handle, &result](const std::string & message) {
        result->success = false;
        result->message = message;
        RCLCPP_ERROR(get_logger(), "CaptureRGBD failed: %s", message.c_str());
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
        } else {
          goal_handle->abort(result);
        }
        finish();
      };
    const auto feedback = [&goal_handle](const std::string & phase) {
        auto update = std::make_shared<Action::Feedback>();
        update->phase = phase;
        goal_handle->publish_feedback(update);
      };

    feedback("reading_camera_data");
    sensor_msgs::msg::Image::ConstSharedPtr rgb;
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;
    {
      std::lock_guard<std::mutex> lock(latest_data_mutex_);
      rgb = latest_rgb_;
      depth = latest_depth_;
      camera_info = latest_camera_info_;
    }

    if (!rgb || !depth || !camera_info) {
      fail("Missing RGB, depth, or camera_info data");
      return;
    }
    if (hasNonZeroStamp(rgb->header.stamp) && hasNonZeroStamp(depth->header.stamp)) {
      const double delta_ms = stampDeltaMs(rgb->header.stamp, depth->header.stamp);
      if (delta_ms > sync_tolerance_ms_) {
        fail("RGB/depth timestamps differ by " + std::to_string(delta_ms) + " ms");
        return;
      }
    }
    if (goal_handle->is_canceling()) {
      fail("RGB-D capture cancelled");
      return;
    }

    try {
      feedback("resolving_transforms");
      const auto robot_transform = tf_buffer_->lookupTransform(
        world_frame_, robot_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
      const auto camera_transform = tf_buffer_->lookupTransform(
        world_frame_, camera_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));

      if (goal_handle->is_canceling()) {
        fail("RGB-D capture cancelled");
        return;
      }

      feedback("writing_capture");
      renee_perception::RgbdCaptureData data;
      data.waypoint_id = goal->waypoint_id;
      data.session_dir = goal->session_dir.empty() ? default_session_dir_ : goal->session_dir;
      data.rgb_image = *rgb;
      data.depth_image = *depth;
      data.camera_info = *camera_info;
      data.robot_pose = transformToPoseStamped(robot_transform);
      data.camera_pose = transformToPoseStamped(camera_transform);
      data.robot_pose.header.stamp = rgb->header.stamp;
      data.camera_pose.header.stamp = rgb->header.stamp;

      const auto record = dataset_writer_.writeCapture(data);
      result->success = true;
      result->message = "Captured RGB-D frame";
      result->timestamp = record.timestamp;
      result->rgb_path = record.rgb_path;
      result->depth_path = record.depth_path;
      result->metadata_path = record.metadata_path;
      result->robot_pose = record.robot_pose;
      result->camera_pose = record.camera_pose;
      goal_handle->succeed(result);
      RCLCPP_INFO(
        get_logger(), "CaptureRGBD succeeded: rgb='%s' depth='%s'",
        result->rgb_path.c_str(), result->depth_path.c_str());
      finish();
    } catch (const std::exception & exception) {
      fail(exception.what());
    }
  }

  void onRgbImage(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(latest_data_mutex_);
    latest_rgb_ = std::move(msg);
  }

  void onDepthImage(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(latest_data_mutex_);
    latest_depth_ = std::move(msg);
  }

  void onCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(latest_data_mutex_);
    latest_camera_info_ = std::move(msg);
  }

  std::string rgb_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string default_session_dir_;
  std::string world_frame_;
  std::string robot_frame_;
  std::string camera_frame_;
  double sync_tolerance_ms_{1000.0};
  double tf_timeout_sec_{0.5};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  renee_perception::RgbdDatasetWriter dataset_writer_;

  std::mutex latest_data_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_rgb_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
  std::atomic<bool> active_goal_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaptureRGBDActionServer>());
  rclcpp::shutdown();
  return 0;
}
