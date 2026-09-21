/** @file capture_rgbd_action_server.cpp
 *  @brief Captures ROS-managed RGB-D stations with robot and calibration metadata.
 */

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <realsense2_camera_msgs/msg/extrinsics.hpp>
#include <realsense2_camera_msgs/msg/metadata.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <future>
#include <iomanip>
#include <iterator>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "renee_action_servers/action/capture_rgbd.hpp"
#include "renee_perception/rgbd_dataset_writer.hpp"

namespace
{
  using namespace std::chrono_literals;

  constexpr std::size_t kMaxBufferedMessages = 256;
  constexpr double kOdomToleranceMs = 250.0;
  constexpr double kAmclToleranceMs = 1000.0;

  geometry_msgs::msg::PoseStamped transformToPose(
    const geometry_msgs::msg::TransformStamped & transform,
    const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = transform.header;
    pose.header.stamp = stamp;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    return pose;
  }

  double deltaMs(
    const builtin_interfaces::msg::Time & lhs,
    const builtin_interfaces::msg::Time & rhs)
  {
    return std::abs((rclcpp::Time(lhs) - rclcpp::Time(rhs)).nanoseconds()) / 1.0e6;
  }

  // Streams are ordered by their header timestamp. Check the two neighbouring
  // messages around stamp instead of scanning the entire history for each RGB
  // frame.
  template<typename MessageT>
  typename MessageT::ConstSharedPtr closest(
    const std::deque<typename MessageT::ConstSharedPtr> & messages,
    const builtin_interfaces::msg::Time & stamp,
    double tolerance_ms)
  {
    const auto target = rclcpp::Time(stamp);
    const auto next = std::lower_bound(
      messages.begin(), messages.end(), target,
      [](const auto & message, const rclcpp::Time & value) {
        return rclcpp::Time(message->header.stamp) < value;
      });

    typename MessageT::ConstSharedPtr result;
    double best = tolerance_ms;
    const auto consider = [&result, &best, &stamp](const auto & message) {
        const double distance = deltaMs(message->header.stamp, stamp);
        if (distance <= best) {
          best = distance;
          result = message;
        }
      };
    if (next != messages.end()) {
      consider(*next);
    }
    if (next != messages.begin()) {
      consider(*std::prev(next));
    }
    return result;
  }

  template<typename MessageT>
  void discardOlderThan(
    std::deque<typename MessageT::ConstSharedPtr> & messages,
    const builtin_interfaces::msg::Time & stamp,
    double tolerance_ms)
  {
    const auto oldest_valid = rclcpp::Time(stamp) -
      rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(tolerance_ms * 1.0e6));
    while (!messages.empty() && rclcpp::Time(messages.front()->header.stamp) < oldest_valid) {
      messages.pop_front();
    }
  }

  std::string extrinsicsJson(const realsense2_camera_msgs::msg::Extrinsics & extrinsics)
  {
    std::ostringstream out;
    out << std::setprecision(12) << "{\"rotation_column_major\":[";
    for (std::size_t i = 0; i < extrinsics.rotation.size(); ++i) {
      if (i) {out << ',';} out << extrinsics.rotation[i];
    }
    out << "],\"translation_m\":[";
    for (std::size_t i = 0; i < extrinsics.translation.size(); ++i) {
      if (i) {out << ',';} out << extrinsics.translation[i];
    }
    out << "]}";
    return out.str();
  }

  double linearSpeed(const nav_msgs::msg::Odometry & odom)
  {
    const auto & value = odom.twist.twist.linear;
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
  }

  double angularSpeed(const nav_msgs::msg::Odometry & odom)
  {
    const auto & value = odom.twist.twist.angular;
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
  }
}  // namespace

/**
 * Collects a short, time-synchronized RGB-D station and persists it through
 * RgbdDatasetWriter. A station is valid only when the required camera and rover
 * data were available and the rover remained still while it was collected.
 */
class CaptureRGBDActionServer : public rclcpp::Node
{
  public:
    using Action = renee_action_servers::action::CaptureRGBD;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  struct CaptureRequest
  {
    std::string waypoint_id;
    std::string session_dir;
    std::uint32_t target_frames;
  };

  CaptureRGBDActionServer()
  : Node("capture_rgbd_action_server")
    {
      // Mode changes both the required inputs and whether this node manages
      // RealSense controls. Topic names remain configurable for each deployment.
      mode_ = declare_parameter<std::string>("capture_mode", "simulation");
      simulated_ = mode_ == "simulation";
      camera_model_ = declare_parameter<std::string>("camera_model", "realsense_d435i");
      if (camera_model_ != "realsense_d435i" && camera_model_ != "stereolabs_zed2i") {
        throw std::invalid_argument("Unsupported camera_model: " + camera_model_);
      }
      if (!simulated_ && camera_model_ == "stereolabs_zed2i") {
        throw std::invalid_argument("stereolabs_zed2i is currently supported only in simulation");
      }
      zed2_simulated_ = simulated_ && camera_model_ == "stereolabs_zed2i";
      if (zed2_simulated_) {depth_units_m_ = 1.0;}
      default_frame_count_ = declare_parameter<int>("default_frame_count", 30);
      default_session_dir_ = declare_parameter<std::string>("session_dir", "/tmp/renee_scan_session");
      sync_tolerance_ms_ = declare_parameter<double>("sync_tolerance_ms", simulated_ ? 50.0 : 5.0);
      capture_timeout_sec_ = declare_parameter<double>("capture_timeout_sec", 15.0);
      tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.5);
      max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.01);
      max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.0175);
      map_frame_ = declare_parameter<std::string>("map_frame", "robot_map");
      odom_frame_ = declare_parameter<std::string>("odom_frame", "robot_odom");
      base_frame_ = declare_parameter<std::string>("base_frame", "robot_base_link");
      camera_frame_ = declare_parameter<std::string>(
        "camera_frame", zed2_simulated_ ?
        "robot_arm_rgbd_camera_left_camera_optical_frame" :
        "robot_arm_rgbd_camera_color_optical_frame");
      camera_node_ = declare_parameter<std::string>(
        "camera_node", "/robot/arm_rgbd_camera");
      manage_camera_controls_ = declare_parameter<bool>("manage_camera_controls", !simulated_);
      camera_warmup_sec_ = declare_parameter<double>("camera_warmup_sec", 2.0);

      const auto topic = [this](const std::string & name, const std::string & fallback) {
          return declare_parameter<std::string>(name, fallback);
        };
      const std::string rgb = topic("rgb_topic", "/robot/arm_rgbd_camera/color/image_raw");
      const std::string depth = topic("depth_topic", "/robot/arm_rgbd_camera/depth/image_rect_raw");
      const std::string aligned = topic(
        "aligned_depth_topic", "/robot/arm_rgbd_camera/aligned_depth_to_color/image_raw");
      const std::string stereo_right = topic(
        "stereo_right_topic", "/robot/arm_rgbd_camera/right/image_raw");
      const std::string ir1 = topic("infrared_left_topic", "/robot/arm_rgbd_camera/infra1/image_rect_raw");
      const std::string ir2 = topic("infrared_right_topic", "/robot/arm_rgbd_camera/infra2/image_rect_raw");
      const std::string color_info = topic("color_info_topic", "/robot/arm_rgbd_camera/color/camera_info");
      const std::string depth_info = topic("depth_info_topic", "/robot/arm_rgbd_camera/depth/camera_info");
      const std::string stereo_right_info = topic(
        "stereo_right_info_topic", "/robot/arm_rgbd_camera/right/camera_info");
      const std::string ir1_info = topic("infrared_left_info_topic", "/robot/arm_rgbd_camera/infra1/camera_info");
      const std::string ir2_info = topic("infrared_right_info_topic", "/robot/arm_rgbd_camera/infra2/camera_info");
      const std::string camera_imu = topic("camera_imu_topic", "/robot/arm_rgbd_camera/imu");
      const std::string camera_accel = topic(
        "camera_accel_topic", "/robot/arm_rgbd_camera/accel/sample");
      const std::string camera_gyro = topic(
        "camera_gyro_topic", "/robot/arm_rgbd_camera/gyro/sample");

      // Keep the streams needed to describe a keyframe. In real mode, metadata
      // and extrinsics are also required so the capture can be reproduced.
      rgb_sub_ = imageSubscription(rgb, rgb_buffer_);
      depth_sub_ = imageSubscription(depth, depth_buffer_);
      aligned_sub_ = imageSubscription(aligned, aligned_buffer_);
      stereo_right_sub_ = imageSubscription(stereo_right, stereo_right_buffer_);
      ir1_sub_ = imageSubscription(ir1, ir1_buffer_);
      ir2_sub_ = imageSubscription(ir2, ir2_buffer_);
      color_info_sub_ = infoSubscription(color_info, color_info_);
      depth_info_sub_ = infoSubscription(depth_info, depth_info_);
      stereo_right_info_sub_ = infoSubscription(stereo_right_info, stereo_right_info_);
      ir1_info_sub_ = infoSubscription(ir1_info, ir1_info_);
      ir2_info_sub_ = infoSubscription(ir2_info, ir2_info_);

      color_metadata_sub_ = metadataSubscription(
        topic("color_metadata_topic", "/robot/arm_rgbd_camera/color/metadata"), color_metadata_);
      depth_metadata_sub_ = metadataSubscription(
        topic("depth_metadata_topic", "/robot/arm_rgbd_camera/depth/metadata"), depth_metadata_);
      ir1_metadata_sub_ = metadataSubscription(
        topic("infrared_left_metadata_topic", "/robot/arm_rgbd_camera/infra1/metadata"), ir1_metadata_);
      ir2_metadata_sub_ = metadataSubscription(
        topic("infrared_right_metadata_topic", "/robot/arm_rgbd_camera/infra2/metadata"), ir2_metadata_);

      const auto qos = rclcpp::SensorDataQoS();
      if (zed2_simulated_) {
        camera_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          camera_imu, qos,
          [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {push(imu_buffer_, std::move(msg));});
      } else {
        accel_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          camera_accel, qos,
          [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {push(imu_buffer_, std::move(msg));});
        gyro_sub_ = create_subscription<sensor_msgs::msg::Imu>(
          camera_gyro, qos,
          [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {push(imu_buffer_, std::move(msg));});
      }
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        topic("wheel_odom_topic", "/robot/robotnik_base_control/odom"), qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {push(odom_buffer_, std::move(msg));});
      amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        topic("amcl_topic", "/robot/amcl_pose"), qos,
        [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
          push(amcl_buffer_, std::move(msg));
        });
      rover_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        topic("rover_imu_topic", "/robot/imu/data"), qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {push(rover_imu_buffer_, std::move(msg));});

      auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
      extrinsics_sub_ = create_subscription<realsense2_camera_msgs::msg::Extrinsics>(
        topic("depth_to_color_extrinsics_topic", "/robot/arm_rgbd_camera/extrinsics/depth_to_color"),
        latched_qos, [this](realsense2_camera_msgs::msg::Extrinsics::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_); extrinsics_ = std::move(msg);
        });

      // TF is queried at the image timestamp, rather than at "now", so poses
      // describe the same instant as the saved camera data.
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      action_server_ = rclcpp_action::create_server<Action>(
        this, "capture_rgbd",
        std::bind(&CaptureRGBDActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CaptureRGBDActionServer::handleCancel, this, std::placeholders::_1),
        std::bind(&CaptureRGBDActionServer::handleAccepted, this, std::placeholders::_1));
      RCLCPP_INFO(
        get_logger(), "CaptureRGBD ready in %s mode with %s",
        mode_.c_str(), camera_model_.c_str());
    }

  private:
    CaptureRequest requestFromGoal(const Action::Goal & goal) const
    {
      return {
        goal.waypoint_id,
        goal.session_dir.empty() ? default_session_dir_ : goal.session_dir,
        goal.frame_count == 0 ?
        static_cast<std::uint32_t>(std::max(1, default_frame_count_)) : goal.frame_count,
      };
    }

    template<typename SharedPtrT>
    void push(
      std::deque<SharedPtrT> & buffer,
      SharedPtrT message)
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      buffer.push_back(std::move(message));
      // Retain enough recent data for synchronization without allowing an idle
      // node to consume unbounded memory.
      while (buffer.size() > kMaxBufferedMessages) {buffer.pop_front();}
      data_cv_.notify_all();
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription(
      const std::string & topic, std::deque<sensor_msgs::msg::Image::ConstSharedPtr> & buffer)
    {
      return create_subscription<sensor_msgs::msg::Image>(
        topic, rclcpp::SensorDataQoS(),
        [this, &buffer](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          push(buffer, std::move(msg));
        });
    }

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr infoSubscription(
      const std::string & topic, sensor_msgs::msg::CameraInfo::ConstSharedPtr & destination)
    {
      return create_subscription<sensor_msgs::msg::CameraInfo>(
        topic, rclcpp::SensorDataQoS(),
        [this, &destination](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(data_mutex_); destination = std::move(msg);
        });
    }

    rclcpp::Subscription<realsense2_camera_msgs::msg::Metadata>::SharedPtr metadataSubscription(
      const std::string & topic,
      std::deque<realsense2_camera_msgs::msg::Metadata::ConstSharedPtr> & destination)
    {
      return create_subscription<realsense2_camera_msgs::msg::Metadata>(
        topic, rclcpp::SensorDataQoS(),
        [this, &destination](realsense2_camera_msgs::msg::Metadata::ConstSharedPtr msg) {
          push(destination, std::move(msg));
        });
    }

    template<typename MessageT>
    std::vector<MessageT> drainUntil(
      std::deque<typename MessageT::ConstSharedPtr> & buffer,
      const builtin_interfaces::msg::Time & stamp)
    {
      // Samples are written once, with the first keyframe whose timestamp is at
      // or after them. This preserves their temporal order across a station.
      std::vector<MessageT> output;
      while (!buffer.empty() && rclcpp::Time(buffer.front()->header.stamp) <= rclcpp::Time(stamp)) {
        output.push_back(*buffer.front());
        buffer.pop_front();
      }
      return output;
    }

    rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal)
    {
      // A single capture owns the shared stream buffers at a time.
      if (goal->waypoint_id.empty() || active_goal_.exchange(true)) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
    {
      data_cv_.notify_all();
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandle> goal)
    {
      std::thread([this, goal]() {execute(goal);}).detach();
    }

    std::string prepareCameraControls()
    {
      std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: enter (simulated_=%d, manage_camera_controls_=%d)\n",
        static_cast<int>(simulated_), static_cast<int>(manage_camera_controls_));
      std::fflush(stderr);
      if (simulated_) {
        return "{\"hardware_controls\":\"not_available_in_simulation\",\"depth_units\":null}";
      }
      if (!manage_camera_controls_) {return "{\"managed_by_action\":false}";}
      if (!camera_parameter_client_) {
        std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: constructing AsyncParametersClient for '%s' via shared_from_this()\n",
          camera_node_.c_str());
        std::fflush(stderr);
        camera_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
          shared_from_this(), camera_node_);
        std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: AsyncParametersClient constructed ok\n");
        std::fflush(stderr);
      }
      const auto & client = camera_parameter_client_;
      std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: wait_for_service(2s) on '%s'\n", camera_node_.c_str());
      std::fflush(stderr);
      if (!client->wait_for_service(2s)) {throw std::runtime_error("RealSense parameter service unavailable");}
      std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: service is up, camera_controls_frozen_=%d\n",
        static_cast<int>(camera_controls_frozen_));
      std::fflush(stderr);
      // Let auto exposure settle once, record the resulting settings, then lock
      // exposure for the rest of the process to keep a station consistent.
      if (!camera_controls_frozen_) {
        std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: sending initial set_parameters (enable auto-exposure)\n");
        std::fflush(stderr);
        auto configure_future = client->set_parameters({
          rclcpp::Parameter("depth_module.visual_preset", 3),
          rclcpp::Parameter("depth_module.emitter_enabled", 1),
          rclcpp::Parameter("rgb_camera.enable_auto_exposure", true),
          rclcpp::Parameter("depth_module.enable_auto_exposure", true)});
        if (configure_future.wait_for(3s) != std::future_status::ready) {
          throw std::runtime_error("Timed out configuring RealSense capture controls");
        }
        std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: initial set_parameters future ready, calling .get()\n");
        std::fflush(stderr);
        const auto configure_results = configure_future.get();
        std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: initial set_parameters returned %zu results\n",
          configure_results.size());
        std::fflush(stderr);
        for (const auto & result : configure_results) {
          if (!result.successful) {
            throw std::runtime_error("Failed to configure RealSense: " + result.reason);
          }
        }
        std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: sleeping camera_warmup_sec_=%.3f s\n", camera_warmup_sec_);
        std::fflush(stderr);
        std::this_thread::sleep_for(std::chrono::duration<double>(camera_warmup_sec_));
        std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: warmup sleep done\n");
        std::fflush(stderr);
      }
      const std::vector<std::string> names = {
        "rgb_camera.exposure", "rgb_camera.gain", "depth_module.exposure",
        "depth_module.gain", "depth_module.depth_units", "depth_module.visual_preset",
        "depth_module.emitter_enabled", "depth_module.laser_power"};
      std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: calling get_parameters() for %zu names\n", names.size());
      std::fflush(stderr);
      auto future = client->get_parameters(names);
      if (future.wait_for(3s) != std::future_status::ready) {
        throw std::runtime_error("Timed out reading RealSense controls");
      }
      std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: get_parameters future ready, calling .get()\n");
      std::fflush(stderr);
      const auto values = future.get();
      std::fprintf(
        stderr,
        "[capture_rgbd][TRACE] prepareCameraControls: get_parameters returned %zu values (requested %zu names)%s\n",
        values.size(), names.size(), values.size() != names.size() ? " <-- SIZE MISMATCH" : "");
      std::fflush(stderr);
      if (values.size() > 4 && values[4].get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        depth_units_m_ = values[4].as_double();
      }
      if (!camera_controls_frozen_) {
        std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: freezing exposure (disable auto-exposure)\n");
        std::fflush(stderr);
        auto set_future = client->set_parameters({
          rclcpp::Parameter("rgb_camera.enable_auto_exposure", false),
          rclcpp::Parameter("depth_module.enable_auto_exposure", false)});
        if (set_future.wait_for(3s) != std::future_status::ready) {
          throw std::runtime_error("Timed out freezing RealSense exposure");
        }
        std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: freeze set_parameters future ready, calling .get()\n");
        std::fflush(stderr);
        const auto freeze_results = set_future.get();
        std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: freeze set_parameters returned %zu results\n",
          freeze_results.size());
        std::fflush(stderr);
        for (const auto & result : freeze_results) {
          if (!result.successful) {throw std::runtime_error("Failed to freeze RealSense exposure: " + result.reason);}
        }
        camera_controls_frozen_ = true;
      }
      std::fprintf(
        stderr,
        "[capture_rgbd][TRACE] prepareCameraControls: building settings JSON from %zu values (loop bound=names.size()=%zu)\n",
        values.size(), names.size());
      std::fflush(stderr);
      std::ostringstream out;
      out << "{\"managed_by_action\":true";
      for (std::size_t i = 0; i < names.size(); ++i) {
        std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: reading values[%zu] ('%s')\n", i, names[i].c_str());
        std::fflush(stderr);
        out << ",\"" << names[i] << "\":";
        // The RealSense parameter service can return fewer entries than requested
        // (e.g. a name it does not declare). Treat a missing index the same as
        // PARAMETER_NOT_SET instead of indexing past the end of `values`.
        if (i >= values.size() || values[i].get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
          out << "null";
        } else if (values[i].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          out << '"' << values[i].as_string() << '"';
        } else {
          out << values[i].value_to_string();
        }
      }
      out << '}';
      std::fprintf(stderr, "[capture_rgbd][TRACE] prepareCameraControls: exit ok\n");
      std::fflush(stderr);
      return out.str();
    }

    void publishFeedback(const std::shared_ptr<GoalHandle> & goal, const std::string & phase,
      std::uint32_t captured, std::uint32_t target)
    {
      auto feedback = std::make_shared<Action::Feedback>();
      feedback->phase = phase;
      feedback->captured_frames = captured;
      feedback->target_frames = target;
      goal->publish_feedback(feedback);
    }

    bool validateRoverState(
      const renee_perception::RgbdCaptureData & data,
      const std::string & waypoint_id,
      std::string & message) const
    {
      if (!data.has_wheel_odometry || !data.has_rover_imu) {
        message = "Required Vogui data was unavailable";
        return false;
      }
      if (!simulated_ && !data.has_amcl_pose) {
        RCLCPP_WARN(
          get_logger(),
          "amcl_pose unavailable for this keyframe (%s) — recorded as null, capture still valid",
          waypoint_id.c_str());
      }
      if (linearSpeed(data.wheel_odometry) > max_linear_speed_ ||
        angularSpeed(data.wheel_odometry) > max_angular_speed_)
      {
        message = "Rover moved during station capture";
        return false;
      }
      return true;
    }

    void resolveTransforms(
      renee_perception::RgbdCaptureData & data,
      const builtin_interfaces::msg::Time & stamp)
    {
      const auto time_point = tf2_ros::fromMsg(stamp);
      std::fprintf(stderr, "[capture_rgbd][TRACE] resolveTransforms: lookup %s->%s\n",
        map_frame_.c_str(), odom_frame_.c_str());
      std::fflush(stderr);
      data.map_to_odom = tf_buffer_->lookupTransform(
        map_frame_, odom_frame_, time_point, tf2::durationFromSec(tf_timeout_sec_));
      std::fprintf(stderr, "[capture_rgbd][TRACE] resolveTransforms: lookup %s->%s\n",
        odom_frame_.c_str(), base_frame_.c_str());
      std::fflush(stderr);
      data.odom_to_base = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, time_point, tf2::durationFromSec(tf_timeout_sec_));
      std::fprintf(stderr, "[capture_rgbd][TRACE] resolveTransforms: lookup %s->%s\n",
        base_frame_.c_str(), camera_frame_.c_str());
      std::fflush(stderr);
      data.base_to_camera = tf_buffer_->lookupTransform(
        base_frame_, camera_frame_, time_point, tf2::durationFromSec(tf_timeout_sec_));
      std::fprintf(stderr, "[capture_rgbd][TRACE] resolveTransforms: lookup %s->%s (world_robot)\n",
        map_frame_.c_str(), base_frame_.c_str());
      std::fflush(stderr);
      const auto world_robot = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, time_point, tf2::durationFromSec(tf_timeout_sec_));
      std::fprintf(stderr, "[capture_rgbd][TRACE] resolveTransforms: lookup %s->%s (world_camera)\n",
        map_frame_.c_str(), camera_frame_.c_str());
      std::fflush(stderr);
      const auto world_camera = tf_buffer_->lookupTransform(
        map_frame_, camera_frame_, time_point, tf2::durationFromSec(tf_timeout_sec_));
      data.robot_pose = transformToPose(world_robot, stamp);
      data.camera_pose = transformToPose(world_camera, stamp);
      std::fprintf(stderr, "[capture_rgbd][TRACE] resolveTransforms: exit ok\n");
      std::fflush(stderr);
    }

    renee_perception::RgbdCaptureRecord writeKeyframe(
      const renee_perception::RgbdCaptureData & data) const
    {
      return writer_.writeCapture(data);
    }

    void finishCapture(
      const CaptureRequest & request, std::uint32_t captured, bool valid,
      const std::string & message, const renee_perception::RgbdCaptureRecord & first_record,
      const std::shared_ptr<GoalHandle> & goal_handle, const std::shared_ptr<Action::Result> & result)
    {
      renee_perception::RgbdStationSummary summary;
      summary.waypoint_id = request.waypoint_id;
      summary.session_dir = request.session_dir;
      summary.camera_model = camera_model_;
      summary.requested_frames = request.target_frames;
      summary.captured_frames = captured;
      summary.simulated = simulated_;
      summary.valid = valid && captured == request.target_frames;
      summary.message = message;
      result->station_metadata_path = writer_.writeStationSummary(summary);
      result->success = valid && captured == request.target_frames;
      result->valid = result->success;
      result->message = message;
      result->captured_frames = captured;
      if (captured > 0) {
        result->timestamp = first_record.timestamp;
        result->rgb_path = first_record.rgb_path;
        result->depth_path = first_record.depth_path;
        result->metadata_path = first_record.metadata_path;
        result->robot_pose = first_record.robot_pose;
        result->camera_pose = first_record.camera_pose;
      }
      if (goal_handle->is_canceling()) {goal_handle->canceled(result);}
      else if (result->success) {goal_handle->succeed(result);}
      else {goal_handle->abort(result);}
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
      const auto goal = goal_handle->get_goal();
      const auto request = requestFromGoal(*goal);
      const std::uint32_t target = request.target_frames;
      const std::string & session = request.session_dir;
      auto result = std::make_shared<Action::Result>();
      bool valid = true;
      std::string message = "Captured ROS RGB-D station";
      std::string settings;
      renee_perception::RgbdCaptureRecord first_record;
      std::uint32_t captured = 0;
      builtin_interfaces::msg::Time previous_stamp;

      try {
        std::fprintf(stderr, "[capture_rgbd][TRACE] execute: goal accepted, waypoint_id='%s' target=%u session_dir='%s'\n",
          request.waypoint_id.c_str(), target, session.c_str());
        std::fflush(stderr);
        publishFeedback(goal_handle, "preparing_camera", 0, target);
        settings = prepareCameraControls();
        std::fprintf(stderr, "[capture_rgbd][TRACE] execute: prepareCameraControls returned, settings.size()=%zu\n",
          settings.size());
        std::fflush(stderr);
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          if (!rgb_buffer_.empty()) {previous_stamp = rgb_buffer_.back()->header.stamp;}
        }
        const auto deadline = std::chrono::steady_clock::now() +
          std::chrono::duration<double>(capture_timeout_sec_);
        // Every iteration waits for a new RGB image, then associates the closest
        // supporting messages before writing one complete keyframe.
      while (captured < target && !goal_handle->is_canceling()) {
        const auto frame_started = std::chrono::steady_clock::now();
        sensor_msgs::msg::Image::ConstSharedPtr rgb;
        renee_perception::RgbdCaptureData data;
          {
            std::unique_lock<std::mutex> lock(data_mutex_);
            data_cv_.wait_until(lock, deadline, [this, &previous_stamp]() {
              return !rgb_buffer_.empty() &&
                (rgb_buffer_.back()->header.stamp != previous_stamp);
            });
            if (std::chrono::steady_clock::now() >= deadline) {
              throw std::runtime_error("Timed out waiting for synchronized camera frames");
            }
            rgb = rgb_buffer_.back();
            previous_stamp = rgb->header.stamp;
            std::fprintf(stderr, "[capture_rgbd][TRACE] execute: got new rgb frame (stamp=%d.%09u), computing closest() matches\n",
              previous_stamp.sec, previous_stamp.nanosec);
            std::fflush(stderr);
            const auto raw_depth = closest<sensor_msgs::msg::Image>(depth_buffer_, previous_stamp, sync_tolerance_ms_);
            const auto aligned = closest<sensor_msgs::msg::Image>(aligned_buffer_, previous_stamp, sync_tolerance_ms_);
            const auto stereo_right = closest<sensor_msgs::msg::Image>(
              stereo_right_buffer_, previous_stamp, sync_tolerance_ms_);
            const auto camera_imu = closest<sensor_msgs::msg::Imu>(
              imu_buffer_, previous_stamp, sync_tolerance_ms_);
            const bool has_camera_imu_sample = std::any_of(
              imu_buffer_.begin(), imu_buffer_.end(), [&previous_stamp](const auto & sample) {
                return rclcpp::Time(sample->header.stamp) <= rclcpp::Time(previous_stamp);
              });
            const auto ir1 = closest<sensor_msgs::msg::Image>(ir1_buffer_, previous_stamp, sync_tolerance_ms_);
            const auto ir2 = closest<sensor_msgs::msg::Image>(ir2_buffer_, previous_stamp, sync_tolerance_ms_);
            const auto color_md = closest<realsense2_camera_msgs::msg::Metadata>(color_metadata_, previous_stamp, sync_tolerance_ms_);
            const auto depth_md = closest<realsense2_camera_msgs::msg::Metadata>(depth_metadata_, previous_stamp, sync_tolerance_ms_);
            const auto ir1_md = closest<realsense2_camera_msgs::msg::Metadata>(ir1_metadata_, previous_stamp, sync_tolerance_ms_);
            const auto ir2_md = closest<realsense2_camera_msgs::msg::Metadata>(ir2_metadata_, previous_stamp, sync_tolerance_ms_);
            std::fprintf(
              stderr,
              "[capture_rgbd][TRACE] execute: matches raw_depth=%d color_info_=%d depth_info_=%d aligned=%d "
              "ir1=%d ir2=%d color_md=%d depth_md=%d ir1_md=%d ir2_md=%d extrinsics_=%d\n",
              static_cast<bool>(raw_depth), static_cast<bool>(color_info_), static_cast<bool>(depth_info_),
              static_cast<bool>(aligned), static_cast<bool>(ir1), static_cast<bool>(ir2),
              static_cast<bool>(color_md), static_cast<bool>(depth_md), static_cast<bool>(ir1_md),
              static_cast<bool>(ir2_md), static_cast<bool>(extrinsics_));
            std::fflush(stderr);
            // ZED simulation requires its stereo pair and IMU. Physical
            // RealSense captures retain their hardware-only requirements.
            if (!raw_depth || !color_info_ || !depth_info_ ||
              (zed2_simulated_ &&
              (!stereo_right || !stereo_right_info_ || !camera_imu || !has_camera_imu_sample)) ||
              (!simulated_ &&
              (!aligned || !ir1 || !ir2 || !color_md || !depth_md || !ir1_md || !ir2_md || !extrinsics_)))
            {
              std::fprintf(stderr, "[capture_rgbd][TRACE] execute: incomplete keyframe, skipping (continue)\n");
              std::fflush(stderr);
              continue;
            }
            std::fprintf(stderr, "[capture_rgbd][TRACE] execute: assigning data fields (rgb/depth images)\n");
            std::fflush(stderr);
            data.waypoint_id = request.waypoint_id;
            data.session_dir = session;
            data.camera_model = camera_model_;
            data.simulated = simulated_;
            data.rgb_image = *rgb;
            data.depth_image = *raw_depth;
            if (zed2_simulated_) {
              data.aligned_depth_image = *raw_depth;
              data.stereo_right_image = *stereo_right;
              data.stereo_right_camera_info = *stereo_right_info_;
            } else if (aligned) {
              data.aligned_depth_image = *aligned;
            }
            if (ir1) {data.infrared_left_image = *ir1;}
            if (ir2) {data.infrared_right_image = *ir2;}
            data.color_camera_info = *color_info_;
            data.depth_camera_info = *depth_info_;
            if (ir1_info_) {data.infrared_left_camera_info = *ir1_info_;}
            if (ir2_info_) {data.infrared_right_camera_info = *ir2_info_;}
            if (color_md) {data.color_metadata_json = color_md->json_data;}
            if (depth_md) {data.depth_metadata_json = depth_md->json_data;}
            if (ir1_md) {data.infrared_left_metadata_json = ir1_md->json_data;}
            if (ir2_md) {data.infrared_right_metadata_json = ir2_md->json_data;}
            if (extrinsics_) {data.depth_to_color_extrinsics_json = extrinsicsJson(*extrinsics_);}
            data.capture_settings_json = settings;
            data.depth_units_m = depth_units_m_;
            const auto odom = closest<nav_msgs::msg::Odometry>(
              odom_buffer_, previous_stamp, kOdomToleranceMs);
            const auto amcl = closest<geometry_msgs::msg::PoseWithCovarianceStamped>(
              amcl_buffer_, previous_stamp, kAmclToleranceMs);
            const auto rover_imu = closest<sensor_msgs::msg::Imu>(
              rover_imu_buffer_, previous_stamp, kOdomToleranceMs);
            data.camera_imu_samples = drainUntil<sensor_msgs::msg::Imu>(imu_buffer_, previous_stamp);
            data.wheel_odometry_samples = drainUntil<nav_msgs::msg::Odometry>(odom_buffer_, previous_stamp);
            data.amcl_pose_samples = drainUntil<geometry_msgs::msg::PoseWithCovarianceStamped>(amcl_buffer_, previous_stamp);
            data.rover_imu_samples = drainUntil<sensor_msgs::msg::Imu>(rover_imu_buffer_, previous_stamp);
            discardOlderThan<sensor_msgs::msg::Image>(rgb_buffer_, previous_stamp, 0.0);
            discardOlderThan<sensor_msgs::msg::Image>(depth_buffer_, previous_stamp, sync_tolerance_ms_);
            discardOlderThan<sensor_msgs::msg::Image>(aligned_buffer_, previous_stamp, sync_tolerance_ms_);
            discardOlderThan<sensor_msgs::msg::Image>(
              stereo_right_buffer_, previous_stamp, sync_tolerance_ms_);
            discardOlderThan<sensor_msgs::msg::Image>(ir1_buffer_, previous_stamp, sync_tolerance_ms_);
            discardOlderThan<sensor_msgs::msg::Image>(ir2_buffer_, previous_stamp, sync_tolerance_ms_);
            discardOlderThan<realsense2_camera_msgs::msg::Metadata>(color_metadata_, previous_stamp, sync_tolerance_ms_);
            discardOlderThan<realsense2_camera_msgs::msg::Metadata>(depth_metadata_, previous_stamp, sync_tolerance_ms_);
            discardOlderThan<realsense2_camera_msgs::msg::Metadata>(ir1_metadata_, previous_stamp, sync_tolerance_ms_);
            discardOlderThan<realsense2_camera_msgs::msg::Metadata>(ir2_metadata_, previous_stamp, sync_tolerance_ms_);
            if (odom) {data.wheel_odometry = *odom; data.has_wheel_odometry = true;}
            if (amcl) {data.amcl_pose = *amcl; data.has_amcl_pose = true;}
            if (rover_imu) {data.rover_imu = *rover_imu; data.has_rover_imu = true;}
            std::fprintf(stderr, "[capture_rgbd][TRACE] execute: data fields assigned ok, leaving locked section\n");
            std::fflush(stderr);
          }

          // A keyframe may be written for diagnosis even when the station is not
          // valid for reconstruction; validity is reported in the final summary.
          std::fprintf(stderr, "[capture_rgbd][TRACE] execute: calling validateRoverState\n");
          std::fflush(stderr);
          valid = validateRoverState(data, request.waypoint_id, message) && valid;
          std::fprintf(stderr, "[capture_rgbd][TRACE] execute: validateRoverState returned valid=%d\n",
            static_cast<int>(valid));
          std::fflush(stderr);

          // Store the transform chain as well as map-relative poses. The chain
          // makes the capture usable if a later map or localization changes.
          publishFeedback(goal_handle, "resolving_transforms", captured, target);
          std::fprintf(stderr, "[capture_rgbd][TRACE] execute: calling resolveTransforms\n");
          std::fflush(stderr);
          resolveTransforms(data, previous_stamp);
          std::fprintf(stderr, "[capture_rgbd][TRACE] execute: resolveTransforms returned ok\n");
          std::fflush(stderr);

          publishFeedback(goal_handle, "writing_keyframe", captured, target);
          std::fprintf(
            stderr,
            "[capture_rgbd][TRACE] execute: calling writeKeyframe (rgb encoding='%s' %ux%u, depth encoding='%s' %ux%u)\n",
            data.rgb_image.encoding.c_str(), data.rgb_image.width, data.rgb_image.height,
            data.depth_image.encoding.c_str(), data.depth_image.width, data.depth_image.height);
          std::fflush(stderr);
          auto record = writeKeyframe(data);
          std::fprintf(stderr, "[capture_rgbd][TRACE] execute: writeKeyframe returned ok\n");
          std::fflush(stderr);
          const auto frame_finished = std::chrono::steady_clock::now();
          const auto elapsed_ms = std::chrono::duration<double, std::milli>(
            frame_finished - frame_started).count();
          RCLCPP_DEBUG(
            get_logger(), "Captured frame %u/%u in %.1f ms", captured + 1, target, elapsed_ms);
          if (captured == 0) {first_record = record;}
          ++captured;
          publishFeedback(goal_handle, "capturing", captured, target);
        }
      } catch (const std::exception & exception) {
        valid = false;
        message = exception.what();
      }

      // Always emit the station summary, including partial and failed captures.
      if (goal_handle->is_canceling()) {valid = false; message = "Capture cancelled";}
      renee_perception::RgbdStationSummary summary;
      summary.waypoint_id = request.waypoint_id;
      summary.session_dir = session;
      summary.camera_model = camera_model_;
      summary.requested_frames = target;
      summary.captured_frames = captured;
      summary.simulated = simulated_;
      summary.valid = valid && captured == target;
      summary.message = message;
      try {result->station_metadata_path = writer_.writeStationSummary(summary);}
      catch (const std::exception & exception) {valid = false; message = exception.what();}

      result->success = valid && captured == target;
      result->valid = result->success;
      result->message = message;
      result->captured_frames = captured;
      if (captured > 0) {
        result->timestamp = first_record.timestamp;
        result->rgb_path = first_record.rgb_path;
        result->depth_path = first_record.depth_path;
        result->metadata_path = first_record.metadata_path;
        result->robot_pose = first_record.robot_pose;
        result->camera_pose = first_record.camera_pose;
      }
      active_goal_ = false;
      if (goal_handle->is_canceling()) {goal_handle->canceled(result);}
      else if (result->success) {goal_handle->succeed(result);}
      else {goal_handle->abort(result);}
    }

    // Configuration selected by rgbd_capture_{sim,real}.yaml or launch overrides.
    std::string mode_, camera_model_, default_session_dir_, map_frame_, odom_frame_, base_frame_;
    std::string camera_frame_, camera_node_;
    bool simulated_{true}, zed2_simulated_{false};
    bool manage_camera_controls_{false}, camera_controls_frozen_{false};
    int default_frame_count_{30};
    double sync_tolerance_ms_{50.0}, capture_timeout_sec_{15.0}, tf_timeout_sec_{0.5};
    double max_linear_speed_{0.01}, max_angular_speed_{0.0175}, camera_warmup_sec_{2.0};
    double depth_units_m_{0.001};

    // Subscribers and the action worker run concurrently; these buffers are
    // protected together so one keyframe is assembled from a consistent snapshot.
    std::mutex data_mutex_;
    std::condition_variable data_cv_;
    std::deque<sensor_msgs::msg::Image::ConstSharedPtr> rgb_buffer_, depth_buffer_, aligned_buffer_;
    std::deque<sensor_msgs::msg::Image::ConstSharedPtr> stereo_right_buffer_, ir1_buffer_, ir2_buffer_;
    std::deque<realsense2_camera_msgs::msg::Metadata::ConstSharedPtr> color_metadata_, depth_metadata_, ir1_metadata_, ir2_metadata_;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_, rover_imu_buffer_;
    std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odom_buffer_;
    std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr> amcl_buffer_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr color_info_, depth_info_, stereo_right_info_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr ir1_info_, ir2_info_;
    realsense2_camera_msgs::msg::Extrinsics::ConstSharedPtr extrinsics_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_, depth_sub_, aligned_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stereo_right_sub_, ir1_sub_, ir2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_, depth_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr stereo_right_info_sub_, ir1_info_sub_, ir2_info_sub_;
    rclcpp::Subscription<realsense2_camera_msgs::msg::Metadata>::SharedPtr color_metadata_sub_, depth_metadata_sub_, ir1_metadata_sub_, ir2_metadata_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr camera_imu_sub_, accel_sub_, gyro_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr rover_imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Subscription<realsense2_camera_msgs::msg::Extrinsics>::SharedPtr extrinsics_sub_;
    rclcpp_action::Server<Action>::SharedPtr action_server_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<rclcpp::AsyncParametersClient> camera_parameter_client_;
    renee_perception::RgbdDatasetWriter writer_;
    std::atomic<bool> active_goal_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaptureRGBDActionServer>());
  rclcpp::shutdown();
  return 0;
}
