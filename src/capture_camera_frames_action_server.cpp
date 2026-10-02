/** @file capture_camera_frames_action_server.cpp
 *  @brief Returns RGB or RGB-D frames from a camera service (the ZED2i on the Jetson).
 *
 *  The camera is attached to a Jetson that has no ROS. A service there keeps the
 *  camera warm and answers requests over a TCP link (see camera_link.hpp). This
 *  action asks it for 1 or N frames, converts the capture times to PC time using
 *  an NTP-style offset estimate, validates them, and hands the frames to ROS:
 *  one frame in the action result, N frames on topics. Nothing is saved to disk.
 */

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "renee_action_servers/action/capture_camera_frames.hpp"
#include "renee_action_servers/camera_link.hpp"
#include "renee_action_servers/clock_sync.hpp"

namespace
{
  namespace clink = renee_action_servers::camera_link;
  namespace csync = renee_action_servers::clock_sync;

  // Capture times come from the Jetson's wall clock, so PC time is the PC's wall
  // clock too (not the ROS clock, which may be simulated).
  std::int64_t systemNowNs()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  }

  builtin_interfaces::msg::Time toTimeMsg(std::int64_t ns)
  {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    return stamp;
  }

  // Link statistics from one clock synchronisation (the best of several pings).
  struct SyncResult
  {
    csync::ClockSample sample;
    bool clock_ok{false};
    std::optional<double> chrony_offset_ms;
  };
}  // namespace

/**
 * Action server for `capture_camera_frames`. One goal at a time: the camera
 * service is the single owner of the camera and its connection is shared with the
 * idle resync timer, so a mutex serialises both.
 */
class CaptureCameraFramesActionServer : public rclcpp::Node
{
  public:
    using Action = renee_action_servers::action::CaptureCameraFrames;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

    CaptureCameraFramesActionServer()
    : Node("capture_camera_frames_action_server")
    {
      link_host_ = declare_parameter<std::string>("link_host", "127.0.0.1");
      link_port_ = declare_parameter<int>("link_port", 7788);
      connect_timeout_sec_ = declare_parameter<double>("connect_timeout_sec", 3.0);
      default_timeout_sec_ = declare_parameter<double>("default_timeout_sec", 20.0);
      max_frames_ = declare_parameter<int>("max_frames", 100);
      ping_count_ = declare_parameter<int>("ping_count", 5);
      resync_period_sec_ = declare_parameter<double>("resync_period_sec", 5.0);
      limits_.max_rtt_ms = declare_parameter<double>("max_rtt_ms", 100.0);
      limits_.max_clock_offset_ms = declare_parameter<double>("max_clock_offset_ms", 50.0);
      camera_frame_ = declare_parameter<std::string>(
        "camera_frame", "robot_arm_rgbd_camera_left_camera_optical_frame");
      const std::string prefix = declare_parameter<std::string>("topic_prefix", "~/");
      const int queue_depth = declare_parameter<int>("topic_queue_depth", 10);

      const auto qos = rclcpp::QoS(static_cast<std::size_t>(std::max(1, queue_depth))).reliable();
      rgb_pub_ = create_publisher<sensor_msgs::msg::Image>(prefix + "rgb/image_raw", qos);
      depth_pub_ = create_publisher<sensor_msgs::msg::Image>(prefix + "depth/image_raw", qos);
      info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(prefix + "camera_info", qos);

      action_server_ = rclcpp_action::create_server<Action>(
        this, "capture_camera_frames",
        std::bind(&CaptureCameraFramesActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CaptureCameraFramesActionServer::handleCancel, this, std::placeholders::_1),
        std::bind(&CaptureCameraFramesActionServer::handleAccepted, this, std::placeholders::_1));

      // Keeps an established connection alive and the offset estimate fresh. It
      // never reconnects: that would block the executor while the Jetson is off.
      if (resync_period_sec_ > 0.0) {
        resync_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(resync_period_sec_)),
          std::bind(&CaptureCameraFramesActionServer::resyncIdle, this));
      }
      RCLCPP_INFO(
        get_logger(), "CaptureCameraFrames ready (camera service at %s:%d)",
        link_host_.c_str(), link_port_);
    }

  private:
    rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>)
    {
      // Only one capture may own the camera service connection at a time.
      if (active_goal_.exchange(true)) {return rclcpp_action::GoalResponse::REJECT;}
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
    {
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
      std::thread([this, goal_handle]() {execute(goal_handle);}).detach();
    }

    void publishFeedback(
      const std::shared_ptr<GoalHandle> & goal_handle, const std::string & phase,
      std::uint32_t captured, std::uint32_t target)
    {
      auto feedback = std::make_shared<Action::Feedback>();
      feedback->phase = phase;
      feedback->captured_frames = captured;
      feedback->target_frames = target;
      goal_handle->publish_feedback(feedback);
    }

    // Caller holds link_mutex_.
    void ensureConnected()
    {
      if (!link_.connected()) {link_.connectTo(link_host_, link_port_, connect_timeout_sec_);}
    }

    // One ping exchange; the stream is closed by the caller on any exception.
    SyncResult pingOnce(const clink::CameraLink::CancelFn & cancel)
    {
      const std::int64_t t0 = systemNowNs();
      link_.send(clink::Json::object({{"type", "ping"}, {"t0", t0}}));
      const auto reply = link_.receive(2.0, cancel);
      const std::int64_t t3 = systemNowNs();
      const auto & header = reply.header;
      if (header["type"] == "error") {
        throw clink::LinkError("camera service error: " + header.value("message", std::string("?")));
      }
      if (header["type"] != "pong" || header.value("t0", std::int64_t{-1}) != t0) {
        throw clink::LinkError("unexpected reply to ping");
      }
      SyncResult result;
      result.sample = csync::computeSample(
        t0, header.at("t1").get<std::int64_t>(), header.at("t2").get<std::int64_t>(), t3);
      result.clock_ok = header.value("clock_ok", false);
      if (header.contains("chrony_offset_ms") && header["chrony_offset_ms"].is_number()) {
        result.chrony_offset_ms = header["chrony_offset_ms"].get<double>();
      }
      return result;
    }

    // Best (smallest round trip) of ping_count_ exchanges. Caller holds link_mutex_.
    SyncResult syncClock(const clink::CameraLink::CancelFn & cancel)
    {
      std::vector<SyncResult> results;
      std::vector<csync::ClockSample> samples;
      for (int i = 0; i < std::max(1, ping_count_); ++i) {
        results.push_back(pingOnce(cancel));
        samples.push_back(results.back().sample);
      }
      return results[csync::bestSampleIndex(samples)];
    }

    // A connection left open from an earlier goal may have gone stale; retry once
    // on a fresh connection before reporting the Jetson as unreachable.
    SyncResult connectAndSync(const clink::CameraLink::CancelFn & cancel)
    {
      try {
        ensureConnected();
        return syncClock(cancel);
      } catch (const clink::LinkCancelled &) {
        throw;
      } catch (const clink::LinkError &) {
        link_.close();
        ensureConnected();
        return syncClock(cancel);
      }
    }

    void resyncIdle()
    {
      if (active_goal_) {return;}
      std::unique_lock<std::mutex> lock(link_mutex_, std::try_to_lock);
      if (!lock.owns_lock() || !link_.connected()) {return;}
      try {
        const auto result = syncClock(nullptr);
        RCLCPP_DEBUG(
          get_logger(), "clock offset %.3f ms, rtt %.3f ms",
          static_cast<double>(result.sample.offset_ns) / 1.0e6,
          static_cast<double>(result.sample.rtt_ns) / 1.0e6);
        const auto reason = csync::validateLink(
          result.sample, result.clock_ok, result.chrony_offset_ms, limits_);
        if (!reason.empty()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "Camera link: %s", reason.c_str());
        }
      } catch (const std::exception & exception) {
        RCLCPP_WARN(get_logger(), "Camera link lost (%s); will reconnect on the next goal", exception.what());
        link_.close();
      }
    }

    sensor_msgs::msg::CameraInfo makeCameraInfo(
      const clink::CameraInfoData & data, const builtin_interfaces::msg::Time & stamp) const
    {
      sensor_msgs::msg::CameraInfo info;
      info.header.stamp = stamp;
      info.header.frame_id = camera_frame_;
      info.width = data.width;
      info.height = data.height;
      info.distortion_model = data.distortion_model;
      info.d = data.D;
      std::copy(data.K.begin(), data.K.end(), info.k.begin());
      std::copy(data.R.begin(), data.R.end(), info.r.begin());
      std::copy(data.P.begin(), data.P.end(), info.p.begin());
      return info;
    }

    sensor_msgs::msg::Image makeRgbImage(
      const clink::DecodedFrame & frame, const builtin_interfaces::msg::Time & stamp) const
    {
      sensor_msgs::msg::Image image;
      image.header.stamp = stamp;
      image.header.frame_id = camera_frame_;
      image.width = frame.width;
      image.height = frame.height;
      image.encoding = frame.rgb_encoding;
      image.is_bigendian = false;
      image.step = frame.rgb_step;
      image.data = frame.rgb;
      return image;
    }

    sensor_msgs::msg::Image makeDepthImage(
      const clink::DecodedFrame & frame, const builtin_interfaces::msg::Time & stamp) const
    {
      sensor_msgs::msg::Image image;
      image.header.stamp = stamp;
      image.header.frame_id = camera_frame_;
      image.width = frame.width;
      image.height = frame.height;
      image.encoding = "32FC1";
      image.is_bigendian = false;
      image.step = frame.width * static_cast<std::uint32_t>(sizeof(float));
      image.data.resize(frame.depth_m.size() * sizeof(float));
      std::memcpy(image.data.data(), frame.depth_m.data(), image.data.size());
      return image;
    }

    // Tells the service to stop (if a capture is in flight) and drops the connection.
    void abandonCapture(bool capture_started)
    {
      if (capture_started) {
        try {link_.send(clink::Json::object({{"type", "cancel"}}));} catch (const std::exception &) {}
      }
      link_.close();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
      const std::int64_t goal_received_ns = systemNowNs();
      const auto goal = goal_handle->get_goal();
      const std::uint32_t target = goal->num_frames > 0 ? goal->num_frames : 1;
      const bool need_depth = goal->mode == "rgbd";
      const double timeout_sec = goal->timeout_sec > 0.0 ? goal->timeout_sec : default_timeout_sec_;
      const auto cancel = [&goal_handle]() {return goal_handle->is_canceling();};

      auto result = std::make_shared<Action::Result>();
      result->success = false;
      result->timestamps_valid = false;
      std::string message;
      bool all_valid = false;
      bool capture_started = false;

      {
        std::lock_guard<std::mutex> lock(link_mutex_);
        try {
          if (goal->mode != "rgb" && goal->mode != "rgbd") {
            throw std::invalid_argument("mode must be 'rgb' or 'rgbd', got '" + goal->mode + "'");
          }
          if (target > static_cast<std::uint32_t>(max_frames_)) {
            throw std::invalid_argument(
              "num_frames " + std::to_string(target) + " exceeds max_frames " +
              std::to_string(max_frames_));
          }
          const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout_sec));
          const auto remaining = [&deadline]() {
              return std::max(
                0.001, std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count());
            };

          publishFeedback(goal_handle, "connecting", 0, target);
          publishFeedback(goal_handle, "syncing_clock", 0, target);
          const SyncResult synced = connectAndSync(cancel);
          const csync::ClockSample & sample = synced.sample;
          result->clock_offset_ms = static_cast<double>(sample.offset_ns) / 1.0e6;
          result->round_trip_ms = static_cast<double>(sample.rtt_ns) / 1.0e6;
          const std::string link_problem = csync::validateLink(
            sample, synced.clock_ok, synced.chrony_offset_ms, limits_);
          if (!link_problem.empty()) {throw clink::LinkError(link_problem);}

          publishFeedback(goal_handle, "capturing", 0, target);
          capture_started = true;
          link_.send(clink::Json::object(
              {{"type", "capture"}, {"mode", goal->mode}, {"num_frames", target},
                {"t0", systemNowNs()}}));

          std::optional<std::int64_t> previous_pc_ns;
          while (true) {
            const auto reply = link_.receive(remaining(), cancel);
            const std::string type = reply.header["type"].get<std::string>();
            if (type == "error") {
              throw clink::LinkError(
                "camera service error: " + reply.header.value("message", std::string("?")));
            }
            if (type == "done") {
              if (reply.header.value("n_frames", 0u) != result->stamps.size()) {
                throw clink::LinkError("camera service frame count does not match the frames received");
              }
              break;
            }
            if (type != "frame") {throw clink::LinkError("unexpected message '" + type + "'");}
            if (result->stamps.size() >= target) {
              throw clink::LinkError("camera service sent more frames than requested");
            }

            const auto frame = clink::decodeFrame(reply, need_depth);
            const std::int64_t pc_ns = csync::toPcTime(frame.capture_ts_ns, sample.offset_ns);
            const std::string bad_stamp = csync::validateFrameStamp(
              pc_ns, goal_received_ns, systemNowNs(), sample.rtt_ns, previous_pc_ns);
            if (!bad_stamp.empty()) {
              throw clink::LinkError(
                "frame " + std::to_string(result->stamps.size()) + ": " + bad_stamp);
            }
            previous_pc_ns = pc_ns;

            const auto stamp = toTimeMsg(pc_ns);
            result->stamps.push_back(stamp);
            result->captured_frames = static_cast<std::uint32_t>(result->stamps.size());
            result->camera_info = makeCameraInfo(frame.info, stamp);
            if (target == 1) {
              result->stamp = stamp;
              result->rgb = makeRgbImage(frame, stamp);
              if (frame.has_depth && need_depth) {result->depth = makeDepthImage(frame, stamp);}
            } else {
              if (result->stamps.size() == 1) {result->stamp = stamp;}
              rgb_pub_->publish(makeRgbImage(frame, stamp));
              if (frame.has_depth && need_depth) {depth_pub_->publish(makeDepthImage(frame, stamp));}
              info_pub_->publish(result->camera_info);
            }
            publishFeedback(goal_handle, "capturing", result->captured_frames, target);
            if (result->captured_frames == target) {
              publishFeedback(goal_handle, "validating", result->captured_frames, target);
            }
          }

          if (result->captured_frames != target) {
            throw clink::LinkError(
              "camera service finished after " + std::to_string(result->captured_frames) +
              " of " + std::to_string(target) + " frames");
          }
          all_valid = true;
          message = "Captured " + std::to_string(target) + (target == 1 ? " frame" : " frames") +
            " (" + goal->mode + ")";
        } catch (const clink::LinkCancelled &) {
          // The stream still has frames in flight; drop it so the next goal starts clean.
          abandonCapture(capture_started);
          message = "Capture cancelled";
        } catch (const std::invalid_argument & exception) {
          message = exception.what();
        } catch (const std::exception & exception) {
          // An unfinished frame stream cannot be resumed, so reconnect next time.
          abandonCapture(capture_started);
          message = exception.what();
        }
      }

      result->success = all_valid;
      result->timestamps_valid = all_valid;
      result->message = message;
      publishFeedback(goal_handle, "validating", result->captured_frames, target);
      active_goal_ = false;
      if (goal_handle->is_canceling()) {goal_handle->canceled(result);}
      else if (result->success) {goal_handle->succeed(result);}
      else {goal_handle->abort(result);}
    }

    std::string link_host_, camera_frame_;
    int link_port_{7788}, max_frames_{100}, ping_count_{5};
    double connect_timeout_sec_{3.0}, default_timeout_sec_{20.0}, resync_period_sec_{5.0};
    csync::LinkLimits limits_;

    std::mutex link_mutex_;
    clink::CameraLink link_;
    std::atomic<bool> active_goal_{false};
    rclcpp::TimerBase::SharedPtr resync_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_, depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp_action::Server<Action>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CaptureCameraFramesActionServer>());
  rclcpp::shutdown();
  return 0;
}
