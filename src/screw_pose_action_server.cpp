// DetectScrew action server: turns the continuous output of the vision module
// (/screw_pose/metadata_json from screw_pose_ros2) into a one-shot, averaged screw pose.
// The detector itself runs in the renee_vision_modules container; this node only needs the topic.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include "renee_action_servers/action/detect_screw.hpp"

using DetectScrew = renee_action_servers::action::DetectScrew;
using GoalHandleDetectScrew = rclcpp_action::ServerGoalHandle<DetectScrew>;
using json = nlohmann::json;

namespace
{

struct Sample
{
  double p[3];
  double q[4];  // x, y, z, w
  double confidence;
};

struct Frame
{
  uint64_t seq{0};
  json data;
};

struct Track
{
  std::vector<Sample> window;  // last num_samples samples
  std::string class_name;
};

// RMS distance of the sampled positions to their mean.
double positionStd(const std::vector<Sample> & s, const double mean[3])
{
  double acc = 0.0;
  for (const auto & x : s) {
    for (int i = 0; i < 3; ++i) {
      acc += (x.p[i] - mean[i]) * (x.p[i] - mean[i]);
    }
  }
  return std::sqrt(acc / static_cast<double>(s.size()));
}

}  // namespace

class ScrewPoseActionServer : public rclcpp::Node
{
public:
  ScrewPoseActionServer()
  : rclcpp::Node("screw_pose_action_server")
  {
    const auto topic = declare_parameter<std::string>("metadata_topic", "/screw_pose/metadata_json");
    default_num_samples_ = declare_parameter<int>("default_num_samples", 5);
    default_timeout_sec_ = declare_parameter<double>("default_timeout_sec", 10.0);
    default_min_confidence_ = declare_parameter<double>("default_min_confidence", 0.3);
    max_position_std_m_ = declare_parameter<double>("max_position_std_m", 0.005);

    sub_ = create_subscription<std_msgs::msg::String>(
      topic, rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::String::SharedPtr msg) {onMetadata(*msg);});

    server_ = rclcpp_action::create_server<DetectScrew>(
      this, "detect_screw",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const DetectScrew::Goal> goal) {
        if (goal->num_samples > 1000 || goal->timeout_sec < 0.0 || goal->min_confidence < 0.0) {
          RCLCPP_WARN(get_logger(), "Rejecting DetectScrew goal with invalid fields");
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<GoalHandleDetectScrew>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<GoalHandleDetectScrew> gh) {
        std::thread([this, gh]() {execute(gh);}).detach();
      });

    RCLCPP_INFO(get_logger(), "DetectScrew server on 'detect_screw' (reads %s)", topic.c_str());
  }

private:
  void onMetadata(const std_msgs::msg::String & msg)
  {
    json parsed = json::parse(msg.data, nullptr, false);
    if (parsed.is_discarded()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring malformed screw metadata");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_.data = std::move(parsed);
      ++latest_.seq;
    }
    cv_.notify_all();
  }

  void execute(const std::shared_ptr<GoalHandleDetectScrew> gh)
  {
    const auto goal = gh->get_goal();
    auto result = std::make_shared<DetectScrew::Result>();
    const size_t n = goal->num_samples > 0 ? goal->num_samples : default_num_samples_;
    const double min_conf =
      goal->min_confidence > 0.0 ? goal->min_confidence : default_min_confidence_;
    const double timeout =
      goal->timeout_sec > 0.0 ? goal->timeout_sec : default_timeout_sec_;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout);

    // Only frames received after the goal count, so a stale detection is never returned.
    uint64_t last_seq;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_seq = latest_.seq;
    }

    std::map<int, Track> tracks;
    auto feedback = std::make_shared<DetectScrew::Feedback>();
    feedback->target_samples = n;
    std::string last_problem = "no detections received (is the vision node running?)";

    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        result->message = "canceled";
        gh->canceled(result);
        return;
      }
      if (std::chrono::steady_clock::now() > deadline) {
        result->success = false;
        result->message = "timeout: " + last_problem;
        gh->abort(result);
        return;
      }

      json frame;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(100), [&] {return latest_.seq != last_seq;});
        if (latest_.seq == last_seq) {
          continue;
        }
        last_seq = latest_.seq;
        frame = latest_.data;
      }

      const std::string robot_frame = frame.value("robot_frame", std::string());
      if (robot_frame.empty()) {
        last_problem = "vision node has no target_frame set (launch it with target_frame:=...)";
        continue;
      }

      std::map<int, Track> seen;  // tracks matching the goal in this frame
      size_t matches = 0;
      for (const auto & d : frame.value("detections", json::array())) {
        if (!d.contains("position_robot_m") || !d.contains("quaternion_robot_xyzw")) {
          continue;  // TF to the robot frame failed for this detection
        }
        const std::string cls = d.value("class_name", std::string());
        const int tid = d.value("track_id", -1);
        const double conf = d.value("confidence", 0.0);
        if (!goal->target_class.empty() && cls != goal->target_class) {continue;}
        if (goal->target_track_id >= 0 && tid != goal->target_track_id) {continue;}
        if (conf < min_conf) {continue;}

        Sample s;
        for (int i = 0; i < 3; ++i) {s.p[i] = d["position_robot_m"][i].get<double>();}
        for (int i = 0; i < 4; ++i) {s.q[i] = d["quaternion_robot_xyzw"][i].get<double>();}
        s.confidence = conf;

        auto & t = tracks[tid];
        t.class_name = cls;
        t.window.push_back(s);
        if (t.window.size() > n) {
          t.window.erase(t.window.begin());
        }
        seen[tid] = t;
        ++matches;
      }
      if (matches == 0) {
        last_problem = "no matching screw (class='" + goal->target_class + "', track=" +
          std::to_string(goal->target_track_id) + ", confidence>=" + std::to_string(min_conf) + ")";
      }

      // A track that is missing from a frame loses continuity: restart its window.
      for (auto it = tracks.begin(); it != tracks.end();) {
        it = seen.count(it->first) ? std::next(it) : tracks.erase(it);
      }

      size_t best = 0;
      for (const auto & kv : tracks) {best = std::max(best, kv.second.window.size());}
      feedback->phase = best == 0 ? "waiting_for_detections" : "collecting";
      feedback->collected_samples = best;
      gh->publish_feedback(feedback);

      // Pick the full window with the best mean confidence.
      int chosen = -2;
      double chosen_conf = -1.0;
      for (const auto & kv : tracks) {
        if (kv.second.window.size() < n) {continue;}
        double c = 0.0;
        for (const auto & s : kv.second.window) {c += s.confidence;}
        c /= static_cast<double>(n);
        if (c > chosen_conf) {chosen = kv.first; chosen_conf = c;}
      }
      if (chosen == -2) {continue;}

      const auto & w = tracks[chosen].window;
      double mean[3] = {0, 0, 0};
      double q[4] = {0, 0, 0, 0};
      for (const auto & s : w) {
        const double sign = (s.q[0] * w[0].q[0] + s.q[1] * w[0].q[1] + s.q[2] * w[0].q[2] +
          s.q[3] * w[0].q[3]) < 0.0 ? -1.0 : 1.0;  // q and -q are the same rotation
        for (int i = 0; i < 3; ++i) {mean[i] += s.p[i] / static_cast<double>(n);}
        for (int i = 0; i < 4; ++i) {q[i] += sign * s.q[i];}
      }
      const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
      const double std_m = positionStd(w, mean);
      if (std_m > max_position_std_m_) {
        last_problem = "screw position unstable (rms " + std::to_string(std_m * 1000.0) +
          " mm > " + std::to_string(max_position_std_m_ * 1000.0) + " mm)";
        continue;  // keep sliding the window until it settles or the goal times out
      }

      result->success = true;
      result->message = "screw " + tracks[chosen].class_name + " track " +
        std::to_string(chosen) + " from " + std::to_string(n) + " frames";
      result->pose.header.frame_id = robot_frame;
      result->pose.header.stamp = now();
      result->pose.pose.position.x = mean[0];
      result->pose.pose.position.y = mean[1];
      result->pose.pose.position.z = mean[2];
      result->pose.pose.orientation.x = q[0] / norm;
      result->pose.pose.orientation.y = q[1] / norm;
      result->pose.pose.orientation.z = q[2] / norm;
      result->pose.pose.orientation.w = q[3] / norm;
      result->class_name = tracks[chosen].class_name;
      result->track_id = chosen;
      result->confidence = chosen_conf;
      result->num_samples = static_cast<uint32_t>(n);
      result->position_std_m = std_m;
      gh->succeed(result);
      return;
    }
    result->message = "shutdown";
    gh->abort(result);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp_action::Server<DetectScrew>::SharedPtr server_;
  std::mutex mutex_;
  std::condition_variable cv_;
  Frame latest_;
  int default_num_samples_{5};
  double default_timeout_sec_{10.0};
  double default_min_confidence_{0.3};
  double max_position_std_m_{0.005};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScrewPoseActionServer>());
  rclcpp::shutdown();
  return 0;
}
