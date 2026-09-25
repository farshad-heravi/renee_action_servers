/** @file zed_ssh_capture_action_server.cpp
 *  @brief Triggers an SSH-based ZED capture on the Jetson.
 *
 *  The real ZED camera is physically attached to the Jetson, which has no
 *  ROS install (its JetPack/L4T version is too old for ROS2). This action
 *  wraps the same ssh + record_data.py --headless pattern already used by
 *  capture_aruco_pose.sh, so the capture is available as a ROS2 action on
 *  the PC instead of a manual shell script. The dataset is left on the
 *  Jetson; transferring it back is done manually.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "renee_action_servers/action/capture_zed_image.hpp"

namespace
{
  using namespace std::chrono_literals;

  // Runs argv as a child process, killing it if it outlives timeout_sec or
  // should_cancel() starts returning true. Returns the child's exit code,
  // or -1 if it had to be killed.
  int runCommandWithTimeout(
    const std::vector<std::string> & argv,
    double timeout_sec,
    const std::function<bool()> & should_cancel)
  {
    std::vector<char *> c_argv;
    c_argv.reserve(argv.size() + 1);
    for (const auto & arg : argv) {c_argv.push_back(const_cast<char *>(arg.c_str()));}
    c_argv.push_back(nullptr);

    const pid_t pid = fork();
    if (pid < 0) {throw std::runtime_error("fork() failed");}
    if (pid == 0) {
      execvp(c_argv[0], c_argv.data());
      _exit(127);
    }

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);
    int status = 0;
    while (true) {
      const pid_t result = waitpid(pid, &status, WNOHANG);
      if (result == pid) {
        return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
      }
      if (std::chrono::steady_clock::now() >= deadline || should_cancel()) {
        kill(pid, SIGKILL);
        waitpid(pid, &status, 0);
        return -1;
      }
      std::this_thread::sleep_for(100ms);
    }
  }

  std::string timestampedDirName()
  {
    const auto now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);
    std::ostringstream out;
    out << "zed_capture_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return out.str();
  }

  // Runs a shell command and parses its stdout as an unsigned integer.
  // Returns 0 if the command fails or prints nothing parseable.
  std::size_t readCountFromCommand(const std::string & command)
  {
    FILE * pipe = popen(command.c_str(), "r");
    if (!pipe) {return 0;}
    unsigned long count = 0;
    if (std::fscanf(pipe, "%lu", &count) != 1) {count = 0;}
    pclose(pipe);
    return count;
  }
}  // namespace

/**
 * in a ROS2 action, so a ZED snapshot can be requested
 * followed by `scp` in a ROS2 action, so a ZED snapshot can be requested
 * from the rest of the ROS graph without a manual shell script.
 */
class ZedSshCaptureActionServer : public rclcpp::Node
{
  public:
    using Action = renee_action_servers::action::CaptureZedImage;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

    ZedSshCaptureActionServer()
    : Node("zed_ssh_capture_action_server")
    {
      jetson_host_ = declare_parameter<std::string>("jetson_host", "jetson");
      jetson_repo_dir_ = declare_parameter<std::string>("jetson_repo_dir", "renee_perception");
      default_image_count_ = declare_parameter<int>("default_image_count", 30);
      capture_timeout_sec_ = declare_parameter<double>("capture_timeout_sec", 60.0);
      ssh_connect_timeout_sec_ = declare_parameter<int>("ssh_connect_timeout_sec", 6);
      depth_mode_ = declare_parameter<std::string>("depth_mode", "QUALITY");

      action_server_ = rclcpp_action::create_server<Action>(
        this, "capture_zed_image",
        std::bind(&ZedSshCaptureActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ZedSshCaptureActionServer::handleCancel, this, std::placeholders::_1),
        std::bind(&ZedSshCaptureActionServer::handleAccepted, this, std::placeholders::_1));
      RCLCPP_INFO(
        get_logger(), "CaptureZedImage ready (jetson_host=%s, jetson_repo_dir=%s)",
        jetson_host_.c_str(), jetson_repo_dir_.c_str());
    }

  private:
    rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>)
    {
      // Only one process can hold the ZED at a time, so only one capture
      // may be in flight.
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
      feedback->captured_images = captured;
      feedback->target_images = target;
      goal_handle->publish_feedback(feedback);
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
      const auto goal = goal_handle->get_goal();
      const std::uint32_t target = goal->image_count > 0 ?
        goal->image_count : static_cast<std::uint32_t>(default_image_count_);
      const std::string remote_dir = goal->session_dir.empty() ?
        timestampedDirName() : goal->session_dir;

      auto result = std::make_shared<Action::Result>();
      result->success = false;
      result->captured_images = 0;
      const auto should_cancel = [&goal_handle]() {return goal_handle->is_canceling();};

      try {
        publishFeedback(goal_handle, "connecting", 0, target);
        const int reachable = runCommandWithTimeout(
          {"ssh", "-o", "BatchMode=yes", "-o",
            "ConnectTimeout=" + std::to_string(ssh_connect_timeout_sec_),
            jetson_host_, "true"},
          static_cast<double>(ssh_connect_timeout_sec_) + 2.0, should_cancel);
        if (reachable != 0) {
          throw std::runtime_error("Could not reach " + jetson_host_ + " over SSH");
        }

        publishFeedback(goal_handle, "capturing_on_jetson", 0, target);
        std::ostringstream remote_cmd;
        remote_cmd << "cd ~/" << jetson_repo_dir_
          << " && python3 tools/record_data.py --camera zed --headless"
          << " --output ~/" << jetson_repo_dir_ << "/" << remote_dir
          << " --images-per-shot " << target
          << " --depth-mode " << depth_mode_
          << " --timeout " << capture_timeout_sec_;
        const int captured_ok = runCommandWithTimeout(
          {"ssh", "-o", "BatchMode=yes", jetson_host_, remote_cmd.str()},
          capture_timeout_sec_ + 15.0, should_cancel);
        if (captured_ok != 0) {
          throw std::runtime_error("ZED capture on the Jetson failed or timed out");
        }

        publishFeedback(goal_handle, "verifying", 0, target);
        const std::string output_dir = "~/" + jetson_repo_dir_ + "/" + remote_dir;
        const std::size_t captured = readCountFromCommand(
          "ssh -o BatchMode=yes -o ConnectTimeout=" + std::to_string(ssh_connect_timeout_sec_) +
          " " + jetson_host_ + " 'ls -A " + output_dir + "/rgb 2>/dev/null | wc -l'");
        result->success = captured > 0;
        result->message = result->success ?
          "Captured ZED images on the Jetson (not transferred)" :
          "Capture finished but no RGB images were found on the Jetson";
        result->captured_images = static_cast<std::uint32_t>(captured);
        result->output_dir = jetson_host_ + ":" + output_dir;
        result->rgb_path = result->output_dir + "/rgb";
        result->session_metadata_path = result->output_dir + "/session.json";
      } catch (const std::exception & exception) {
        result->success = false;
        result->message = exception.what();
      }

      publishFeedback(goal_handle, "done", result->captured_images, target);
      active_goal_ = false;
      if (goal_handle->is_canceling()) {goal_handle->canceled(result);}
      else if (result->success) {goal_handle->succeed(result);}
      else {goal_handle->abort(result);}
    }

    std::string jetson_host_, jetson_repo_dir_, depth_mode_;
    int default_image_count_{30};
    double capture_timeout_sec_{60.0};
    int ssh_connect_timeout_sec_{6};
    std::atomic<bool> active_goal_{false};
    rclcpp_action::Server<Action>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedSshCaptureActionServer>());
  rclcpp::shutdown();
  return 0;
}
