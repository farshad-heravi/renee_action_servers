/** @file clock_sync.hpp
 *  @brief Clock-offset estimation and timestamp validation for the camera link.
 *
 *  Pure functions, no ROS or socket dependency, so they are unit-testable.
 *  All times are int64 nanoseconds since the Unix epoch on the stated clock.
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace renee_action_servers
{
  namespace clock_sync
  {
    /** One NTP-style exchange: PC sends at t0, service receives at t1 and replies
     *  at t2, PC receives at t3. t0/t3 are PC clock, t1/t2 are Jetson clock. */
    struct ClockSample
    {
      std::int64_t t0{0}, t1{0}, t2{0}, t3{0};
      std::int64_t offset_ns{0};  // Jetson clock minus PC clock
      std::int64_t rtt_ns{0};     // network round trip, excluding service processing
    };

    inline ClockSample computeSample(
      std::int64_t t0, std::int64_t t1, std::int64_t t2, std::int64_t t3)
    {
      ClockSample sample;
      sample.t0 = t0;
      sample.t1 = t1;
      sample.t2 = t2;
      sample.t3 = t3;
      sample.offset_ns = ((t1 - t0) + (t2 - t3)) / 2;
      sample.rtt_ns = (t3 - t0) - (t2 - t1);
      return sample;
    }

    /** The sample with the smallest round trip is the least affected by queueing. */
    inline std::size_t bestSampleIndex(const std::vector<ClockSample> & samples)
    {
      if (samples.empty()) {throw std::invalid_argument("no clock samples");}
      return static_cast<std::size_t>(std::distance(
        samples.begin(), std::min_element(
          samples.begin(), samples.end(),
          [](const ClockSample & a, const ClockSample & b) {return a.rtt_ns < b.rtt_ns;})));
    }

    /** Converts a Jetson-clock time to PC time. */
    inline std::int64_t toPcTime(std::int64_t jetson_ns, std::int64_t offset_ns)
    {
      return jetson_ns - offset_ns;
    }

    struct LinkLimits
    {
      double max_rtt_ms{100.0};
      double max_clock_offset_ms{50.0};  // applies to the service's own chrony offset, if reported
    };

    /** Returns an empty string if the link is trustworthy, else the reason it is not. */
    inline std::string validateLink(
      const ClockSample & sample, bool clock_ok, const std::optional<double> & chrony_offset_ms,
      const LinkLimits & limits)
    {
      std::ostringstream out;
      const double rtt_ms = static_cast<double>(sample.rtt_ns) / 1.0e6;
      if (rtt_ms < 0.0 || rtt_ms > limits.max_rtt_ms) {
        out << "Link round trip " << rtt_ms << " ms is outside [0, " << limits.max_rtt_ms << "] ms";
        return out.str();
      }
      if (!clock_ok) {return "Camera service reports its clock is not synchronised";}
      if (chrony_offset_ms && std::abs(*chrony_offset_ms) > limits.max_clock_offset_ms) {
        out << "Camera service chrony offset " << *chrony_offset_ms << " ms exceeds "
            << limits.max_clock_offset_ms << " ms";
        return out.str();
      }
      return "";
    }

    /** Checks one frame's capture time (already converted to PC time).
     *  The offset is only known to within rtt/2, which is used as the tolerance.
     *  Returns an empty string if valid. */
    inline std::string validateFrameStamp(
      std::int64_t pc_capture_ns, std::int64_t goal_received_ns, std::int64_t now_ns,
      std::int64_t rtt_ns, const std::optional<std::int64_t> & previous_ns)
    {
      const std::int64_t tolerance = rtt_ns / 2;
      if (pc_capture_ns < goal_received_ns - tolerance) {
        return "frame was captured before the goal was received (stale frame)";
      }
      if (pc_capture_ns > now_ns + tolerance) {
        return "frame capture time is in the future (clock offset is wrong)";
      }
      if (previous_ns && pc_capture_ns <= *previous_ns) {
        return "frame timestamps are not strictly increasing";
      }
      return "";
    }
  }  // namespace clock_sync
}  // namespace renee_action_servers
