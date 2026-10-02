/** @file test_camera_link.cpp
 *  @brief Unit tests for clock_sync.hpp and camera_link.hpp (no ROS needed).
 */

#include <gtest/gtest.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <thread>
#include <vector>

#include "renee_action_servers/camera_link.hpp"
#include "renee_action_servers/clock_sync.hpp"

namespace clink = renee_action_servers::camera_link;
namespace csync = renee_action_servers::clock_sync;
using clink::Json;

namespace
{
  constexpr std::int64_t kMs = 1000000;

  // A connected AF_UNIX pair: .first is adopted by the link under test, .second is the "service".
  std::pair<int, int> socketPair()
  {
    int fds[2];
    EXPECT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0);
    return {fds[0], fds[1]};
  }

  void writeAll(int fd, const clink::Blob & bytes)
  {
    ASSERT_EQ(::write(fd, bytes.data(), bytes.size()), static_cast<ssize_t>(bytes.size()));
  }

  clink::Blob zlibOf(const clink::Blob & raw)
  {
    return clink::zlibCompress(raw.data(), raw.size());
  }

  Json cameraInfoJson(std::uint32_t width, std::uint32_t height)
  {
    return Json{
      {"width", width}, {"height", height}, {"distortion_model", "plumb_bob"},
      {"D", {0.1, 0.0, 0.0, 0.0, 0.0}},
      {"K", {500.0, 0.0, 2.0, 0.0, 500.0, 2.0, 0.0, 0.0, 1.0}},
      {"R", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}},
      {"P", {500.0, 0.0, 2.0, 0.0, 0.0, 500.0, 2.0, 0.0, 0.0, 0.0, 1.0, 0.0}}};
  }

  // A 4x2 frame; depth values in mm are {0, 1000, 1500, 65535, ...}.
  clink::Message frameMessage(bool with_depth, std::uint32_t info_width = 4)
  {
    const clink::Blob rgb(4 * 2 * 3, 7);
    const std::vector<std::uint16_t> depth_mm = {0, 1000, 1500, 65535, 1, 2, 0, 4000};
    clink::Blob depth_bytes;
    for (auto value : depth_mm) {
      depth_bytes.push_back(static_cast<std::uint8_t>(value & 0xFF));
      depth_bytes.push_back(static_cast<std::uint8_t>(value >> 8));
    }
    clink::Message message;
    message.header = Json{
      {"type", "frame"}, {"index", 3}, {"capture_ts_ns", 123456789012345LL},
      {"rgb", {{"width", 4}, {"height", 2}, {"encoding", "bgr8"}, {"codec", "zlib"}}},
      {"camera_info", cameraInfoJson(info_width, 2)}};
    message.blobs.push_back(zlibOf(rgb));
    if (with_depth) {
      message.header["depth"] = {{"width", 4}, {"height", 2}, {"encoding", "16UC1"}, {"codec", "zlib"}};
      message.blobs.push_back(zlibOf(depth_bytes));
    }
    return message;
  }
}  // namespace

// ---- clock_sync -------------------------------------------------------------------------

TEST(ClockSync, OffsetAndRoundTripWithKnownSkew)
{
  // Jetson clock is 5 s ahead; 10 ms each way; 1 ms service time.
  const std::int64_t skew = 5000 * kMs;
  const std::int64_t t0 = 1000 * kMs;
  const std::int64_t t1 = t0 + 10 * kMs + skew;
  const std::int64_t t2 = t1 + 1 * kMs;
  const std::int64_t t3 = t0 + 10 * kMs + 1 * kMs + 10 * kMs;
  const auto sample = csync::computeSample(t0, t1, t2, t3);
  EXPECT_EQ(sample.offset_ns, skew);
  EXPECT_EQ(sample.rtt_ns, 20 * kMs);
  EXPECT_EQ(csync::toPcTime(t2 + 7, sample.offset_ns), t2 + 7 - skew);
}

TEST(ClockSync, AsymmetricDelayErrorIsBoundedByHalfRtt)
{
  // 30 ms out, 10 ms back: the estimate is off by (30 - 10) / 2 = 10 ms = rtt/2 at most.
  const std::int64_t t0 = 0;
  const std::int64_t t1 = 30 * kMs;
  const std::int64_t t2 = 30 * kMs;
  const std::int64_t t3 = 40 * kMs;
  const auto sample = csync::computeSample(t0, t1, t2, t3);
  EXPECT_EQ(sample.rtt_ns, 40 * kMs);
  EXPECT_LE(std::abs(sample.offset_ns), sample.rtt_ns / 2);
}

TEST(ClockSync, BestSampleIsSmallestRoundTrip)
{
  std::vector<csync::ClockSample> samples(3);
  samples[0].rtt_ns = 30 * kMs;
  samples[1].rtt_ns = 8 * kMs;
  samples[2].rtt_ns = 12 * kMs;
  EXPECT_EQ(csync::bestSampleIndex(samples), 1u);
  EXPECT_THROW(csync::bestSampleIndex({}), std::invalid_argument);
}

namespace
{
  struct Batch
  {
    csync::ClockSample sample;
    int id{0};
  };

  // Returns one batch per call with the given round trips (ms); counts the calls.
  struct FakeBatches
  {
    std::vector<double> rtts_ms;
    int calls{0};
    Batch operator()()
    {
      Batch batch;
      batch.id = ++calls;
      const double rtt = rtts_ms.at(std::min<std::size_t>(calls - 1, rtts_ms.size() - 1));
      batch.sample.rtt_ns = static_cast<std::int64_t>(rtt * 1.0e6);
      return batch;
    }
  };
}  // namespace

TEST(ClockSync, NoRetryWhenFirstBatchIsFine)
{
  FakeBatches batches{{20.0, 5.0}};
  const auto outcome = csync::syncWithRetries<Batch>(std::ref(batches), 100.0, 2);
  EXPECT_EQ(batches.calls, 1);
  EXPECT_EQ(outcome.attempts, 1);
  EXPECT_EQ(outcome.result.id, 1);
  EXPECT_DOUBLE_EQ(outcome.best_rtt_ms, 20.0);
  EXPECT_TRUE(outcome.failure.empty());
}

TEST(ClockSync, SucceedsOnSecondBatchAfterSlowFirst)
{
  FakeBatches batches{{300.0, 20.0, 5.0}};
  const auto outcome = csync::syncWithRetries<Batch>(std::ref(batches), 100.0, 2);
  EXPECT_EQ(batches.calls, 2);  // stops at the first good batch
  EXPECT_EQ(outcome.attempts, 2);
  EXPECT_EQ(outcome.result.id, 2);
  EXPECT_DOUBLE_EQ(outcome.best_rtt_ms, 20.0);
  EXPECT_TRUE(outcome.failure.empty());
}

TEST(ClockSync, AbortsAfterRetriesWithBestRoundTripInMessage)
{
  FakeBatches batches{{300.0, 250.0, 400.0, 5.0}};
  const auto outcome = csync::syncWithRetries<Batch>(std::ref(batches), 100.0, 2);
  EXPECT_EQ(batches.calls, 3);  // 1 + 2 retries, the 4th (good) batch is never taken
  EXPECT_EQ(outcome.attempts, 3);
  EXPECT_EQ(outcome.result.id, 2);  // the batch with the smallest round trip is kept
  EXPECT_DOUBLE_EQ(outcome.best_rtt_ms, 250.0);
  EXPECT_EQ(
    outcome.failure,
    "Link round trip 250 ms is outside [0, 100] ms (best of 3 sync attempts)");
}

TEST(ClockSync, ZeroRetriesMeansOneAttempt)
{
  FakeBatches batches{{300.0, 5.0}};
  const auto outcome = csync::syncWithRetries<Batch>(std::ref(batches), 100.0, 0);
  EXPECT_EQ(batches.calls, 1);
  EXPECT_EQ(
    outcome.failure,
    "Link round trip 300 ms is outside [0, 100] ms (best of 1 sync attempt)");
  FakeBatches negative_retries{{300.0, 5.0}};
  EXPECT_EQ(csync::syncWithRetries<Batch>(std::ref(negative_retries), 100.0, -3).attempts, 1);
}

TEST(ClockSync, NegativeRoundTripCountsAsSlowAndIsRetried)
{
  FakeBatches batches{{-2.0, 10.0}};  // clock stepped backwards during the exchange
  const auto outcome = csync::syncWithRetries<Batch>(std::ref(batches), 100.0, 2);
  EXPECT_EQ(outcome.attempts, 2);
  EXPECT_TRUE(outcome.failure.empty());
}

TEST(ClockSync, MeasurementErrorsAreNotRetried)
{
  int calls = 0;
  const auto throwing = [&calls]() -> Batch {
      ++calls;
      throw std::runtime_error("link lost");
    };
  EXPECT_THROW(csync::syncWithRetries<Batch>(throwing, 100.0, 2), std::runtime_error);
  EXPECT_EQ(calls, 1);
}

TEST(ClockSync, ValidateLink)
{
  csync::LinkLimits limits;
  limits.max_rtt_ms = 100.0;
  limits.max_clock_offset_ms = 50.0;
  csync::ClockSample good;
  good.rtt_ns = 20 * kMs;
  EXPECT_EQ(csync::validateLink(good, true, std::nullopt, limits), "");
  EXPECT_EQ(csync::validateLink(good, true, 10.0, limits), "");

  csync::ClockSample slow;
  slow.rtt_ns = 150 * kMs;
  EXPECT_NE(csync::validateLink(slow, true, std::nullopt, limits), "");
  EXPECT_NE(csync::validateLink(good, false, std::nullopt, limits), "");
  EXPECT_NE(csync::validateLink(good, true, -80.0, limits), "");

  csync::ClockSample negative;  // clock stepped backwards during the exchange
  negative.rtt_ns = -1 * kMs;
  EXPECT_NE(csync::validateLink(negative, true, std::nullopt, limits), "");
}

TEST(ClockSync, ValidateFrameStamp)
{
  const std::int64_t goal = 100000 * kMs;
  const std::int64_t now = goal + 500 * kMs;
  const std::int64_t rtt = 20 * kMs;  // tolerance 10 ms
  const auto check = [&](std::int64_t capture, std::optional<std::int64_t> previous) {
      return csync::validateFrameStamp(capture, goal, now, rtt, previous);
    };

  EXPECT_EQ(check(goal + 100 * kMs, std::nullopt), "");
  EXPECT_EQ(check(goal - 9 * kMs, std::nullopt), "");       // within tolerance
  EXPECT_NE(check(goal - 11 * kMs, std::nullopt), "");      // stale
  EXPECT_EQ(check(now + 9 * kMs, std::nullopt), "");        // within tolerance
  EXPECT_NE(check(now + 11 * kMs, std::nullopt), "");       // from the future
  EXPECT_EQ(check(goal + 200 * kMs, goal + 100 * kMs), "");
  EXPECT_NE(check(goal + 100 * kMs, goal + 100 * kMs), ""); // duplicate
  EXPECT_NE(check(goal + 90 * kMs, goal + 100 * kMs), "");  // out of order
}

// ---- framing ----------------------------------------------------------------------------

TEST(CameraLink, EncodeUsesBigEndianLengthAndBlobSizes)
{
  const auto bytes = clink::encodeMessage(Json{{"type", "x"}}, {{1, 2, 3}, {4}});
  const std::uint32_t length = (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
  ASSERT_EQ(bytes.size(), 4u + length + 4u);
  const auto header = Json::parse(std::string(bytes.begin() + 4, bytes.begin() + 4 + length));
  EXPECT_EQ(header["version"], 1);
  EXPECT_EQ(header["blob_sizes"], Json::array({3, 1}));
  EXPECT_EQ(bytes.back(), 4);
}

TEST(CameraLink, ReceivesHeaderAndBlobs)
{
  const auto [mine, theirs] = socketPair();
  clink::CameraLink client;
  client.adopt(mine);
  writeAll(theirs, clink::encodeMessage(Json{{"type", "frame"}, {"index", 2}}, {{9, 8, 7}, {}}));
  const auto message = client.receive(1.0);
  EXPECT_EQ(message.header["type"], "frame");
  EXPECT_EQ(message.header["index"], 2);
  ASSERT_EQ(message.blobs.size(), 2u);
  EXPECT_EQ(message.blobs[0], (clink::Blob{9, 8, 7}));
  EXPECT_TRUE(message.blobs[1].empty());
  ::close(theirs);
}

TEST(CameraLink, SendIsReadableByPeer)
{
  const auto [mine, theirs] = socketPair();
  clink::CameraLink client, service;
  client.adopt(mine);
  service.adopt(theirs);
  client.send(Json::object({{"type", "ping"}, {"t0", 42}}));
  const auto message = service.receive(1.0);
  EXPECT_EQ(message.header["type"], "ping");
  EXPECT_EQ(message.header["t0"], 42);
}

TEST(CameraLink, TimeoutCancelAndClose)
{
  {
    const auto [mine, theirs] = socketPair();
    clink::CameraLink client;
    client.adopt(mine);
    EXPECT_THROW(client.receive(0.15), clink::LinkTimeout);
    ::close(theirs);
  }
  {
    const auto [mine, theirs] = socketPair();
    clink::CameraLink client;
    client.adopt(mine);
    EXPECT_THROW(client.receive(5.0, []() {return true;}), clink::LinkCancelled);
    ::close(theirs);
  }
  {
    const auto [mine, theirs] = socketPair();
    clink::CameraLink client;
    client.adopt(mine);
    ::close(theirs);
    EXPECT_THROW(client.receive(1.0), clink::LinkClosed);
  }
}

TEST(CameraLink, RejectsBadHeaders)
{
  const auto expect_error = [](const clink::Blob & bytes) {
      const auto [mine, theirs] = socketPair();
      clink::CameraLink client;
      client.adopt(mine);
      writeAll(theirs, bytes);
      EXPECT_THROW(client.receive(1.0), clink::LinkError);
      ::close(theirs);
    };
  // Wrong protocol version.
  const std::string old_header = R"({"version":2,"type":"x"})";
  clink::Blob wrong_version = {0, 0, 0, static_cast<std::uint8_t>(old_header.size())};
  wrong_version.insert(wrong_version.end(), old_header.begin(), old_header.end());
  expect_error(wrong_version);
  // Absurd header length.
  expect_error({0xFF, 0xFF, 0xFF, 0xFF});
  // Not JSON.
  const std::string junk = "not json";
  clink::Blob bad_json = {0, 0, 0, static_cast<std::uint8_t>(junk.size())};
  bad_json.insert(bad_json.end(), junk.begin(), junk.end());
  expect_error(bad_json);
}

TEST(CameraLink, ConnectsOverTcpAndExchanges)
{
  const int listener = ::socket(AF_INET, SOCK_STREAM, 0);
  ASSERT_GE(listener, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  ASSERT_EQ(::bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof(address)), 0);
  ASSERT_EQ(::listen(listener, 1), 0);
  socklen_t length = sizeof(address);
  ASSERT_EQ(::getsockname(listener, reinterpret_cast<sockaddr *>(&address), &length), 0);
  const int port = ntohs(address.sin_port);

  std::thread service([listener]() {
      const int fd = ::accept(listener, nullptr, nullptr);
      clink::CameraLink peer;
      peer.adopt(fd);
      const auto ping = peer.receive(2.0);
      peer.send(Json::object({{"type", "pong"}, {"t0", ping.header["t0"]}, {"t1", 1}, {"t2", 2}}));
    });

  clink::CameraLink client;
  client.connectTo("127.0.0.1", port, 2.0);
  EXPECT_TRUE(client.connected());
  client.send(Json::object({{"type", "ping"}, {"t0", 77}}));
  const auto pong = client.receive(2.0);
  EXPECT_EQ(pong.header["type"], "pong");
  EXPECT_EQ(pong.header["t0"], 77);
  service.join();
  ::close(listener);

  // Nothing is listening any more: connecting fails with a LinkError, not a hang.
  clink::CameraLink second;
  EXPECT_THROW(second.connectTo("127.0.0.1", port, 1.0), clink::LinkError);
}

// ---- codec and frame decoding ---------------------------------------------------------

TEST(CameraLinkCodec, ZlibRoundTripAndCorruption)
{
  clink::Blob raw(1000);
  for (std::size_t i = 0; i < raw.size(); ++i) {raw[i] = static_cast<std::uint8_t>(i % 13);}
  const auto packed = zlibOf(raw);
  EXPECT_LT(packed.size(), raw.size());
  EXPECT_EQ(clink::zlibDecompress(packed, raw.size()), raw);
  EXPECT_THROW(clink::zlibDecompress(packed, raw.size() + 1), clink::LinkError);  // wrong size
  auto corrupt = packed;
  corrupt[corrupt.size() / 2] ^= 0xFF;
  EXPECT_THROW(clink::zlibDecompress(corrupt, raw.size()), clink::LinkError);
}

TEST(CameraLinkCodec, DepthMillimetresToMetresWithNanForInvalid)
{
  const auto depth = clink::depthMmToMeters({0, 0, 0xE8, 0x03, 0xFF, 0xFF});
  ASSERT_EQ(depth.size(), 3u);
  EXPECT_TRUE(std::isnan(depth[0]));
  EXPECT_FLOAT_EQ(depth[1], 1.0f);
  EXPECT_FLOAT_EQ(depth[2], 65.535f);
}

TEST(CameraLinkCodec, DecodesRgbdFrame)
{
  const auto frame = clink::decodeFrame(frameMessage(true), true);
  EXPECT_EQ(frame.capture_ts_ns, 123456789012345LL);
  EXPECT_EQ(frame.index, 3u);
  EXPECT_EQ(frame.width, 4u);
  EXPECT_EQ(frame.height, 2u);
  EXPECT_EQ(frame.rgb_encoding, "bgr8");
  EXPECT_EQ(frame.rgb_step, 12u);
  EXPECT_EQ(frame.rgb, clink::Blob(24, 7));
  ASSERT_TRUE(frame.has_depth);
  ASSERT_EQ(frame.depth_m.size(), 8u);
  EXPECT_TRUE(std::isnan(frame.depth_m[0]));
  EXPECT_FLOAT_EQ(frame.depth_m[1], 1.0f);
  EXPECT_FLOAT_EQ(frame.depth_m[2], 1.5f);
  EXPECT_FLOAT_EQ(frame.depth_m[7], 4.0f);
  EXPECT_EQ(frame.info.distortion_model, "plumb_bob");
  EXPECT_EQ(frame.info.D.size(), 5u);
  EXPECT_DOUBLE_EQ(frame.info.K[0], 500.0);
  EXPECT_DOUBLE_EQ(frame.info.P[10], 1.0);
}

TEST(CameraLinkCodec, RgbFrameWithoutDepth)
{
  const auto frame = clink::decodeFrame(frameMessage(false), false);
  EXPECT_FALSE(frame.has_depth);
  EXPECT_THROW(clink::decodeFrame(frameMessage(false), true), clink::LinkError);
}

TEST(CameraLinkCodec, RejectsInconsistentFrames)
{
  // camera_info width differs from the image.
  EXPECT_THROW(clink::decodeFrame(frameMessage(true, 8), true), clink::LinkError);

  // Missing timestamp.
  auto no_ts = frameMessage(true);
  no_ts.header.erase("capture_ts_ns");
  EXPECT_THROW(clink::decodeFrame(no_ts, true), clink::LinkError);

  // Truncated pixel data.
  auto truncated = frameMessage(true);
  truncated.blobs[0] = zlibOf(clink::Blob(5, 0));
  EXPECT_THROW(clink::decodeFrame(truncated, true), clink::LinkError);

  // Wrong depth encoding and unsupported codec.
  auto bad_depth = frameMessage(true);
  bad_depth.header["depth"]["encoding"] = "32FC1";
  EXPECT_THROW(clink::decodeFrame(bad_depth, true), clink::LinkError);
  auto bad_codec = frameMessage(true);
  bad_codec.header["rgb"]["codec"] = "png";
  EXPECT_THROW(clink::decodeFrame(bad_codec, true), clink::LinkError);
}
