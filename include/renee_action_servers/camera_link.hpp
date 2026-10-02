/** @file camera_link.hpp
 *  @brief Client side of the PC <-> camera service (Jetson) link: TCP framing,
 *  zlib codec and frame decoding. No ROS dependency.
 *
 *  Wire format: each message is a uint32 big-endian header length, a UTF-8 JSON
 *  header, then binary blobs whose sizes are listed in header["blob_sizes"].
 *  Every header carries "version" (1) and "type".
 */

#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <zlib.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace renee_action_servers
{
  namespace camera_link
  {
    using Json = nlohmann::json;
    using Blob = std::vector<std::uint8_t>;

    constexpr int kProtocolVersion = 1;
    constexpr std::uint32_t kMaxHeaderBytes = 1u << 20;
    constexpr std::uint64_t kMaxBlobBytes = 512ull << 20;

    struct LinkError : std::runtime_error {using std::runtime_error::runtime_error;};
    struct LinkTimeout : LinkError {using LinkError::LinkError;};
    struct LinkClosed : LinkError {using LinkError::LinkError;};
    struct LinkCancelled : LinkError {using LinkError::LinkError;};

    struct Message
    {
      Json header;
      std::vector<Blob> blobs;
    };

    /** Serialises a message. "version" and "blob_sizes" are filled in here. */
    inline Blob encodeMessage(Json header, const std::vector<Blob> & blobs = {})
    {
      header["version"] = kProtocolVersion;
      Json sizes = Json::array();
      for (const auto & blob : blobs) {sizes.push_back(blob.size());}
      header["blob_sizes"] = sizes;
      const std::string text = header.dump();
      if (text.size() > kMaxHeaderBytes) {throw LinkError("message header too large");}

      Blob out;
      const auto length = static_cast<std::uint32_t>(text.size());
      out.push_back(static_cast<std::uint8_t>(length >> 24));
      out.push_back(static_cast<std::uint8_t>(length >> 16));
      out.push_back(static_cast<std::uint8_t>(length >> 8));
      out.push_back(static_cast<std::uint8_t>(length));
      out.insert(out.end(), text.begin(), text.end());
      for (const auto & blob : blobs) {out.insert(out.end(), blob.begin(), blob.end());}
      return out;
    }

    inline Blob zlibCompress(const std::uint8_t * data, std::size_t size, int level = 1)
    {
      uLongf bound = compressBound(static_cast<uLong>(size));
      Blob out(bound);
      if (compress2(out.data(), &bound, data, static_cast<uLong>(size), level) != Z_OK) {
        throw LinkError("zlib compression failed");
      }
      out.resize(bound);
      return out;
    }

    /** Inflates a blob and checks it has exactly the expected size. */
    inline Blob zlibDecompress(const Blob & data, std::size_t expected_size)
    {
      Blob out(expected_size);
      uLongf out_size = static_cast<uLongf>(expected_size);
      const int status = uncompress(
        out.data(), &out_size, data.data(), static_cast<uLong>(data.size()));
      if (status != Z_OK || out_size != expected_size) {
        throw LinkError("zlib blob is corrupt or has an unexpected size");
      }
      return out;
    }

    /** Blocking TCP connection with deadline-based reads that can be cancelled. */
    class CameraLink
    {
      public:
        using CancelFn = std::function<bool()>;

        CameraLink() = default;
        CameraLink(const CameraLink &) = delete;
        CameraLink & operator=(const CameraLink &) = delete;
        ~CameraLink() {close();}

        bool connected() const {return fd_ >= 0;}

        /** Takes ownership of an already connected socket (used by tests). */
        void adopt(int fd)
        {
          close();
          fd_ = fd;
        }

        void close()
        {
          if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
          }
        }

        void connectTo(const std::string & host, int port, double timeout_sec)
        {
          close();
          addrinfo hints{};
          hints.ai_family = AF_UNSPEC;
          hints.ai_socktype = SOCK_STREAM;
          addrinfo * results = nullptr;
          const int rc = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &results);
          if (rc != 0) {throw LinkError("cannot resolve " + host + ": " + gai_strerror(rc));}

          std::string last_error = "no address";
          for (addrinfo * entry = results; entry != nullptr; entry = entry->ai_next) {
            const int fd = ::socket(entry->ai_family, entry->ai_socktype, entry->ai_protocol);
            if (fd < 0) {last_error = std::strerror(errno); continue;}
            if (connectWithTimeout(fd, entry, timeout_sec, last_error)) {
              freeaddrinfo(results);
              int one = 1;
              setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
              fd_ = fd;
              return;
            }
            ::close(fd);
          }
          freeaddrinfo(results);
          throw LinkError(
            "cannot connect to " + host + ":" + std::to_string(port) + ": " + last_error);
        }

        void send(const Json & header, const std::vector<Blob> & blobs = {})
        {
          if (fd_ < 0) {throw LinkClosed("link is not connected");}
          const Blob bytes = encodeMessage(header, blobs);
          std::size_t sent = 0;
          while (sent < bytes.size()) {
            const ssize_t n = ::send(fd_, bytes.data() + sent, bytes.size() - sent, MSG_NOSIGNAL);
            if (n < 0) {
              if (errno == EINTR) {continue;}
              throw LinkError(std::string("send failed: ") + std::strerror(errno));
            }
            sent += static_cast<std::size_t>(n);
          }
        }

        /** Reads one whole message before timeout_sec elapses. After any exception the
         *  stream may be desynchronised and the link should be closed. */
        Message receive(double timeout_sec, const CancelFn & cancel = nullptr)
        {
          if (fd_ < 0) {throw LinkClosed("link is not connected");}
          const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(timeout_sec));

          std::uint8_t prefix[4];
          readExact(prefix, 4, deadline, cancel);
          const std::uint32_t length = (static_cast<std::uint32_t>(prefix[0]) << 24) |
            (static_cast<std::uint32_t>(prefix[1]) << 16) |
            (static_cast<std::uint32_t>(prefix[2]) << 8) | static_cast<std::uint32_t>(prefix[3]);
          if (length == 0 || length > kMaxHeaderBytes) {
            throw LinkError("invalid message header length " + std::to_string(length));
          }

          std::string text(length, '\0');
          readExact(reinterpret_cast<std::uint8_t *>(&text[0]), length, deadline, cancel);

          Message message;
          try {
            message.header = Json::parse(text);
          } catch (const Json::exception & exception) {
            throw LinkError(std::string("invalid JSON header: ") + exception.what());
          }
          if (!message.header.is_object() || message.header.value("version", 0) != kProtocolVersion) {
            throw LinkError("unsupported protocol version or malformed header");
          }
          if (!message.header.contains("type") || !message.header["type"].is_string()) {
            throw LinkError("message header has no type");
          }

          if (message.header.contains("blob_sizes")) {
            const auto & sizes = message.header["blob_sizes"];
            if (!sizes.is_array()) {throw LinkError("blob_sizes is not an array");}
            for (const auto & size : sizes) {
              if (!size.is_number_unsigned() || size.get<std::uint64_t>() > kMaxBlobBytes) {
                throw LinkError("invalid blob size");
              }
              Blob blob(size.get<std::size_t>());
              readExact(blob.data(), blob.size(), deadline, cancel);
              message.blobs.push_back(std::move(blob));
            }
          }
          return message;
        }

      private:
        using Deadline = std::chrono::steady_clock::time_point;

        static bool connectWithTimeout(
          int fd, const addrinfo * entry, double timeout_sec, std::string & error)
        {
          const int flags = fcntl(fd, F_GETFL, 0);
          fcntl(fd, F_SETFL, flags | O_NONBLOCK);
          int rc = ::connect(fd, entry->ai_addr, entry->ai_addrlen);
          if (rc < 0 && errno != EINPROGRESS) {error = std::strerror(errno); return false;}
          if (rc < 0) {
            pollfd waiting{fd, POLLOUT, 0};
            do {
              rc = poll(&waiting, 1, static_cast<int>(timeout_sec * 1000.0));
            } while (rc < 0 && errno == EINTR);
            if (rc == 0) {error = "connection timed out"; return false;}
            if (rc < 0) {error = std::strerror(errno); return false;}
            int socket_error = 0;
            socklen_t length = sizeof(socket_error);
            getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &length);
            if (socket_error != 0) {error = std::strerror(socket_error); return false;}
          }
          fcntl(fd, F_SETFL, flags);
          return true;
        }

        void readExact(
          std::uint8_t * buffer, std::size_t size, const Deadline & deadline, const CancelFn & cancel)
        {
          std::size_t received = 0;
          while (received < size) {
            if (cancel && cancel()) {throw LinkCancelled("cancelled");}
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
              deadline - std::chrono::steady_clock::now()).count();
            if (remaining <= 0) {throw LinkTimeout("timed out waiting for the camera service");}
            pollfd waiting{fd_, POLLIN, 0};
            const int rc = poll(&waiting, 1, static_cast<int>(std::min<long>(remaining, 100)));
            if (rc < 0) {
              if (errno == EINTR) {continue;}
              throw LinkError(std::string("poll failed: ") + std::strerror(errno));
            }
            if (rc == 0) {continue;}
            const ssize_t n = ::recv(fd_, buffer + received, size - received, 0);
            if (n == 0) {throw LinkClosed("camera service closed the connection");}
            if (n < 0) {
              if (errno == EINTR) {continue;}
              throw LinkError(std::string("recv failed: ") + std::strerror(errno));
            }
            received += static_cast<std::size_t>(n);
          }
        }

        int fd_{-1};
    };

    struct CameraInfoData
    {
      std::uint32_t width{0}, height{0};
      std::string distortion_model;
      std::vector<double> D;
      std::array<double, 9> K{}, R{};
      std::array<double, 12> P{};
    };

    struct DecodedFrame
    {
      std::int64_t capture_ts_ns{0};  // Jetson clock
      std::uint32_t index{0};
      std::uint32_t width{0}, height{0};
      std::string rgb_encoding;
      std::uint32_t rgb_step{0};
      Blob rgb;
      bool has_depth{false};
      std::vector<float> depth_m;  // NaN where the camera reported no depth
      CameraInfoData info;
    };

    inline std::size_t bytesPerPixel(const std::string & encoding)
    {
      if (encoding == "bgr8" || encoding == "rgb8") {return 3;}
      if (encoding == "bgra8" || encoding == "rgba8") {return 4;}
      if (encoding == "mono8") {return 1;}
      throw LinkError("unsupported RGB encoding '" + encoding + "'");
    }

    /** uint16 millimetres (little endian) to float metres; 0 means invalid. */
    inline std::vector<float> depthMmToMeters(const Blob & little_endian_u16)
    {
      std::vector<float> out(little_endian_u16.size() / 2);
      for (std::size_t i = 0; i < out.size(); ++i) {
        const std::uint16_t mm = static_cast<std::uint16_t>(
          little_endian_u16[2 * i] | (little_endian_u16[2 * i + 1] << 8));
        out[i] = mm == 0 ? std::numeric_limits<float>::quiet_NaN() : static_cast<float>(mm) / 1000.0f;
      }
      return out;
    }

    template<typename ArrayT>
    inline void readFixedArray(const Json & source, const char * key, ArrayT & destination)
    {
      if (!source.contains(key) || !source[key].is_array() ||
        source[key].size() != destination.size())
      {
        throw LinkError(std::string("camera_info.") + key + " has the wrong size");
      }
      for (std::size_t i = 0; i < destination.size(); ++i) {
        destination[i] = source[key][i].get<double>();
      }
    }

    /** Decodes a "frame" message. Throws LinkError on any protocol violation. */
    inline DecodedFrame decodeFrame(const Message & message, bool need_depth)
    {
      try {
        const Json & header = message.header;
        DecodedFrame frame;
        frame.capture_ts_ns = header.at("capture_ts_ns").get<std::int64_t>();
        frame.index = header.value("index", 0u);

        const Json & rgb = header.at("rgb");
        frame.width = rgb.at("width").get<std::uint32_t>();
        frame.height = rgb.at("height").get<std::uint32_t>();
        frame.rgb_encoding = rgb.at("encoding").get<std::string>();
        if (rgb.value("codec", "zlib") != "zlib") {throw LinkError("unsupported RGB codec");}
        const std::size_t pixel_bytes = bytesPerPixel(frame.rgb_encoding);
        frame.rgb_step = static_cast<std::uint32_t>(frame.width * pixel_bytes);
        if (message.blobs.empty()) {throw LinkError("frame has no RGB blob");}
        frame.rgb = zlibDecompress(message.blobs[0], frame.rgb_step * frame.height);

        if (header.contains("depth")) {
          const Json & depth = header["depth"];
          if (depth.at("encoding").get<std::string>() != "16UC1") {
            throw LinkError("unsupported depth encoding");
          }
          if (depth.value("codec", "zlib") != "zlib") {throw LinkError("unsupported depth codec");}
          if (depth.at("width").get<std::uint32_t>() != frame.width ||
            depth.at("height").get<std::uint32_t>() != frame.height)
          {
            throw LinkError("depth and RGB sizes differ");
          }
          if (message.blobs.size() < 2) {throw LinkError("frame has no depth blob");}
          frame.depth_m = depthMmToMeters(
            zlibDecompress(message.blobs[1], static_cast<std::size_t>(frame.width) * frame.height * 2));
          frame.has_depth = true;
        } else if (need_depth) {
          throw LinkError("rgbd capture requested but the frame has no depth");
        }

        const Json & info = header.at("camera_info");
        frame.info.width = info.at("width").get<std::uint32_t>();
        frame.info.height = info.at("height").get<std::uint32_t>();
        frame.info.distortion_model = info.value("distortion_model", "plumb_bob");
        frame.info.D = info.at("D").get<std::vector<double>>();
        readFixedArray(info, "K", frame.info.K);
        readFixedArray(info, "R", frame.info.R);
        readFixedArray(info, "P", frame.info.P);
        if (frame.info.width != frame.width || frame.info.height != frame.height) {
          throw LinkError("camera_info size does not match the image");
        }
        return frame;
      } catch (const Json::exception & exception) {
        throw LinkError(std::string("malformed frame message: ") + exception.what());
      }
    }
  }  // namespace camera_link
}  // namespace renee_action_servers
