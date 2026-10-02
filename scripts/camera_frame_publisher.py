#!/usr/bin/env python3
"""Manual end-to-end / demo publisher for the camera link (not part of the capture stack).

Requests one RGB-D frame from /capture_camera_frames every period_sec and publishes it,
so the real ZED can be watched in rqt without writing a client.

Topics (reliable QoS, depth 5, so rqt_image_view works with either QoS choice):
  /camera_frame/rgb/image_raw    sensor_msgs/Image (as returned by the server)
  /camera_frame/depth/image_raw  sensor_msgs/Image (32FC1, metres, NaN = invalid)
  /camera_frame/camera_info      sensor_msgs/CameraInfo (left optical frame)

Small previews for viewers (rqt_image_view subscribes best-effort, and DDS over loopback
drops full-size 1280x720 images that exceed the kernel socket buffer; a preview_width
(default 640) bgr8 image fits). Set preview_width to 0 to disable:
  /camera_frame/rgb/preview      sensor_msgs/Image (bgr8, downscaled)
  /camera_frame/depth/preview    sensor_msgs/Image (bgr8, JET colour map over
                                 preview_min_m..preview_max_m, invalid = black)

Goals never overlap: the next one starts at the next period tick after the previous
finished, or immediately if the previous one took longer than the period. A failed
cycle is logged and the loop keeps going.
"""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from renee_action_servers.action import CaptureCameraFrames
from sensor_msgs.msg import CameraInfo, Image


class CameraFramePublisher(Node):
    def __init__(self):
        super().__init__('camera_frame_publisher')
        self.period = self.declare_parameter('period_sec', 3.0).value
        self.mode = self.declare_parameter('mode', 'rgbd').value
        self.action_name = self.declare_parameter('action_name', '/capture_camera_frames').value
        self.prefix = self.declare_parameter('topic_prefix', '/camera_frame').value
        self.timeout = self.declare_parameter('goal_timeout_sec', 20.0).value

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.rgb_pub = self.create_publisher(Image, f'{self.prefix}/rgb/image_raw', qos)
        self.depth_pub = self.create_publisher(Image, f'{self.prefix}/depth/image_raw', qos)
        self.info_pub = self.create_publisher(CameraInfo, f'{self.prefix}/camera_info', qos)
        self.preview_width = int(self.declare_parameter('preview_width', 640).value)
        self.preview_min = float(self.declare_parameter('preview_min_m', 0.3).value)
        self.preview_max = float(self.declare_parameter('preview_max_m', 8.0).value)
        self.rgb_preview_pub = self.create_publisher(Image, f'{self.prefix}/rgb/preview', qos)
        self.depth_preview_pub = self.create_publisher(Image, f'{self.prefix}/depth/preview', qos)

        self.client = ActionClient(self, CaptureCameraFrames, self.action_name)
        self.in_flight = False
        self.next_start = time.monotonic()
        self.started = 0.0
        self.cycle = 0
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f'requesting a {self.mode} frame every {self.period:.1f} s from {self.action_name}')

    def tick(self):
        if self.in_flight or time.monotonic() < self.next_start:
            return
        if not self.client.server_is_ready():
            if self.cycle == 0 or int(time.monotonic()) % 10 == 0:
                self.get_logger().warn('action server not available yet')
            self.next_start = time.monotonic() + 1.0
            return
        goal = CaptureCameraFrames.Goal()
        goal.mode = self.mode
        goal.num_frames = 1
        goal.timeout_sec = float(self.timeout)
        self.in_flight = True
        self.cycle += 1
        self.started = time.monotonic()
        self.client.send_goal_async(goal).add_done_callback(self.on_goal_response)

    def finish_cycle(self):
        self.in_flight = False
        self.next_start = max(self.started + self.period, time.monotonic())

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'#{self.cycle} goal rejected (another capture running?)')
            self.finish_cycle()
            return
        handle.get_result_async().add_done_callback(self.on_result)

    def publish_previews(self, result):
        """Downscaled bgr8 copies of the frame (cv2/numpy are imported lazily: optional)."""
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.get_logger().warn('cv2/numpy unavailable: previews disabled')
            self.preview_width = 0
            return
        rgb = result.rgb
        height = max(1, rgb.height * self.preview_width // rgb.width)
        img = np.frombuffer(rgb.data, dtype=np.uint8).reshape(rgb.height, rgb.width, -1)
        if rgb.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        small = cv2.resize(img, (self.preview_width, height), interpolation=cv2.INTER_AREA)
        self.rgb_preview_pub.publish(self.to_image(small, rgb.header))
        if self.mode != 'rgbd' or not result.depth.data:
            return
        depth = np.frombuffer(result.depth.data, dtype=np.float32).reshape(
            result.depth.height, result.depth.width)
        valid = np.isfinite(depth)
        scaled = np.clip((np.nan_to_num(depth, nan=0.0) - self.preview_min) /
                         (self.preview_max - self.preview_min), 0.0, 1.0)
        color = cv2.applyColorMap((scaled * 255).astype(np.uint8), cv2.COLORMAP_JET)
        color[~valid] = 0
        color = cv2.resize(color, (self.preview_width, height), interpolation=cv2.INTER_NEAREST)
        self.depth_preview_pub.publish(self.to_image(color, result.depth.header))

    @staticmethod
    def to_image(bgr, header):
        msg = Image()
        msg.header = header
        msg.height, msg.width = bgr.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = bgr.tobytes()
        return msg

    def on_result(self, future):
        elapsed = time.monotonic() - self.started
        wrapped = future.result()
        result = wrapped.result
        stamp = result.stamp.sec + result.stamp.nanosec * 1e-9
        age = time.time() - stamp if stamp > 0 else float('nan')
        summary = (f'#{self.cycle} goal->result {elapsed:.2f} s, stamp age {age:.2f} s, '
                   f'clock offset {result.clock_offset_ms:.1f} ms, rtt {result.round_trip_ms:.1f} ms, '
                   f'timestamps_valid={result.timestamps_valid}')
        if result.success and result.captured_frames == 1:
            self.rgb_pub.publish(result.rgb)
            if self.mode == 'rgbd':
                self.depth_pub.publish(result.depth)
            self.info_pub.publish(result.camera_info)
            if self.preview_width > 0:
                self.publish_previews(result)
            self.get_logger().info(f'{summary}: published ({result.rgb.width}x{result.rgb.height} '
                                   f'{result.rgb.encoding})')
        else:
            self.get_logger().warn(f'{summary}: FAILED status={wrapped.status}: {result.message}')
        self.finish_cycle()


def main():
    rclpy.init()
    node = CameraFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
