#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros

# Reuse the RPLidar implementation bundled in the previous-semester workspace.
from . import rplidar_driver as LPFL


class LidarPublisherNode(Node):
    def __init__(self):
        super().__init__('lidar_publisher_node_v2')

        self.declare_parameter('port', '/dev/lidar')
        self.declare_parameter('topic', 'lidar_raw')
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rotation_offset_deg', 180.0)
        self.declare_parameter('publish_period', 0.05)
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 12.0)

        self.port = self.get_parameter('port').value
        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.rotation_offset_deg = float(self.get_parameter('rotation_offset_deg').value)
        self.period = float(self.get_parameter('publish_period').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(LaserScan, self.topic, qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.lidar = None
        self.scan_iter = None
        self._open_lidar()
        self.timer = self.create_timer(self.period, self.publish_scan)
        self.get_logger().info(
            f'RPLidar opened: port={self.port}, topic=/{self.topic}, rotation={self.rotation_offset_deg:.1f} deg'
        )

    def _open_lidar(self):
        self._close_lidar()
        self.lidar = LPFL.RPLidar(self.port)
        self.scan_iter = self.lidar.iter_scans()

    def _close_lidar(self):
        if self.lidar is None:
            return
        try:
            self.lidar.stop()
        except Exception:
            pass
        try:
            self.lidar.stop_motor()
        except Exception:
            pass
        try:
            self.lidar.disconnect()
        except Exception:
            pass
        self.lidar = None
        self.scan_iter = None

    def _publish_tf(self, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.frame_id
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        # Identity quaternion. The original workspace left all quaternion fields at zero,
        # which is not a valid rotation.
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)

    def publish_scan(self):
        try:
            scan = next(self.scan_iter)
        except Exception as exc:
            self.get_logger().error(f'LiDAR read error: {exc}; reconnecting')
            try:
                self._open_lidar()
            except Exception as reconnect_exc:
                self.get_logger().error(f'LiDAR reconnect failed: {reconnect_exc}')
            return

        stamp = self.get_clock().now().to_msg()
        self._publish_tf(stamp)

        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = math.radians(1.0)
        msg.time_increment = 0.0
        msg.scan_time = self.period
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        ranges = [float('inf')] * 360
        intensities = [0.0] * 360
        for quality, angle_deg, distance_mm in scan:
            idx = int(round(float(angle_deg))) % 360
            dist_m = float(distance_mm) / 1000.0
            if self.range_min <= dist_m <= self.range_max:
                ranges[idx] = dist_m
                intensities[idx] = float(quality)

        msg.ranges = ranges
        msg.intensities = intensities
        msg = LPFL.rotate_lidar_data(msg, offset=int(round(self.rotation_offset_deg)) % 360)
        self.pub.publish(msg)

    def destroy_node(self):
        self._close_lidar()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'[lidar_publisher_node_v2] ERROR: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
