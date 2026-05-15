#!/usr/bin/env python3
"""Subscribe to /jt128/dlio/map_points with transient_local durability and save to PCD."""

import sys
import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class DLIOmapCapture(Node):
    def __init__(self, output_path):
        super().__init__('dlio_map_capture')
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.received = False

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub = self.create_subscription(
            PointCloud2, '/jt128/dlio/map_points', self.cb, qos
        )
        self.get_logger().info(
            f'Waiting for /jt128/dlio/map_points (transient_local)...'
        )

    def cb(self, msg):
        if self.received:
            return
        self.received = True
        self.get_logger().info(
            f'Received map_points: {msg.width}x{msg.height} points, '
            f'frame={msg.header.frame_id}'
        )
        # Read points
        points = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        arr = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
        self.get_logger().info(f'Saving {len(arr)} points to {self.output_path}')
        self._write_pcd(arr, msg.header.frame_id)
        self.get_logger().info('Done.')
        rclpy.shutdown()

    def _write_pcd(self, arr, frame_id):
        with open(self.output_path, 'w') as f:
            f.write(
                '# .PCD v0.7 - Point Cloud Data file format\n'
                'VERSION 0.7\n'
                'FIELDS x y z\n'
                'SIZE 4 4 4\n'
                'TYPE F F F\n'
                'COUNT 1 1 1\n'
                f'WIDTH {len(arr)}\n'
                'HEIGHT 1\n'
                'VIEWPOINT 0 0 0 1 0 0 0\n'
                f'POINTS {len(arr)}\n'
                'DATA ascii\n'
            )
            for p in arr:
                f.write(f'{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output', nargs='?',
                        default='/tmp/dlio_accumulated.pcd')
    args = parser.parse_args()
    rclpy.init()
    node = DLIOmapCapture(args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
