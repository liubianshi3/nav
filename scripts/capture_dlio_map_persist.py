#!/usr/bin/env python3
"""Persistent capture of /jt128/dlio/map_points. Saves on every update."""

import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class DLIOmapCapture(Node):
    def __init__(self, output_stem):
        super().__init__('dlio_map_capture')
        self.output_stem = Path(output_stem)
        self.save_count = 0

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub = self.create_subscription(
            PointCloud2, '/jt128/dlio/map_points', self.cb, qos
        )
        self.get_logger().info('Waiting for /jt128/dlio/map_points...')

    def cb(self, msg):
        self.save_count += 1
        points = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        arr = np.array([[float(p[0]), float(p[1]), float(p[2])] for p in points], dtype=np.float32)
        output_path = self.output_stem.parent / f'{self.output_stem.stem}_v{self.save_count}.pcd'
        self.get_logger().info(
            f'Received {len(arr)} points (frame={msg.header.frame_id}), '
            f'saving to {output_path}'
        )
        with open(output_path, 'w') as f:
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
        self.get_logger().info(f'Saved ({self.save_count} total)')


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/dlio_map'
    rclpy.init()
    node = DLIOmapCapture(out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
