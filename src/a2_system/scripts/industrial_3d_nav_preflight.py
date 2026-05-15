#!/usr/bin/env python3
import argparse
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import time

class PreflightNode(Node):
    def __init__(self):
        super().__init__('industrial_3d_nav_preflight')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.topics = {
            '/jt128/front/points': PointCloud2,
            '/jt128/front/imu': Imu,
            '/jt128/dlio/odom': Odometry,
            '/a2/map/pointcloud_3d': PointCloud2,
            '/a2/relocalization/status': String,
            '/a2/localization_ok': Bool,
            '/a2/obstacle/points': PointCloud2,
            '/a2/traversability': OccupancyGrid,
            '/a2/traversability/obstacle_points': PointCloud2,
            '/a2/safety/status': String,
        }
        self.data = {t: None for t in self.topics}
        self.stamps = {t: 0 for t in self.topics}
        
        self.subs = []
        for t, msg_type in self.topics.items():
            cb = lambda msg, topic=t: self.on_msg(topic, msg)
            self.subs.append(self.create_subscription(msg_type, t, cb, 10))

    def on_msg(self, topic, msg):
        self.data[topic] = msg
        self.stamps[topic] = time.time()

    def check_tf(self, target, source):
        try:
            self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            return True, "ok"
        except Exception as e:
            return False, str(e)

def main():
    parser = argparse.ArgumentParser(description="Check industrial 3D navigation topic and TF readiness.")
    parser.add_argument("--output", default="preflight_3d.json", help="Machine-readable JSON output path")
    parser.add_argument("--timeout-sec", type=float, default=5.0, help="Topic collection timeout")
    args = parser.parse_args()

    rclpy.init()
    node = PreflightNode()
    start_time = time.time()
    timeout = max(0.5, float(args.timeout_sec))
    
    print(f"Collecting topic data (timeout {timeout:.1f}s)...")
    while rclpy.ok() and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(node.stamps[t] > 0 for t in node.topics):
            break

    results = []
    report = {"timestamp": time.time(), "items": []}
    
    # Topic checks
    for t in node.topics:
        fresh = (time.time() - node.stamps[t]) < 2.0 if node.stamps[t] > 0 else False
        status = "PASS" if fresh else "FAIL"
        reason = "ok" if fresh else ("stale" if node.stamps[t] > 0 else "missing")
        
        # Extra logic for status strings
        if t == '/a2/relocalization/status' and node.data[t]:
            if "ready=true" not in node.data[t].data.lower():
                status = "FAIL"
                reason = "ready=false"
        if t == '/a2/localization_ok' and node.data[t]:
            if not node.data[t].data:
                status = "FAIL"
                reason = "data=false"
        if t == '/a2/safety/status' and node.data[t]:
            if "ready=true" not in node.data[t].data.lower():
                status = "FAIL"
                reason = "ready=false"

        report["items"].append({"type": "topic", "name": t, "status": status, "reason": reason})

    # TF checks
    tf_pairs = [("map", "odom"), ("odom", "base_link"), ("base_link", "jt128_front_link")]
    for target, source in tf_pairs:
        ok, reason = node.check_tf(target, source)
        status = "PASS" if ok else "FAIL"
        report["items"].append({"type": "tf", "name": f"{target}->{source}", "status": status, "reason": reason})

    # Output Markdown
    print("\n=== Industrial 3D Navigation Preflight Report ===\n")
    print("| Category | Component | Status | Detail |")
    print("|---|---|---|---|")
    for item in report["items"]:
        print(f"| {item['type']} | {item['name']} | {item['status']} | {item['reason']} |")
    
    # Machine readable JSON
    with open(args.output, "w") as f:
        json.dump(report, f, indent=2)
    
    final_status = "PASS" if all(i["status"] == "PASS" for i in report["items"]) else "FAIL"
    print(f"\nOVERALL STATUS: {final_status}")
    
    node.destroy_node()
    rclpy.shutdown()
    return 0 if final_status == "PASS" else 1

if __name__ == '__main__':
    raise SystemExit(main())
