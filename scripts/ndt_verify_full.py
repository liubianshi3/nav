#!/usr/bin/env python3
"""
Full NDT verification: activate, send initial pose, wait, and report all outputs.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseWithCovarianceStamped
from autoware_internal_debug_msgs.msg import Float32Stamped, Int32Stamped
from std_msgs.msg import String
import time
import sys


class NDTVerifier(Node):
    def __init__(self):
        super().__init__('ndt_verifier')
        self.results = {}
        self.done = False

        # Subscribers for NDT outputs
        self.create_subscription(Float32Stamped, '/transform_probability', self.on_score, 10)
        self.create_subscription(Float32Stamped, '/nearest_voxel_transformation_likelihood', self.on_likelihood, 10)
        self.create_subscription(Int32Stamped, '/iteration_num', self.on_iteration, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/ndt_pose_with_covariance', self.on_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/a2/relocalization/pose', self.on_reloc_pose, 10)
        self.create_subscription(String, '/a2/localization/status', self.on_localization, 10)
        self.create_subscription(String, '/a2/ndt/health_status', self.on_health, 10)
        self.create_subscription(String, '/a2/relocalization/status', self.on_adapter_status, 10)

    def on_score(self, msg):
        self.results['transform_probability'] = msg.data
    def on_likelihood(self, msg):
        self.results['nearest_voxel_transformation_likelihood'] = msg.data
    def on_iteration(self, msg):
        self.results['iteration_num'] = msg.data
    def on_pose(self, msg):
        self.results['ndt_pose_with_covariance'] = msg
    def on_reloc_pose(self, msg):
        self.results['a2/relocalization/pose'] = msg
    def on_localization(self, msg):
        self.results['a2/localization/status'] = msg.data
    def on_health(self, msg):
        self.results['a2/ndt/health_status'] = msg.data
    def on_adapter_status(self, msg):
        self.results['a2/relocalization/status'] = msg.data


def main():
    rclpy.init()
    verifier = NDTVerifier()
    executor = SingleThreadedExecutor()
    executor.add_node(verifier)

    # Step 1: Activate NDT
    print("\n=== Step 1: Activating NDT via /trigger_node_srv ===")
    cli = verifier.create_client(SetBool, '/trigger_node_srv')
    if not cli.wait_for_service(timeout_sec=5.0):
        print("ERROR: /trigger_node_srv not available")
        sys.exit(1)
    req = SetBool.Request()
    req.data = True
    future = cli.call_async(req)
    executor.spin_until_future_complete(future, timeout_sec=5.0)
    if future.done() and future.result().success:
        print("ACTIVATION: success=True")
    else:
        print(f"ACTIVATION: success={future.result().success if future.done() else 'timeout'}")

    # Step 2: Send initial pose
    print("\n=== Step 2: Sending initial pose ===")
    pose_pub = verifier.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    time.sleep(0.5)
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = verifier.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = 3.94
    msg.pose.pose.position.y = -7.42
    msg.pose.pose.orientation.w = 1.0
    cov = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
    msg.pose.covariance = [float(v) for v in cov]
    for i in range(5):
        pose_pub.publish(msg)
        time.sleep(0.1)
    print(f"Published initial pose at ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")

    # Step 3: Wait and collect results
    print("\n=== Step 3: Waiting 30s for NDT convergence ===")
    for i in range(30):
        executor.spin_once(timeout_sec=0.1)
        time.sleep(1)
        if i % 5 == 0:
            print(f"  ... waited {i+1}s")
        if verifier.results.get('ndt_pose_with_covariance') is not None:
            print(f"  -> NDT pose received at {i+1}s!")
            break

    # Step 4: Report
    print("\n============ NDT VERIFICATION RESULTS ============")
    checks = [
        ('transform_probability', 'Score'),
        ('nearest_voxel_transformation_likelihood', 'Likelihood'),
        ('iteration_num', 'Iterations'),
        ('ndt_pose_with_covariance', 'NDT pose'),
        ('a2/relocalization/pose', 'Reloc pose (adapter)'),
        ('a2/relocalization/status', 'Adapter status'),
        ('a2/ndt/health_status', 'NDT health'),
        ('a2/localization/status', 'Localization gate'),
    ]
    all_ok = True
    for key, label in checks:
        val = verifier.results.get(key)
        if val is not None:
            if isinstance(val, PoseWithCovarianceStamped):
                p = val.pose.pose.position
                print(f"  ✅ {label}: position=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
            elif isinstance(val, float):
                print(f"  ✅ {label}: {val:.4f}")
            elif isinstance(val, int):
                print(f"  ✅ {label}: {val}")
            else:
                print(f"  ✅ {label}: {val}")
        else:
            print(f"  ❌ {label}: NOT RECEIVED")
            all_ok = False

    print("\n=== CONCLUSION ===")
    if (verifier.results.get('transform_probability') is not None and
            verifier.results.get('iteration_num') is not None):
        print(f"  NDT score: {verifier.results.get('transform_probability', 'N/A')}")
        print(f"  NDT iterations: {verifier.results.get('iteration_num', 'N/A')}")
    if verifier.results.get('ndt_pose_with_covariance') is not None:
        print("  ✅ NDT IS PUBLISHING POSE")
    else:
        print("  ❌ NDT IS NOT PUBLISHING POSE")
    if verifier.results.get('a2/localization/status') is not None:
        status = verifier.results['a2/localization/status']
        print(f"  Localization gate: {status}")
        if 'ready=true' in status:
            print("  ✅ LOCALIZATION READY")
        else:
            print("  ❌ Localization not ready")
    else:
        print("  ❌ LOCALIZATION STATUS NOT RECEIVED")

    verifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
