#!/usr/bin/env python3

import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def validate_static_tf_contract(parent, child, xyz, rotation_spec, dynamic_frames, children_seen):
    if not parent or not child:
        return False, "empty_parent_or_child"
    if child in dynamic_frames:
        return False, "dynamic_child_frame"
    if child in children_seen:
        return False, "duplicate_child_frame"
    if len(xyz) != 3:
        return False, "invalid_vector_length"
    try:
        [float(value) for value in xyz]
    except (TypeError, ValueError):
        return False, "non_numeric_transform"
    if len(rotation_spec) not in (3, 9):
        return False, "invalid_vector_length"
    try:
        [float(value) for value in rotation_spec]
    except (TypeError, ValueError):
        return False, "non_numeric_transform"
    return True, "ok"


def rotation_matrix_to_quaternion(rotation_matrix):
    rotation = [float(value) for value in rotation_matrix]
    r00, r01, r02, r10, r11, r12, r20, r21, r22 = rotation
    trace = r00 + r11 + r22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r21 - r12) / s
        qy = (r02 - r20) / s
        qz = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / s
        qx = 0.25 * s
        qy = (r01 + r10) / s
        qz = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / s
        qx = (r01 + r10) / s
        qy = 0.25 * s
        qz = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / s
        qx = (r02 + r20) / s
        qy = (r12 + r21) / s
        qz = 0.25 * s
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm <= 1.0e-9:
        raise ValueError("rotation matrix produced an invalid quaternion")
    return qw / norm, qx / norm, qy / norm, qz / norm


class StaticTfManager(Node):
    def __init__(self):
        super().__init__("static_tf_manager")
        self.extrinsics_file = self.declare_parameter("extrinsics_file", "").value
        self.tf_file = self.declare_parameter("tf_file", "").value
        self.base_height = float(self.declare_parameter("base_height", 0.28).value)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_once()

    def load_yaml(self, path):
        if not path:
            return {}
        file_path = Path(path)
        if not file_path.exists():
            self.get_logger().warn(f"YAML not found: {file_path}")
            return {}
        with file_path.open("r", encoding="utf-8") as handle:
            return yaml.safe_load(handle) or {}

    def make_transform(self, parent, child, xyz, rotation_spec):
        if len(xyz) != 3 or len(rotation_spec) not in (3, 9):
            raise ValueError(
                f"Invalid TF spec for {parent}->{child}: xyz must contain 3 values and rotation must contain 3 rpy or 9 matrix values"
            )
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(xyz[0])
        msg.transform.translation.y = float(xyz[1])
        msg.transform.translation.z = float(xyz[2])
        if len(rotation_spec) == 3:
            roll, pitch, yaw = [float(value) for value in rotation_spec]
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            msg.transform.rotation.w = cr * cp * cy + sr * sp * sy
            msg.transform.rotation.x = sr * cp * cy - cr * sp * sy
            msg.transform.rotation.y = cr * sp * cy + sr * cp * sy
            msg.transform.rotation.z = cr * cp * sy - sr * sp * cy
        else:
            qw, qx, qy, qz = rotation_matrix_to_quaternion(rotation_spec)
            msg.transform.rotation.w = qw
            msg.transform.rotation.x = qx
            msg.transform.rotation.y = qy
            msg.transform.rotation.z = qz
        return msg

    def publish_once(self):
        extrinsics = self.load_yaml(self.extrinsics_file).get("extrinsics", {})
        tf_tree = self.load_yaml(self.tf_file).get("tf_tree", {})
        base_frame = tf_tree.get("base_frame", "base_link")
        footprint_frame = tf_tree.get("footprint_frame", "base_footprint")
        trunk_frame = tf_tree.get("body_semantic_frame", "trunk")
        dynamic_frames = {
            tf_tree.get("map_frame", "map"),
            tf_tree.get("odom_frame", "odom"),
        }
        children_seen = set()
        transforms = []

        def add_transform(parent, child, xyz, rotation_spec):
            valid, reason = validate_static_tf_contract(
                parent, child, xyz, rotation_spec, dynamic_frames, children_seen
            )
            if not valid:
                self.get_logger().warn(f"Skipping TF {parent}->{child}: {reason}")
                return
            try:
                transforms.append(self.make_transform(parent, child, xyz, rotation_spec))
                children_seen.add(child)
            except (TypeError, ValueError) as exc:
                self.get_logger().warn(str(exc))

        add_transform(footprint_frame, base_frame, [0.0, 0.0, self.base_height], [0.0, 0.0, 0.0])
        add_transform(base_frame, trunk_frame, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

        for sensor_name, spec in extrinsics.items():
            rotation_spec = spec.get("rotation_matrix")
            if rotation_spec is None:
                rotation_spec = spec.get("rpy", [0.0, 0.0, 0.0])
            add_transform(
                spec.get("parent", base_frame),
                spec.get("child", sensor_name + "_link"),
                spec.get("xyz", [0.0, 0.0, 0.0]),
                rotation_spec,
            )

        if transforms:
            self.broadcaster.sendTransform(transforms)


def main():
    rclpy.init()
    node = StaticTfManager()
    rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
