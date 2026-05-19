#!/usr/bin/env python3
"""Publish lightweight PointCloud2 preview topics for RViz/Web visualization."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

try:  # pragma: no cover - exercised on ROS hosts
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
except ImportError:  # pragma: no cover - lets pure helper tests run without ROS
    rclpy = None
    Node = object  # type: ignore[assignment,misc]
    PointCloud2 = None  # type: ignore[assignment]
    PointField = None  # type: ignore[assignment]
    DurabilityPolicy = HistoryPolicy = QoSProfile = ReliabilityPolicy = None  # type: ignore[assignment]


POINT_FIELD_INT8 = 1
POINT_FIELD_UINT8 = 2
POINT_FIELD_INT16 = 3
POINT_FIELD_UINT16 = 4
POINT_FIELD_INT32 = 5
POINT_FIELD_UINT32 = 6
POINT_FIELD_FLOAT32 = 7
POINT_FIELD_FLOAT64 = 8

_NUMPY_DTYPE_BY_POINT_FIELD = {
    POINT_FIELD_INT8: np.int8,
    POINT_FIELD_UINT8: np.uint8,
    POINT_FIELD_INT16: np.int16,
    POINT_FIELD_UINT16: np.uint16,
    POINT_FIELD_INT32: np.int32,
    POINT_FIELD_UINT32: np.uint32,
    POINT_FIELD_FLOAT32: np.float32,
    POINT_FIELD_FLOAT64: np.float64,
}


def preview_topic_name(input_topic: str) -> str:
    topic = str(input_topic or "").strip().rstrip("/")
    if not topic:
        return "/points_preview"
    if topic.endswith("_preview"):
        return topic
    return f"{topic}_preview"


def should_publish_preview(last_publish_ns: int | None, now_ns: int, preview_rate_hz: float) -> bool:
    if last_publish_ns is None:
        return True
    if not math.isfinite(preview_rate_hz) or preview_rate_hz <= 0.0:
        return True
    period_ns = int(1_000_000_000.0 / preview_rate_hz)
    return now_ns - last_publish_ns >= period_ns


def prepare_preview_points(
    points: np.ndarray,
    *,
    voxel_size_m: float,
    min_range_m: float,
    max_range_m: float,
    max_points: int,
) -> np.ndarray:
    if points.size == 0:
        return np.empty((0, points.shape[1] if points.ndim == 2 else 3), dtype=np.float32)

    preview = np.asarray(points, dtype=np.float32)
    if preview.ndim != 2 or preview.shape[1] < 3:
        return np.empty((0, 3), dtype=np.float32)

    xyz = preview[:, :3]
    finite_mask = np.isfinite(xyz).all(axis=1)
    if min_range_m > 0.0 or max_range_m > 0.0:
        ranges = np.linalg.norm(xyz, axis=1)
        if min_range_m > 0.0:
            finite_mask &= ranges >= float(min_range_m)
        if max_range_m > 0.0:
            finite_mask &= ranges <= float(max_range_m)
    preview = preview[finite_mask]

    if preview.shape[0] == 0:
        return np.empty((0, points.shape[1]), dtype=np.float32)

    if math.isfinite(voxel_size_m) and voxel_size_m > 0.0 and preview.shape[0] > 1:
        voxel_keys = np.floor(preview[:, :3] / float(voxel_size_m)).astype(np.int64)
        _, unique_indices = np.unique(voxel_keys, axis=0, return_index=True)
        unique_indices.sort()
        preview = preview[unique_indices]

    if max_points > 0 and preview.shape[0] > max_points:
        indices = np.linspace(0, preview.shape[0] - 1, int(max_points), dtype=np.int64)
        preview = preview[indices]

    return np.ascontiguousarray(preview, dtype=np.float32)


def _field_by_name(msg: Any, name: str) -> Any | None:
    for field in getattr(msg, "fields", []):
        if getattr(field, "name", "") == name:
            return field
    return None


def _dtype_for_field(field: Any, is_bigendian: bool) -> np.dtype:
    base = _NUMPY_DTYPE_BY_POINT_FIELD.get(int(getattr(field, "datatype", 0)))
    if base is None:
        raise ValueError(f"unsupported field datatype for {getattr(field, 'name', 'unknown')}")
    dtype = np.dtype(base)
    if dtype.itemsize > 1:
        dtype = dtype.newbyteorder(">" if is_bigendian else "<")
    return dtype


def _field_array(msg: Any, field: Any) -> np.ndarray:
    width = int(getattr(msg, "width", 0))
    height = int(getattr(msg, "height", 1) or 1)
    point_step = int(getattr(msg, "point_step", 0))
    row_step = int(getattr(msg, "row_step", width * point_step))
    if width <= 0 or height <= 0 or point_step <= 0:
        return np.empty((0,), dtype=np.float32)

    dtype = _dtype_for_field(field, bool(getattr(msg, "is_bigendian", False)))
    offset = int(getattr(field, "offset", 0))
    count = width * height
    expected_compact_row_step = width * point_step
    buffer = getattr(msg, "data", b"")

    if height == 1 or row_step == expected_compact_row_step:
        return np.ndarray(
            shape=(count,),
            dtype=dtype,
            buffer=buffer,
            offset=offset,
            strides=(point_step,),
        )

    rows = []
    for row_index in range(height):
        rows.append(
            np.ndarray(
                shape=(width,),
                dtype=dtype,
                buffer=buffer,
                offset=row_index * row_step + offset,
                strides=(point_step,),
            )
        )
    return np.concatenate(rows)


def extract_preview_points(msg: Any, *, include_intensity: bool) -> np.ndarray:
    x_field = _field_by_name(msg, "x")
    y_field = _field_by_name(msg, "y")
    z_field = _field_by_name(msg, "z")
    if x_field is None or y_field is None or z_field is None:
        raise ValueError("PointCloud2 is missing x/y/z fields")

    arrays = [
        _field_array(msg, x_field).astype(np.float32, copy=False),
        _field_array(msg, y_field).astype(np.float32, copy=False),
        _field_array(msg, z_field).astype(np.float32, copy=False),
    ]
    if include_intensity:
        intensity_field = _field_by_name(msg, "intensity")
        if intensity_field is not None:
            arrays.append(_field_array(msg, intensity_field).astype(np.float32, copy=False))

    lengths = [array.shape[0] for array in arrays]
    if not lengths or min(lengths) == 0:
        return np.empty((0, len(arrays)), dtype=np.float32)
    sample_count = min(lengths)
    return np.column_stack([array[:sample_count] for array in arrays]).astype(np.float32, copy=False)


def build_preview_cloud(source_msg: Any, points: np.ndarray) -> Any:
    if PointCloud2 is None or PointField is None:
        raise RuntimeError("ROS sensor_msgs is not available")

    output = PointCloud2()
    output.header = source_msg.header
    output.height = 1
    output.width = int(points.shape[0])
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    if points.shape[1] >= 4:
        fields.append(PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1))
    output.fields = fields
    output.is_bigendian = False
    output.point_step = 4 * len(fields)
    output.row_step = output.point_step * output.width
    output.is_dense = True
    output.data = np.ascontiguousarray(points[:, : len(fields)], dtype=np.float32).tobytes()
    return output


def _make_qos(reliability: str) -> Any:
    if QoSProfile is None:
        raise RuntimeError("ROS QoSProfile is not available")
    reliability_policy = ReliabilityPolicy.BEST_EFFORT
    if str(reliability).strip().lower() == "reliable":
        reliability_policy = ReliabilityPolicy.RELIABLE
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=reliability_policy,
        durability=DurabilityPolicy.VOLATILE,
    )


class PointCloudPreviewNode(Node):  # type: ignore[misc]
    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError("rclpy is not available")
        super().__init__("pointcloud_preview_node")

        self.input_topic = str(self.declare_parameter("input_topic", "/jt128/front/points").value)
        default_output = preview_topic_name(self.input_topic)
        self.output_topic = str(self.declare_parameter("output_topic", default_output).value)
        self.preview_rate_hz = float(self.declare_parameter("preview_rate_hz", 5.0).value)
        self.voxel_size_m = float(self.declare_parameter("voxel_size_m", 0.05).value)
        self.min_range_m = float(self.declare_parameter("min_range_m", 0.0).value)
        self.max_range_m = float(self.declare_parameter("max_range_m", 0.0).value)
        self.max_points = int(self.declare_parameter("max_points", 30000).value)
        self.include_intensity = bool(self.declare_parameter("include_intensity", True).value)
        qos_reliability = str(self.declare_parameter("qos_reliability", "best_effort").value)

        qos = _make_qos(qos_reliability)
        self._publisher = self.create_publisher(PointCloud2, self.output_topic, qos)
        self.create_subscription(PointCloud2, self.input_topic, self._on_cloud, qos)
        self._last_publish_ns: int | None = None
        self._input_frames = 0
        self._published_frames = 0

        self.get_logger().info(
            "PointCloud preview active: "
            f"input={self.input_topic} output={self.output_topic} "
            f"rate={self.preview_rate_hz:.2f}Hz voxel={self.voxel_size_m:.3f}m "
            f"max_points={self.max_points}"
        )

    def _on_cloud(self, msg: Any) -> None:
        self._input_frames += 1
        now_ns = self.get_clock().now().nanoseconds
        if not should_publish_preview(self._last_publish_ns, now_ns, self.preview_rate_hz):
            return

        try:
            points = extract_preview_points(msg, include_intensity=self.include_intensity)
            preview = prepare_preview_points(
                points,
                voxel_size_m=self.voxel_size_m,
                min_range_m=self.min_range_m,
                max_range_m=self.max_range_m,
                max_points=self.max_points,
            )
            self._publisher.publish(build_preview_cloud(msg, preview))
        except Exception as exc:  # pragma: no cover - runtime guard
            self.get_logger().warn(f"Failed to publish pointcloud preview: {exc}", throttle_duration_sec=2.0)
            return

        self._last_publish_ns = now_ns
        self._published_frames += 1
        if self._published_frames == 1 or self._published_frames % 50 == 0:
            self.get_logger().info(
                f"Published preview {self._published_frames}: "
                f"input_points={int(getattr(msg, 'width', 0)) * int(getattr(msg, 'height', 1) or 1)} "
                f"preview_points={preview.shape[0]} topic={self.output_topic}"
            )


def main(args: list[str] | None = None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not available")
    rclpy.init(args=args)
    node = PointCloudPreviewNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
