from __future__ import annotations

from tf_manager.static_tf_manager import validate_static_tf_contract


def test_static_tf_contract_accepts_valid_sensor_child():
    valid, reason = validate_static_tf_contract(
        "base_link",
        "camera_link",
        [0.2, 0.0, 0.3],
        [0.0, 0.0, 0.0],
        {"map", "odom"},
        set(),
    )
    assert valid is True
    assert reason == "ok"


def test_static_tf_contract_accepts_rotation_matrix():
    valid, reason = validate_static_tf_contract(
        "base_link",
        "jt128_front_link",
        [0.33767, 0.0, 0.08134],
        [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        {"map", "odom"},
        set(),
    )
    assert valid is True
    assert reason == "ok"


def test_static_tf_contract_rejects_dynamic_and_duplicate_children():
    assert validate_static_tf_contract(
        "base_link",
        "odom",
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        {"map", "odom"},
        set(),
    )[1] == "dynamic_child_frame"
    assert validate_static_tf_contract(
        "base_link",
        "lidar_link",
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        {"map", "odom"},
        {"lidar_link"},
    )[1] == "duplicate_child_frame"


def test_static_tf_contract_rejects_invalid_vectors():
    assert validate_static_tf_contract(
        "base_link",
        "imu_link",
        [0.0, 0.0],
        [0.0, 0.0, 0.0],
        {"map", "odom"},
        set(),
    )[1] == "invalid_vector_length"
    assert validate_static_tf_contract(
        "base_link",
        "imu_link",
        [0.0, "bad", 0.0],
        [0.0, 0.0, 0.0],
        {"map", "odom"},
        set(),
    )[1] == "non_numeric_transform"
