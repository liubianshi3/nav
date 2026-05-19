from __future__ import annotations

import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from backend.grpc_codegen import _find_proto_files


def test_find_proto_files_ignores_hidden_mac_resource_forks(tmp_path: Path) -> None:
    proto_root = tmp_path / "proto"
    device_dir = proto_root / "device"
    hidden_dir = proto_root / ".hidden"
    device_dir.mkdir(parents=True)
    hidden_dir.mkdir(parents=True)

    robot_dog = device_dir / "robot_dog.proto"
    apple_double = device_dir / "._robot_dog.proto"
    hidden_proto = hidden_dir / "ignored.proto"

    robot_dog.write_text('syntax = "proto3";\n', encoding="utf-8")
    apple_double.write_bytes(b"\x00\x05invalid")
    hidden_proto.write_text('syntax = "proto3";\n', encoding="utf-8")

    assert _find_proto_files(proto_root) == [robot_dog]
