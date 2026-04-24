from __future__ import annotations

import os
import re
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

from .config import AppConfig
from .models import MapSnapshot, NodeCheck, SavedMapInfo, StackStatus


class StackControlError(RuntimeError):
    """Raised when a stack lifecycle command cannot be completed."""


MAP_ID_RE = re.compile(r"^[A-Za-z0-9_.-]+$")

MAPPING_NODES: list[tuple[str, str, str]] = [
    ("bringup", "bringup.launch.py", "bringup.launch.py"),
    ("sdk", "a2_sdk_bridge", "a2_sdk_bridge_node"),
    ("control", "a2_control_bridge", "a2_control_bridge_node"),
    ("occupancy", "occupancy_mapper", "occupancy_mapper"),
    ("map_manager", "map_manager", "map_manager_node"),
]

NAVIGATION_NODES: list[tuple[str, str, str]] = [
    ("bringup", "bringup.launch.py", "bringup.launch.py"),
    ("sdk", "a2_sdk_bridge", "a2_sdk_bridge_node"),
    ("control", "a2_control_bridge", "a2_control_bridge_node"),
    ("localization", "manual localization", "manual_localization_publisher"),
    ("goal_bridge", "goal bridge", "goal_bridge"),
    ("map_server", "map server", "map_server"),
    ("controller", "controller server", "controller_server"),
    ("planner", "planner server", "planner_server"),
    ("bt_navigator", "bt navigator", "bt_navigator"),
    ("velocity", "velocity smoother", "velocity_smoother"),
    ("lifecycle", "lifecycle manager", "lifecycle_manager"),
]


def _expand_path(value: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(value))).resolve()


class StackController:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.workspace = _expand_path(config.stack.workspace)
        self.map_root = _expand_path(config.stack.map_root)
        self.start_script = _expand_path(config.stack.start_script)
        self.stop_script = _expand_path(config.stack.stop_script)
        self.pid_file = self.workspace / "runtime" / "bringup.pid"
        self.runtime_state_file = self.workspace / "runtime" / "web_stack_state.yaml"
        self.timeout = float(config.stack.command_timeout_sec)

    def _run(self, command: list[str], *, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
        merged_env = os.environ.copy()
        merged_env["A2_WORKSPACE"] = str(self.workspace)
        if env:
            merged_env.update(env)
        try:
            result = subprocess.run(
                command,
                cwd=str(self.workspace),
                env=merged_env,
                text=True,
                capture_output=True,
                timeout=self.timeout,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise StackControlError(f"命令超时: {' '.join(command)}") from exc
        if result.returncode != 0:
            detail = (result.stderr or result.stdout or "").strip()
            raise StackControlError(detail or f"命令失败: {' '.join(command)}")
        return result

    def _write_runtime_state(self, **state: Any) -> None:
        self.runtime_state_file.parent.mkdir(parents=True, exist_ok=True)
        existing = self._read_runtime_state()
        existing.update(state)
        with self.runtime_state_file.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(existing, handle, sort_keys=False)

    def _read_runtime_state(self) -> dict[str, Any]:
        if not self.runtime_state_file.exists():
            return {}
        try:
            return yaml.safe_load(self.runtime_state_file.read_text(encoding="utf-8")) or {}
        except Exception:
            return {}

    def stop(self) -> dict[str, str]:
        if not self.stop_script.exists():
            raise StackControlError(f"停止脚本不存在: {self.stop_script}")
        result = self._run([str(self.stop_script)])
        self._cleanup_processes()
        self._write_runtime_state(mode="stopped", selected_map_id=None, selected_map_yaml=None)
        return {"message": (result.stdout or "已停止当前栈").strip()}

    def start_mapping(self) -> dict[str, str]:
        if not self.start_script.exists():
            raise StackControlError(f"启动脚本不存在: {self.start_script}")
        self.stop_if_running()
        result = self._run(
            [str(self.start_script), self.config.stack.network_interface, "enable_control_bridge:=true"],
            env={"A2_ENABLE_NAV2": "false", "A2_MAP_YAML": ""},
        )
        self._write_runtime_state(mode="mapping", selected_map_id=None, selected_map_yaml=None)
        return {"message": (result.stdout or "建图模式已启动").strip()}

    def start_navigation(self, map_id: str) -> dict[str, str]:
        map_info = self.get_map(map_id)
        if map_info is None:
            raise StackControlError(f"地图不存在: {map_id}")
        self.stop_if_running()
        result = self._run(
            [str(self.start_script), self.config.stack.network_interface],
            env={
                "A2_ENABLE_NAV2": "true",
                "A2_MAP_YAML": map_info.map_yaml,
            },
        )
        self._write_runtime_state(
            mode="navigation",
            selected_map_id=map_info.map_id,
            selected_map_yaml=map_info.map_yaml,
        )
        return {"message": (result.stdout or f"导航模式已启动: {map_id}").strip()}

    def stop_if_running(self) -> None:
        if self.pid_file.exists() or any("bringup.launch.py" in line for line in self._process_lines()):
            self.stop()

    def _cleanup_processes(self) -> None:
        patterns = [
            "bringup.launch.py",
            "a2_sdk_bridge_node",
            "a2_control_bridge_node",
            "manual_localization_publisher",
            "goal_bridge",
            "occupancy_mapper",
            "map_manager_node",
            "map_server",
            "controller_server",
            "planner_server",
            "bt_navigator",
            "velocity_smoother",
            "lifecycle_manager",
        ]
        for signal in ("-TERM", "-KILL"):
            for pattern in patterns:
                subprocess.run(["pkill", signal, "-f", pattern], capture_output=True, text=True, check=False)
            if signal == "-TERM":
                subprocess.run(["sleep", "1"], check=False)

    def list_maps(self) -> list[SavedMapInfo]:
        if not self.map_root.exists():
            return []
        maps: list[SavedMapInfo] = []
        for item in sorted(self.map_root.iterdir(), key=lambda path: path.name):
            if not item.is_dir():
                continue
            map_yaml = item / "map.yaml"
            if not map_yaml.exists():
                continue
            metadata = self._read_yaml(item / "metadata.yaml")
            maps.append(
                SavedMapInfo(
                    map_id=item.name,
                    map_yaml=str(map_yaml),
                    created_at=metadata.get("created_at"),
                    width=metadata.get("width"),
                    height=metadata.get("height"),
                    resolution=metadata.get("resolution"),
                )
            )
        return maps

    def get_map(self, map_id: str) -> SavedMapInfo | None:
        for item in self.list_maps():
            if item.map_id == map_id:
                return item
        return None

    def save_map(self, map_id: str, map_snapshot: MapSnapshot) -> SavedMapInfo:
        map_id = map_id.strip()
        if not map_id:
            map_id = datetime.now().strftime("map_%Y%m%d_%H%M%S")
        if not MAP_ID_RE.match(map_id):
            raise StackControlError("地图名只能包含字母、数字、下划线、点和短横线")
        if not map_snapshot.loaded or map_snapshot.width <= 0 or map_snapshot.height <= 0 or not map_snapshot.data:
            raise StackControlError("当前没有可保存的地图")

        map_dir = self.map_root / map_id
        map_dir.mkdir(parents=True, exist_ok=True)
        image_path = map_dir / "map.pgm"
        yaml_path = map_dir / "map.yaml"
        metadata_path = map_dir / "metadata.yaml"

        with image_path.open("wb") as handle:
            handle.write(f"P5\n{map_snapshot.width} {map_snapshot.height}\n255\n".encode("ascii"))
            for row in range(map_snapshot.height - 1, -1, -1):
                for col in range(map_snapshot.width):
                    index = row * map_snapshot.width + col
                    value = int(map_snapshot.data[index]) if 0 <= index < len(map_snapshot.data) else -1
                    if value < 0:
                        pixel = 205
                    elif value >= 65:
                        pixel = 0
                    else:
                        pixel = 254
                    handle.write(bytes([pixel]))

        yaml_data = {
            "image": "map.pgm",
            "resolution": float(map_snapshot.resolution),
            "origin": [float(map_snapshot.origin.x), float(map_snapshot.origin.y), 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.25,
            "mode": "trinary",
        }
        with yaml_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(yaml_data, handle, sort_keys=False)

        metadata = {
            "created_at": datetime.now().isoformat(),
            "source": "web_console",
            "width": map_snapshot.width,
            "height": map_snapshot.height,
            "resolution": map_snapshot.resolution,
        }
        with metadata_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(metadata, handle, sort_keys=False)

        return SavedMapInfo(
            map_id=map_id,
            map_yaml=str(yaml_path),
            created_at=metadata["created_at"],
            width=map_snapshot.width,
            height=map_snapshot.height,
            resolution=map_snapshot.resolution,
        )

    def status(self) -> StackStatus:
        processes = self._process_lines()
        runtime_state = self._read_runtime_state()
        has_nav = any("bt_navigator" in line or "controller_server" in line for line in processes)
        has_mapping = any("occupancy_mapper" in line for line in processes)
        has_bringup = any("bringup.launch.py" in line for line in processes)
        mode = "navigation" if has_nav else "mapping" if has_mapping else "starting" if has_bringup else "stopped"
        expected = NAVIGATION_NODES if mode == "navigation" else MAPPING_NODES if mode == "mapping" else []
        nodes = [
            NodeCheck(
                key=key,
                label=label,
                running=any(pattern in line for line in processes),
                required=True,
            )
            for key, label, pattern in expected
        ]
        return StackStatus(
            mode=mode,
            pid=self._read_pid(),
            log_file=self._latest_log_file(),
            selected_map_id=runtime_state.get("selected_map_id"),
            selected_map_yaml=runtime_state.get("selected_map_yaml"),
            nodes=nodes,
            maps=self.list_maps(),
        )

    def _process_lines(self) -> list[str]:
        result = subprocess.run(
            ["ps", "-eo", "pid,args"],
            text=True,
            capture_output=True,
            check=False,
        )
        return result.stdout.splitlines() if result.returncode == 0 else []

    def _read_pid(self) -> int | None:
        if not self.pid_file.exists():
            return None
        try:
            return int(self.pid_file.read_text(encoding="utf-8").strip())
        except ValueError:
            return None

    def _latest_log_file(self) -> str | None:
        log_dir = self.workspace / "runtime" / "logs"
        if not log_dir.exists():
            return None
        logs = sorted(log_dir.glob("bringup_real_*.log"), key=lambda path: path.stat().st_mtime, reverse=True)
        return str(logs[0]) if logs else None

    def _read_yaml(self, path: Path) -> dict[str, Any]:
        if not path.exists():
            return {}
        try:
            return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except Exception:
            return {}
