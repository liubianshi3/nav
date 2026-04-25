from __future__ import annotations

import os
import re
import signal
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

from .config import AppConfig
from .models import MapSnapshot, NodeCheck, SavedMapInfo, StackStatus


class StackControlError(RuntimeError):
    """Raised when a stack lifecycle command cannot be completed."""


MAP_ID_RE = re.compile(r"^[A-Za-z0-9_.-]+$")
POLL_INTERVAL_SEC = 0.5
START_STABILITY_POLLS = 3

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

STACK_CLEANUP_PATTERNS = [
    "bringup.launch.py",
    "a2_state_publisher_node",
    "a2_sdk_bridge_node",
    "a2_control_bridge_node",
    "safety_supervisor",
    "real_readiness_monitor",
    "static_tf_manager",
    "sync_monitor",
    "mid360_driver_guard",
    "pointcloud_frame_relay",
    "slam_orchestrator",
    "localization_gate",
    "exploration_manager",
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


@dataclass(frozen=True)
class ProcessInfo:
    pid: int
    ppid: int
    stat: str
    args: str

    @property
    def is_zombie(self) -> bool:
        return "Z" in self.stat


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
        self.start_timeout = max(self.timeout, 30.0)
        self.stop_timeout = max(self.timeout, 12.0)

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
        previous = self._read_runtime_state()
        last_mode = previous.get("target_mode") or previous.get("mode")
        self._write_runtime_state(mode="stopping", target_mode=last_mode, message="正在停止当前栈")

        if self.stop_script.exists():
            result = self._run([str(self.stop_script)])
            message = (result.stdout or "已停止当前栈").strip()
        else:
            message = "已停止当前栈"

        remaining = self._terminate_runtime_processes()
        if remaining:
            detail = ", ".join(sorted({self._describe_process(proc) for proc in remaining}))
            self._write_runtime_state(mode="stopped", target_mode=None, selected_map_id=None, selected_map_yaml=None, message=detail)
            raise StackControlError(f"停止后仍有残留进程: {detail}")

        self._write_runtime_state(
            mode="stopped",
            target_mode=None,
            selected_map_id=None,
            selected_map_yaml=None,
            message=None,
        )
        return {"message": message}

    def start_mapping(self) -> dict[str, str]:
        if not self.start_script.exists():
            raise StackControlError(f"启动脚本不存在: {self.start_script}")

        self.stop_if_running()
        self._write_runtime_state(
            mode="starting",
            target_mode="mapping",
            selected_map_id=None,
            selected_map_yaml=None,
            message="建图模式启动中",
        )

        try:
            result = self._run(
                [str(self.start_script), self.config.stack.network_interface, "enable_control_bridge:=true"],
                env={"A2_ENABLE_NAV2": "false", "A2_MAP_YAML": ""},
            )
            self._wait_for_expected_nodes("mapping")
        except Exception:
            self._terminate_runtime_processes()
            self._write_runtime_state(mode="stopped", target_mode=None, selected_map_id=None, selected_map_yaml=None, message=None)
            raise

        self._write_runtime_state(
            mode="mapping",
            target_mode=None,
            selected_map_id=None,
            selected_map_yaml=None,
            message=None,
        )
        return {"message": (result.stdout or "建图模式已启动").strip()}

    def start_navigation(self, map_id: str) -> dict[str, str]:
        if not self.start_script.exists():
            raise StackControlError(f"启动脚本不存在: {self.start_script}")

        map_info = self.get_map(map_id)
        if map_info is None:
            raise StackControlError(f"地图不存在: {map_id}")

        self.stop_if_running()
        self._write_runtime_state(
            mode="starting",
            target_mode="navigation",
            selected_map_id=map_info.map_id,
            selected_map_yaml=map_info.map_yaml,
            message=f"导航模式启动中: {map_info.map_id}",
        )

        try:
            result = self._run(
                [str(self.start_script), self.config.stack.network_interface],
                env={
                    "A2_ENABLE_NAV2": "true",
                    "A2_MAP_YAML": map_info.map_yaml,
                },
            )
            self._wait_for_expected_nodes("navigation")
        except Exception:
            self._terminate_runtime_processes()
            self._write_runtime_state(mode="stopped", target_mode=None, selected_map_id=None, selected_map_yaml=None, message=None)
            raise

        self._write_runtime_state(
            mode="navigation",
            target_mode=None,
            selected_map_id=map_info.map_id,
            selected_map_yaml=map_info.map_yaml,
            message=None,
        )
        return {"message": (result.stdout or f"导航模式已启动: {map_id}").strip()}

    def stop_if_running(self) -> None:
        if self._runtime_processes() or self._read_runtime_state().get("mode") not in (None, "stopped"):
            self.stop()

    def _wait_for_expected_nodes(self, mode: str) -> None:
        expected = NAVIGATION_NODES if mode == "navigation" else MAPPING_NODES
        deadline = time.monotonic() + self.start_timeout
        missing_labels: list[str] = []
        stable_polls = 0

        while time.monotonic() < deadline:
            processes = self._runtime_processes()
            missing_labels = [label for _, label, pattern in expected if not any(pattern in proc.args for proc in processes)]
            if not missing_labels:
                stable_polls += 1
                if stable_polls >= START_STABILITY_POLLS:
                    return
            else:
                stable_polls = 0
            time.sleep(POLL_INTERVAL_SEC)

        detail = "、".join(missing_labels) if missing_labels else "未知节点"
        raise StackControlError(f"{'导航' if mode == 'navigation' else '建图'}模式启动超时，缺少节点: {detail}")

    def _terminate_runtime_processes(self) -> list[ProcessInfo]:
        remaining = self._runtime_processes()
        if not remaining:
            self.pid_file.unlink(missing_ok=True)
            return []

        for sig, wait_window in ((signal.SIGTERM, 5.0), (signal.SIGKILL, 3.0)):
            records = self._process_records()
            target_pids = self._collect_cleanup_pids(records)
            self._signal_pids(target_pids, sig)
            remaining = self._wait_for_process_exit(wait_window)
            if not remaining:
                break

        self.pid_file.unlink(missing_ok=True)
        return remaining

    def _collect_cleanup_pids(self, records: list[ProcessInfo]) -> set[int]:
        if not records:
            return set()

        pid_to_children: dict[int, set[int]] = {}
        for proc in records:
            pid_to_children.setdefault(proc.ppid, set()).add(proc.pid)

        def collect_descendants(root_pid: int) -> set[int]:
            pending = [root_pid]
            collected: set[int] = set()
            while pending:
                current = pending.pop()
                if current in collected:
                    continue
                collected.add(current)
                pending.extend(pid_to_children.get(current, ()))
            return collected

        roots: set[int] = set()
        recorded_pid = self._read_pid()
        if recorded_pid:
            roots.add(recorded_pid)
        roots.update(proc.pid for proc in records if "bringup.launch.py" in proc.args)

        target_pids: set[int] = set()
        for root in roots:
            target_pids.update(collect_descendants(root))

        for proc in records:
            if any(pattern in proc.args for pattern in STACK_CLEANUP_PATTERNS):
                target_pids.add(proc.pid)

        target_pids.discard(os.getpid())
        return target_pids

    def _signal_pids(self, pids: set[int], sig: signal.Signals) -> None:
        for pid in sorted(pids):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                continue
            except PermissionError:
                continue

    def _wait_for_process_exit(self, timeout: float) -> list[ProcessInfo]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            remaining = self._runtime_processes()
            if not remaining:
                return []
            time.sleep(POLL_INTERVAL_SEC)
        return self._runtime_processes()

    def _runtime_processes(self) -> list[ProcessInfo]:
        return [
            proc
            for proc in self._process_records()
            if any(pattern in proc.args for pattern in STACK_CLEANUP_PATTERNS)
        ]

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
        processes = self._process_records()
        runtime_state = self._read_runtime_state()
        inferred_mode = self._infer_mode(processes)
        runtime_mode = str(runtime_state.get("mode") or "")
        target_mode = str(runtime_state.get("target_mode") or "")

        if runtime_mode in {"starting", "stopping"}:
            mode = runtime_mode
            expected_mode = target_mode or inferred_mode
        elif inferred_mode != "stopped":
            mode = inferred_mode
            expected_mode = inferred_mode
        elif runtime_mode in {"mapping", "navigation"}:
            mode = runtime_mode
            expected_mode = runtime_mode
        else:
            mode = "stopped"
            expected_mode = "stopped"

        expected = NAVIGATION_NODES if expected_mode == "navigation" else MAPPING_NODES if expected_mode == "mapping" else []
        nodes = [
            NodeCheck(
                key=key,
                label=label,
                state=self._node_state(processes, pattern, mode),
                running=any(pattern in proc.args for proc in processes),
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
            message=runtime_state.get("message"),
        )

    def _infer_mode(self, processes: list[ProcessInfo]) -> str:
        has_nav = any("bt_navigator" in proc.args or "controller_server" in proc.args for proc in processes)
        has_mapping = any("occupancy_mapper" in proc.args for proc in processes)
        has_bringup = any("bringup.launch.py" in proc.args for proc in processes)
        if has_nav:
            return "navigation"
        if has_mapping:
            return "mapping"
        if has_bringup:
            return "starting"
        return "stopped"

    def _node_state(self, processes: list[ProcessInfo], pattern: str, mode: str) -> str:
        if any(pattern in proc.args for proc in processes):
            return "running"
        if mode == "starting":
            return "starting"
        if mode == "stopping":
            return "stopping"
        return "missing"

    def _process_records(self) -> list[ProcessInfo]:
        result = subprocess.run(
            ["ps", "-eo", "pid=,ppid=,stat=,args="],
            text=True,
            capture_output=True,
            check=False,
        )
        if result.returncode != 0:
            return []

        records: list[ProcessInfo] = []
        for line in result.stdout.splitlines():
            parts = line.strip().split(maxsplit=3)
            if len(parts) < 4:
                continue
            try:
                pid = int(parts[0])
                ppid = int(parts[1])
            except ValueError:
                continue
            proc = ProcessInfo(pid=pid, ppid=ppid, stat=parts[2], args=parts[3])
            if proc.is_zombie:
                continue
            records.append(proc)
        return records

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

    def _describe_process(self, process: ProcessInfo) -> str:
        for _, label, pattern in NAVIGATION_NODES + MAPPING_NODES:
            if pattern in process.args:
                return label
        return f"pid={process.pid}"
