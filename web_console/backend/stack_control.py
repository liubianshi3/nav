from __future__ import annotations

import os
import math
import re
import shutil
import shlex
import signal
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

from .config import AppConfig
from .models import (
    MapArtifactInfo,
    MapMediaEntry,
    MapMediaListing,
    MapSnapshot,
    NodeCheck,
    SavedMapInfo,
    StackStatus,
    VirtualObstacleListing,
    VirtualObstacleUpsertRequest,
    VirtualObstacleZone,
)


class StackControlError(RuntimeError):
    """Raised when a stack lifecycle command cannot be completed."""


MAP_ID_RE = re.compile(r"^[A-Za-z0-9_.-]+$")
OBSTACLE_ID_RE = re.compile(r"^[A-Za-z0-9_.-]+$")
POLL_INTERVAL_SEC = 0.5
START_STABILITY_POLLS = 3
LOG_TAIL_LINE_LIMIT = 80
LOG_HIGHLIGHT_LIMIT = 12
LOG_TEXT_LIMIT = 1800
# Legacy 2D AMCL removed from primary lifecycle nodes; 3D NDT adapter is not lifecycle-managed
NAV_LIFECYCLE_NODES = ("map_server",)
LOG_HIGHLIGHT_MARKERS = (
    "[error]",
    "traceback",
    "exception",
    "failed",
    "not found",
    "no such file",
    "timeout",
    "killed",
    "rclerror",
)
PatternSpec = str | tuple[str, ...]

MAPPING_NODES: list[tuple[str, str, PatternSpec]] = [
    ("driver", "JT128 Hesai driver", ("jt128_hesai_driver", "hesai_ros_driver_node")),
    ("dlio_odom", "JT128 DLIO odom", ("jt128_dlio_odom", "dlio_odom_node")),
    ("dlio_map", "JT128 DLIO map", ("jt128_dlio_map", "dlio_map_node")),
    ("map_manager", "map_manager", "map_manager_node"),
]

NAVIGATION_NODES: list[tuple[str, str, PatternSpec]] = [
    ("bringup", "bringup.launch.py", "bringup.launch.py"),
    ("sdk", "a2_sdk_bridge", "a2_sdk_bridge_node"),
    ("control", "a2_control_bridge", "a2_control_bridge_node"),
    ("localization", "3D NDT localization", ("ndt_adapter", "localization_gate")),  # legacy AMCL removed
    ("goal_bridge", "goal bridge", "goal_bridge"),
    ("map_server", "map server", "map_server"),
    ("controller", "controller server", "controller_server"),
    ("planner", "planner server", "planner_server"),
    ("bt_navigator", "bt navigator", "bt_navigator"),
    ("velocity", "velocity smoother", "velocity_smoother"),
    ("lifecycle", "lifecycle manager", "lifecycle_manager"),
]

NAVIGATION_NODES_3D: list[tuple[str, str, PatternSpec]] = [
    ("navigation_launch", "JT128 3D navigation launch", "jt128_3d_navigation.launch.py"),
    ("dlio_odom", "JT128 DLIO odom", ("jt128_dlio_odom", "dlio_odom_node")),
    ("dlio_map", "JT128 DLIO map", ("jt128_dlio_map", "dlio_map_node")),
    ("sdk", "a2_sdk_bridge", "a2_sdk_bridge_node"),
    ("control", "a2_control_bridge", "a2_control_bridge_node"),
    ("map_loader", "3D pointcloud map loader", "pointcloud_map_loader"),
    ("relocalizer", "3D PCD relocalizer", "pcd_relocalizer_3d"),
    ("localization", "3D localization gate", "localization_gate"),
    ("goal_bridge", "3D goal bridge", "goal_bridge"),
    ("goal_controller", "3D pose goal controller", "pose_goal_controller_3d"),
    ("map_manager", "3D map manager", "map_manager_node"),
]

STACK_CLEANUP_PATTERNS = [
    "bringup.launch.py",
    "jt128_3d_navigation.launch.py",
    "dlio_mapping.launch.py",
    "a2_state_publisher_node",
    "a2_sdk_bridge_node",
    "a2_control_bridge_node",
    "task_manager.py",
    "safety_supervisor",
    "real_readiness_monitor",
    "static_tf_manager",
    "sync_monitor",
    "pointcloud_guard",
    "pointcloud_map_loader",
    "mid360_driver_guard",
    "pointcloud_relay",
    "pointcloud_accumulator",
    "jt128_hesai_driver",
    "jt128_dlio_odom",
    "jt128_dlio_map",
    "dlio_odom_node",
    "dlio_map_node",
    # Legacy 2D nodes removed from primary validation:
    # "pointcloud_to_laserscan",
    # "slam_toolbox",
    "native_map_relay",
    "slam_orchestrator",
    "pcd_relocalizer_3d",
    "localization_gate",
    "exploration_manager",
    "manual_localization_publisher",
    # "amcl",  # legacy 2D — removed from primary validation
    "goal_bridge",
    "pose_goal_controller_3d",
    "occupancy_mapper",
    "map_manager_node",
    "map_server",
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
    "lifecycle_manager",
]

IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg", ".webp", ".bmp", ".gif")
POINTCLOUD_SUFFIXES = (".pcd", ".ply", ".xyz", ".xyzn", ".pts")
TEXT_SUFFIXES = (".txt", ".log", ".json", ".yaml", ".yml", ".csv")

ExplicitMediaIndex = dict[str, dict[str, Any]]


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
        self.a2_system_config_dir = self._resolve_a2_system_config_dir()
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

    def _start_script_command(self, mode: str, map_id: str | None = None) -> list[str]:
        if self.start_script.name == "start_jt128_3d_stack.sh":
            command = [
                str(self.start_script),
                "--mode",
                mode,
                "--lidar-iface",
                self.config.stack.network_interface,
                "--no-web",
            ]
            if mode == "navigation":
                if not map_id:
                    raise StackControlError("3D 导航模式缺少地图 ID")
                command.extend(["--map-id", map_id, "--enable-motion"])
            return command
        if self.start_script.name == "start_jt128_dlio_mapping.sh":
            if mode != "mapping":
                fallback = self.start_script.with_name("start_jt128_3d_stack.sh")
                if fallback.exists():
                    command = [
                        str(fallback),
                        "--mode",
                        "navigation",
                        "--map-id",
                        map_id or "",
                        "--lidar-iface",
                        self.config.stack.network_interface,
                        "--enable-motion",
                        "--no-web",
                    ]
                    return command
                raise StackControlError("当前启动脚本只支持建图，不能启动 3D 导航")
            return [str(self.start_script), "--iface", self.config.stack.network_interface, "--no-web"]
        return [str(self.start_script), self.config.stack.network_interface, "enable_control_bridge:=true"]

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
                self._start_script_command("mapping"),
                env={"A2_ENABLE_NAV2": "false", "A2_MAP_YAML": ""},
            )
            self._wait_for_expected_nodes("mapping")
        except Exception as exc:
            self._terminate_runtime_processes()
            self._write_runtime_state(
                mode="stopped",
                target_mode=None,
                selected_map_id=None,
                selected_map_yaml=None,
                message=str(exc),
            )
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

        map_info = self.get_map(map_id, include_incompatible=True)
        if map_info is None:
            raise StackControlError(f"地图不存在: {map_id}")
        if not map_info.navigation_compatible:
            raise StackControlError(
                map_info.navigation_compatibility_reason or f"地图不兼容当前导航链: {map_id}"
            )

        self.stop_if_running()
        self._write_runtime_state(
            mode="starting",
            target_mode="navigation",
            selected_map_id=map_info.map_id,
            selected_map_yaml=map_info.map_yaml,
            message=f"导航模式启动中: {map_info.map_id}",
        )

        use_3d_navigation = self._is_3d_navigation_map(map_info)
        try:
            result = self._run(
                self._start_script_command("navigation" if use_3d_navigation else "mapping", map_info.map_id),
                env={
                    "A2_ENABLE_NAV2": "false" if use_3d_navigation else "true",
                    "A2_REAL_LOCALIZATION_MODE": "uslam_odom" if use_3d_navigation else "amcl",
                    "A2_MAP_YAML": map_info.map_yaml or "",
                },
            )
            self._wait_for_expected_nodes("navigation", use_3d_navigation=use_3d_navigation)
            if not use_3d_navigation:
                self._ensure_navigation_lifecycle_ready()
        except Exception as exc:
            self._terminate_runtime_processes()
            self._write_runtime_state(
                mode="stopped",
                target_mode=None,
                selected_map_id=None,
                selected_map_yaml=None,
                message=str(exc),
            )
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

    def mapping_source_profile(self) -> str:
        slam_cfg = self._read_yaml(self.a2_system_config_dir / "slam.yaml")
        params = slam_cfg.get("slam_manager", {}).get("ros__parameters", {}) or {}
        profile = str(params.get("mapping_stack_profile", "") or "").strip()
        return profile or "front_lidar_pointcloud_3d"  # legacy default "slam_toolbox" removed

    def navigation_representation(self) -> str:
        slam_cfg = self._read_yaml(self.a2_system_config_dir / "slam.yaml")
        params = slam_cfg.get("slam_manager", {}).get("ros__parameters", {}) or {}
        return str(params.get("navigation_representation", "") or "").strip()

    def _is_3d_navigation_map(self, map_info: SavedMapInfo | None = None) -> bool:
        if self.navigation_representation() == "pointcloud_map_3d":
            return bool(map_info and map_info.navigation_compatible)
        if map_info is None:
            return False
        return bool(map_info.has_pointcloud_3d or map_info.representation == "pointcloud_map_3d")

    def _navigation_compatibility_for_map(self, map_info: SavedMapInfo) -> tuple[bool, str | None]:
        if self.navigation_representation() != "pointcloud_map_3d":
            return True, None
        if map_info.has_pointcloud_3d or map_info.representation == "pointcloud_map_3d":
            return True, None
        return (
            False,
            "当前导航链要求 3D 点云地图；该地图缺少 pointcloud_map_3d 资产，仅保留旧 2D 兼容资产",
        )

    def _expected_nodes_for_mode(
        self,
        mode: str,
        *,
        use_3d_navigation: bool | None = None,
    ) -> list[tuple[str, str, PatternSpec]]:
        if mode == "navigation":
            if use_3d_navigation is None:
                runtime_state = self._read_runtime_state()
                selected_map_id = runtime_state.get("selected_map_id")
                map_info = self.get_map(str(selected_map_id), include_incompatible=True) if selected_map_id else None
                if map_info is not None:
                    use_3d_navigation = self._is_3d_navigation_map(map_info)
                else:
                    use_3d_navigation = self.navigation_representation() == "pointcloud_map_3d"
            if use_3d_navigation:
                return NAVIGATION_NODES_3D
            return NAVIGATION_NODES
        return MAPPING_NODES

    def _wait_for_expected_nodes(self, mode: str, *, use_3d_navigation: bool | None = None) -> None:
        expected = self._expected_nodes_for_mode(mode, use_3d_navigation=use_3d_navigation)
        deadline = time.monotonic() + self.start_timeout
        missing_labels: list[str] = []
        stable_polls = 0

        while time.monotonic() < deadline:
            processes = self._runtime_processes()
            missing_labels = [
                label
                for _, label, pattern in expected
                if not any(self._matches_pattern(proc.args, pattern) for proc in processes)
            ]
            if not missing_labels:
                stable_polls += 1
                if stable_polls >= START_STABILITY_POLLS:
                    return
            else:
                stable_polls = 0
            time.sleep(POLL_INTERVAL_SEC)

        raise StackControlError(self._build_start_timeout_message(mode, missing_labels))

    def _ensure_navigation_lifecycle_ready(self) -> None:
        deadline = time.monotonic() + self.start_timeout
        stable_polls = 0
        activation_attempts: set[tuple[str, str]] = set()
        states: dict[str, str] = {}
        last_known_states: dict[str, str] = {}
        failures: list[str] = []

        while time.monotonic() < deadline:
            raw_states = {node: self._get_lifecycle_state(node) for node in NAV_LIFECYCLE_NODES}
            states = {}
            for node, raw_state in raw_states.items():
                if raw_state.startswith("query_failed:") and last_known_states.get(node, "").startswith("active"):
                    states[node] = last_known_states[node]
                else:
                    states[node] = raw_state
                if states[node].startswith("active"):
                    last_known_states[node] = states[node]
            if all(state.startswith("active") for state in states.values()):
                failures = []
                stable_polls += 1
                if stable_polls >= START_STABILITY_POLLS:
                    return
            else:
                stable_polls = 0
                failures = []
                for node, state in states.items():
                    if state.startswith("active"):
                        continue
                    if state.startswith("inactive"):
                        transition = "activate"
                    elif state.startswith("unconfigured"):
                        transition = "configure"
                    else:
                        failures.append(f"{node}={state}")
                        continue
                    attempt = (node, transition)
                    if attempt in activation_attempts:
                        continue
                    if self._set_lifecycle_transition(node, transition):
                        activation_attempts.add(attempt)
                    else:
                        failures.append(f"{node}={state},transition={transition} failed")
            time.sleep(POLL_INTERVAL_SEC)

        raise StackControlError(self._build_navigation_lifecycle_message(states, failures))

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
        roots.update(
            proc.pid
            for proc in records
            if "bringup.launch.py" in proc.args
            or "jt128_3d_navigation.launch.py" in proc.args
            or "dlio_mapping.launch.py" in proc.args
        )

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

    def list_maps(self, *, include_incompatible: bool = False) -> list[SavedMapInfo]:
        if not self.map_root.exists():
            return []
        maps: list[SavedMapInfo] = []
        for item in sorted(self.map_root.iterdir(), key=lambda path: path.name):
            if not item.is_dir():
                continue
            map_yaml = item / "map.yaml"
            metadata = self._read_yaml(item / "metadata.yaml")
            if not map_yaml.exists() and not metadata:
                continue
            artifact_models = [
                MapArtifactInfo(**artifact)
                for artifact in (metadata.get("artifacts") or [])
                if isinstance(artifact, dict) and artifact.get("kind") and artifact.get("path")
            ]
            map_info = SavedMapInfo(
                map_id=item.name,
                map_yaml=str(map_yaml) if map_yaml.exists() else None,
                created_at=metadata.get("created_at"),
                representation=metadata.get("representation"),
                source_topic=metadata.get("source_topic"),
                pointcloud_topic_3d=metadata.get("pointcloud_topic_3d"),
                has_pointcloud_3d=any(
                    artifact.kind in {"pointcloud_snapshot_3d", "native_pointcloud_map_3d"}
                    for artifact in artifact_models
                ),
                width=metadata.get("width"),
                height=metadata.get("height"),
                resolution=metadata.get("resolution"),
                artifacts=artifact_models,
            )
            compatible, reason = self._navigation_compatibility_for_map(map_info)
            map_info.navigation_compatible = compatible
            map_info.navigation_compatibility_reason = reason
            if include_incompatible or compatible:
                maps.append(map_info)
        return maps

    def get_map(self, map_id: str, *, include_incompatible: bool = True) -> SavedMapInfo | None:
        for item in self.list_maps(include_incompatible=include_incompatible):
            if item.map_id == map_id:
                return item
        return None

    def list_map_media(self, map_id: str) -> MapMediaListing:
        map_info = self.get_map(map_id)
        if map_info is None:
            raise StackControlError(f"地图不存在: {map_id}")

        map_dir = self._resolve_map_dir(map_id)
        artifact_by_path = {artifact.path: artifact for artifact in map_info.artifacts}
        explicit_index = self._read_map_media_index(map_dir)
        candidate_paths = {
            path.relative_to(map_dir).as_posix()
            for path in map_dir.rglob("*")
            if path.is_file()
        }
        entries: list[MapMediaEntry] = []

        for relative_path in sorted(candidate_paths):
            file_path = map_dir / Path(relative_path)
            artifact = artifact_by_path.get(relative_path)
            explicit = explicit_index.get(relative_path, {})
            kind = str(explicit.get("kind") or self._classify_media_kind(file_path, artifact.kind if artifact else None))
            explicit_linked_pointcloud = self._normalize_media_link(explicit.get("linked_pointcloud_path"), candidate_paths)
            explicit_linked_image = self._normalize_media_link(explicit.get("linked_image_path"), candidate_paths)
            inferred_linked_pointcloud = (
                self._find_linked_pointcloud_path(relative_path, candidate_paths) if kind == "image" else None
            )
            inferred_linked_image = (
                self._find_linked_image_path(relative_path, candidate_paths) if kind == "pointcloud" else None
            )
            linked_pointcloud = explicit_linked_pointcloud or inferred_linked_pointcloud
            linked_image = explicit_linked_image or inferred_linked_image
            link_source = None
            if explicit_linked_pointcloud or explicit_linked_image:
                link_source = "metadata"
            elif linked_pointcloud or linked_image:
                link_source = "inferred"
            entries.append(
                MapMediaEntry(
                    kind=kind,
                    path=relative_path,
                    name=str(explicit.get("name") or file_path.name),
                    group=str(explicit.get("group") or (file_path.parent.relative_to(map_dir).as_posix() if file_path.parent != map_dir else "root")),
                    size_bytes=file_path.stat().st_size if file_path.exists() else None,
                    artifact_kind=artifact.kind if artifact else None,
                    linked_pointcloud_path=linked_pointcloud,
                    linked_image_path=linked_image,
                    link_source=link_source,
                )
            )

        return MapMediaListing(map_id=map_id, entries=entries)

    def list_virtual_obstacles(self, map_id: str) -> VirtualObstacleListing:
        map_dir = self._resolve_map_dir(map_id)
        return VirtualObstacleListing(
            map_id=map_id,
            obstacles=self._load_virtual_obstacles(map_dir),
        )

    def save_virtual_obstacle(
        self,
        map_id: str,
        request: VirtualObstacleUpsertRequest,
    ) -> VirtualObstacleListing:
        map_dir = self._resolve_map_dir(map_id)
        obstacle_id = self._normalize_obstacle_id(request.obstacle_id)
        radius = float(request.radius)
        x = float(request.x)
        y = float(request.y)
        if not all(math.isfinite(value) for value in (x, y, radius)):
            raise StackControlError("障碍物坐标或半径包含非有限数值")
        if radius <= 0.0:
            raise StackControlError("障碍物半径必须大于 0")

        now = datetime.now().isoformat()
        zones = self._load_virtual_obstacles(map_dir)
        existing = next((zone for zone in zones if zone.obstacle_id == obstacle_id), None)
        created_at = existing.created_at if existing is not None else now
        replacement = VirtualObstacleZone(
            obstacle_id=obstacle_id,
            label=(request.label or "").strip() or obstacle_id,
            kind="circle_keepout",
            x=x,
            y=y,
            radius=radius,
            created_at=created_at,
            updated_at=now,
        )
        updated = [zone for zone in zones if zone.obstacle_id != obstacle_id]
        updated.append(replacement)
        updated.sort(key=lambda zone: zone.obstacle_id)
        self._write_virtual_obstacles(map_dir, updated)
        return VirtualObstacleListing(map_id=map_id, obstacles=updated)

    def delete_virtual_obstacle(self, map_id: str, obstacle_id: str) -> VirtualObstacleListing:
        map_dir = self._resolve_map_dir(map_id)
        normalized_id = self._normalize_obstacle_id(obstacle_id)
        zones = self._load_virtual_obstacles(map_dir)
        updated = [zone for zone in zones if zone.obstacle_id != normalized_id]
        if len(updated) == len(zones):
            raise StackControlError(f"虚拟障碍物不存在: {normalized_id}")
        self._write_virtual_obstacles(map_dir, updated)
        return VirtualObstacleListing(map_id=map_id, obstacles=updated)

    def find_virtual_obstacle_hit(
        self,
        map_id: str,
        *,
        x: float,
        y: float,
        padding: float = 0.0,
    ) -> VirtualObstacleZone | None:
        if not map_id:
            return None
        obstacles = self.list_virtual_obstacles(map_id).obstacles
        clearance = max(0.0, float(padding))
        for obstacle in obstacles:
            limit = obstacle.radius + clearance
            if math.hypot(float(x) - obstacle.x, float(y) - obstacle.y) <= limit:
                return obstacle
        return None

    def validate_point_outside_virtual_obstacles(
        self,
        map_id: str,
        *,
        x: float,
        y: float,
        subject: str,
        padding: float = 0.0,
    ) -> None:
        obstacle = self.find_virtual_obstacle_hit(map_id, x=x, y=y, padding=padding)
        if obstacle is None:
            return
        label = obstacle.label or obstacle.obstacle_id
        raise StackControlError(
            f"{subject}落在虚拟障碍物内: {label} (中心 {obstacle.x:.2f}, {obstacle.y:.2f}, 半径 {obstacle.radius:.2f} m)"
        )

    def resolve_map_file(self, map_id: str, relative_path: str) -> Path:
        map_dir = self._resolve_map_dir(map_id)
        candidate = (map_dir / relative_path).resolve()
        if map_dir != candidate and map_dir not in candidate.parents:
            raise StackControlError("非法地图文件路径")
        if not candidate.exists() or not candidate.is_file():
            raise StackControlError(f"地图文件不存在: {relative_path}")
        return candidate

    def attach_native_pointcloud_artifact(
        self,
        map_id: str,
        native_pcd_path: str,
        *,
        pointcloud_topic_3d: str | None = None,
    ) -> SavedMapInfo:
        map_id = map_id.strip()
        if not map_id:
            raise StackControlError("地图名不能为空")
        if not MAP_ID_RE.match(map_id):
            raise StackControlError("地图名只能包含字母、数字、下划线、点和短横线")

        source_path = Path(native_pcd_path).expanduser().resolve()
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if source_path.exists() and source_path.is_file() and source_path.stat().st_size > 0:
                break
            time.sleep(0.25)
        if not source_path.exists() or not source_path.is_file() or source_path.stat().st_size <= 0:
            raise StackControlError(f"原生 3D 地图文件不存在或为空: {source_path}")

        map_dir = self.map_root / map_id
        map_dir.mkdir(parents=True, exist_ok=True)
        target_path = map_dir / "native_map.pcd"
        shutil.copy2(source_path, target_path)

        metadata_path = map_dir / "metadata.yaml"
        metadata = self._read_yaml(metadata_path)
        artifacts = [
            artifact
            for artifact in (metadata.get("artifacts") or [])
            if isinstance(artifact, dict)
            and artifact.get("kind") != "native_pointcloud_map_3d"
        ]
        artifacts.append(
            {
                "kind": "native_pointcloud_map_3d",
                "path": target_path.name,
                "topic": pointcloud_topic_3d,
            }
        )

        if not metadata.get("created_at"):
            metadata["created_at"] = datetime.now().isoformat()
        metadata["representation"] = "pointcloud_map_3d"
        metadata["pointcloud_topic_3d"] = pointcloud_topic_3d
        metadata["artifacts"] = artifacts

        with metadata_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(metadata, handle, sort_keys=False)

        saved = self.get_map(map_id)
        if saved is None:
            raise StackControlError(f"3D 地图资产已写入，但未能注册地图: {map_id}")
        return saved

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
            "representation": map_snapshot.representation,
            "source_topic": "/map",
            "pointcloud_topic_3d": None,
            "width": map_snapshot.width,
            "height": map_snapshot.height,
            "resolution": map_snapshot.resolution,
            "artifacts": [
                {
                    "kind": "occupancy_grid_2d",
                    "path": "map.yaml",
                    "topic": "/map",
                    "resolution": map_snapshot.resolution,
                }
            ],
        }
        with metadata_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(metadata, handle, sort_keys=False)

        return SavedMapInfo(
            map_id=map_id,
            map_yaml=str(yaml_path),
            created_at=metadata["created_at"],
            representation=metadata["representation"],
            source_topic=metadata["source_topic"],
            pointcloud_topic_3d=None,
            has_pointcloud_3d=False,
            width=map_snapshot.width,
            height=map_snapshot.height,
            resolution=map_snapshot.resolution,
            artifacts=[MapArtifactInfo(**metadata["artifacts"][0])],
        )

    def _resolve_map_dir(self, map_id: str) -> Path:
        map_id = map_id.strip()
        if not map_id or not MAP_ID_RE.match(map_id):
            raise StackControlError("地图名非法")
        map_dir = (self.map_root / map_id).resolve()
        map_root = self.map_root.resolve()
        if map_dir != map_root and map_root not in map_dir.parents:
            raise StackControlError("非法地图路径")
        if not map_dir.exists() or not map_dir.is_dir():
            raise StackControlError(f"地图目录不存在: {map_id}")
        return map_dir

    def _classify_media_kind(self, file_path: Path, artifact_kind: str | None = None) -> str:
        suffix = file_path.suffix.lower()
        if suffix in IMAGE_SUFFIXES:
            return "image"
        if suffix in POINTCLOUD_SUFFIXES:
            return "pointcloud"
        if suffix == ".pgm" or artifact_kind == "occupancy_grid_2d":
            return "occupancy"
        if suffix in {".yaml", ".yml"}:
            return "yaml"
        if suffix in TEXT_SUFFIXES:
            return "text"
        return "other"

    def _virtual_obstacles_path(self, map_dir: Path) -> Path:
        return map_dir / "virtual_obstacles.yaml"

    def _normalize_obstacle_id(self, obstacle_id: str | None) -> str:
        normalized = (obstacle_id or "").strip()
        if not normalized:
            normalized = f"obs_{datetime.now().strftime('%H%M%S_%f')}"
        if not OBSTACLE_ID_RE.fullmatch(normalized):
            raise StackControlError("障碍物 ID 只能包含字母、数字、下划线、点和短横线")
        return normalized

    def _load_virtual_obstacles(self, map_dir: Path) -> list[VirtualObstacleZone]:
        payload = self._read_yaml(self._virtual_obstacles_path(map_dir))
        raw_entries = payload.get("zones") or payload.get("obstacles") or []
        if not isinstance(raw_entries, list):
            return []
        loaded: list[VirtualObstacleZone] = []
        for entry in raw_entries:
            if not isinstance(entry, dict):
                continue
            try:
                obstacle_id = self._normalize_obstacle_id(str(entry.get("obstacle_id") or entry.get("id") or ""))
                radius = float(entry.get("radius"))
                x = float(entry.get("x"))
                y = float(entry.get("y"))
            except (TypeError, ValueError, StackControlError):
                continue
            if not all(math.isfinite(value) for value in (x, y, radius)) or radius <= 0.0:
                continue
            loaded.append(
                VirtualObstacleZone(
                    obstacle_id=obstacle_id,
                    label=(str(entry.get("label") or obstacle_id)).strip() or obstacle_id,
                    kind="circle_keepout",
                    x=x,
                    y=y,
                    radius=radius,
                    created_at=str(entry.get("created_at") or "") or None,
                    updated_at=str(entry.get("updated_at") or "") or None,
                )
            )
        loaded.sort(key=lambda zone: zone.obstacle_id)
        return loaded

    def _write_virtual_obstacles(self, map_dir: Path, obstacles: list[VirtualObstacleZone]) -> None:
        payload = {
            "version": 1,
            "zones": [
                {
                    "obstacle_id": obstacle.obstacle_id,
                    "label": obstacle.label,
                    "kind": obstacle.kind,
                    "x": float(obstacle.x),
                    "y": float(obstacle.y),
                    "radius": float(obstacle.radius),
                    "created_at": obstacle.created_at,
                    "updated_at": obstacle.updated_at,
                }
                for obstacle in obstacles
            ],
        }
        with self._virtual_obstacles_path(map_dir).open("w", encoding="utf-8") as handle:
            yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=True)

    def _read_map_media_index(self, map_dir: Path) -> ExplicitMediaIndex:
        metadata = self._read_yaml(map_dir / "metadata.yaml")
        candidates: list[Any] = []
        media_index_file = map_dir / "media_index.yaml"
        if media_index_file.exists():
            media_index_payload = self._read_yaml(media_index_file)
            candidates.append(media_index_payload.get("entries"))
        candidates.append(metadata.get("media_entries"))
        candidates.append(metadata.get("media_index", {}).get("entries") if isinstance(metadata.get("media_index"), dict) else None)

        index: ExplicitMediaIndex = {}
        for candidate in candidates:
            if not isinstance(candidate, list):
                continue
            for entry in candidate:
                if not isinstance(entry, dict):
                    continue
                raw_path = str(entry.get("path") or "").strip()
                if not raw_path:
                    continue
                normalized = Path(raw_path).as_posix()
                index[normalized] = entry
        return index

    def _normalize_media_link(self, value: Any, candidates: set[str]) -> str | None:
        if not value:
            return None
        normalized = Path(str(value)).as_posix()
        return normalized if normalized in candidates else None

    def _find_linked_pointcloud_path(self, relative_path: str, candidates: set[str]) -> str | None:
        stem = Path(relative_path).stem
        parent = Path(relative_path).parent
        search_dirs = [
            parent,
            parent / "PCD",
            parent / "pcd",
            Path("PCD"),
            Path("pcd"),
            Path("pointcloud"),
            Path("pointclouds"),
        ]
        for directory in search_dirs:
            candidate = (directory / f"{stem}.pcd").as_posix()
            if candidate in candidates:
                return candidate
        return None

    def _find_linked_image_path(self, relative_path: str, candidates: set[str]) -> str | None:
        stem = Path(relative_path).stem
        parent = Path(relative_path).parent
        search_dirs = [
            parent,
            parent / "images",
            parent / "img",
            Path("images"),
            Path("img"),
        ]
        for directory in search_dirs:
            for suffix in IMAGE_SUFFIXES:
                candidate = (directory / f"{stem}{suffix}").as_posix()
                if candidate in candidates:
                    return candidate
        return None

    def status(self) -> StackStatus:
        processes = self._process_records()
        runtime_state = self._read_runtime_state()
        inferred_mode = self._infer_mode(processes)
        runtime_mode = str(runtime_state.get("mode") or "")
        target_mode = str(runtime_state.get("target_mode") or "")

        if runtime_mode in {"starting", "stopping"}:
            mode = runtime_mode
            expected_mode = target_mode or inferred_mode
        elif runtime_mode in {"mapping", "navigation"}:
            mode = runtime_mode
            expected_mode = runtime_mode
        elif inferred_mode != "stopped":
            mode = inferred_mode
            expected_mode = inferred_mode
        else:
            mode = "stopped"
            expected_mode = "stopped"

        expected = self._expected_nodes_for_mode(expected_mode) if expected_mode in {"mapping", "navigation"} else []
        nodes = [
            NodeCheck(
                key=key,
                label=label,
                state=self._node_state(processes, pattern, mode),
                running=any(self._matches_pattern(proc.args, pattern) for proc in processes),
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
        has_nav = any(
            "bt_navigator" in proc.args
            or "controller_server" in proc.args
            or "jt128_3d_navigation.launch.py" in proc.args
            or "pose_goal_controller_3d" in proc.args
            or "pcd_relocalizer_3d" in proc.args
            for proc in processes
        )
        has_mapping = any(
            self._matches_pattern(proc.args, pattern)
            for proc in processes
            for pattern in ("occupancy_mapper", "native_map_relay", "slam_toolbox", "map_manager_node")
        )
        has_bringup = any("bringup.launch.py" in proc.args for proc in processes)
        if has_nav:
            return "navigation"
        if has_mapping:
            return "mapping"
        if has_bringup:
            return "starting"
        return "stopped"

    def _node_state(self, processes: list[ProcessInfo], pattern: PatternSpec, mode: str) -> str:
        if any(self._matches_pattern(proc.args, pattern) for proc in processes):
            return "running"
        if mode == "starting":
            return "starting"
        if mode == "stopping":
            return "stopping"
        return "missing"

    def _matches_pattern(self, args: str, pattern: PatternSpec) -> bool:
        if isinstance(pattern, tuple):
            return any(token in args for token in pattern)
        return pattern in args

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
        logs = sorted(
            [
                *log_dir.glob("jt128_3d_navigation_*.log"),
                *log_dir.glob("jt128_dlio_mapping_*.log"),
                *log_dir.glob("bringup_real_*.log"),
            ],
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        return str(logs[0]) if logs else None

    def _build_start_timeout_message(self, mode: str, missing_labels: list[str]) -> str:
        detail = "、".join(missing_labels) if missing_labels else "未知节点"
        parts = [f"{'导航' if mode == 'navigation' else '建图'}模式启动超时，缺少节点: {detail}"]
        latest_log = self._latest_log_file()
        if latest_log:
            parts.append(f"日志: {latest_log}")
            excerpt = self._latest_log_excerpt(Path(latest_log))
            if excerpt:
                parts.append(f"摘要: {excerpt}")
        return "；".join(parts)

    def _build_navigation_lifecycle_message(
        self,
        states: dict[str, str],
        failures: list[str] | None = None,
    ) -> str:
        state_text = "、".join(f"{node}={state}" for node, state in states.items()) or "未知"
        parts = [f"导航模式启动异常，生命周期未就绪: {state_text}"]
        if failures:
            parts.append(f"恢复尝试: {'；'.join(failures)}")
        latest_log = self._latest_log_file()
        if latest_log:
            parts.append(f"日志: {latest_log}")
            excerpt = self._latest_log_excerpt(Path(latest_log))
            if excerpt:
                parts.append(f"摘要: {excerpt}")
        return "；".join(parts)

    def _latest_log_excerpt(self, path: Path) -> str:
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            return ""

        tail = [line.strip() for line in lines[-LOG_TAIL_LINE_LIMIT:] if line.strip()]
        if not tail:
            return ""

        highlights = [
            line
            for line in tail
            if any(marker in line.lower() for marker in LOG_HIGHLIGHT_MARKERS)
        ]
        selected = highlights[-LOG_HIGHLIGHT_LIMIT:] if highlights else tail[-LOG_HIGHLIGHT_LIMIT:]
        excerpt = " | ".join(selected)
        if len(excerpt) > LOG_TEXT_LIMIT:
            excerpt = f"...{excerpt[-LOG_TEXT_LIMIT:]}"
        return excerpt

    def _run_ros_shell(self, command: str) -> subprocess.CompletedProcess[str]:
        install_setup = self.workspace / "install" / "setup.bash"
        shell_parts = [
            "source /opt/ros/humble/setup.bash",
            f"source {shlex.quote(str(install_setup))}",
            command,
        ]
        return self._run(["bash", "-lc", " && ".join(shell_parts)])

    def _get_lifecycle_state(self, node_name: str) -> str:
        try:
            result = self._run_ros_shell(f"ros2 lifecycle get /{node_name}")
        except StackControlError as exc:
            return f"query_failed:{exc}"
        output = (result.stdout or result.stderr or "").strip().splitlines()
        return output[-1].strip() if output else "unknown"

    def _set_lifecycle_transition(self, node_name: str, transition: str) -> bool:
        try:
            result = self._run_ros_shell(f"ros2 lifecycle set /{node_name} {transition}")
        except StackControlError:
            return False
        output = ((result.stdout or "") + (result.stderr or "")).lower()
        return "successful" in output

    def _read_yaml(self, path: Path) -> dict[str, Any]:
        if not path.exists():
            return {}
        try:
            return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except Exception:
            return {}

    def _resolve_a2_system_config_dir(self) -> Path:
        candidates = [
            self.workspace / "install" / "a2_system" / "share" / "a2_system" / "config",
            self.workspace / "src" / "a2_system" / "config",
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return candidates[-1]

    def _describe_process(self, process: ProcessInfo) -> str:
        for _, label, pattern in NAVIGATION_NODES + NAVIGATION_NODES_3D + MAPPING_NODES:
            if self._matches_pattern(process.args, pattern):
                return label
        return f"pid={process.pid}"
