from __future__ import annotations

import asyncio
import io
import logging
import math
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, AsyncIterator

import grpc
from PIL import Image as PilImage

from .grpc_codegen import ensure_grpc_generated
from .map_formats import _occupancy_bytes_from_nav2_luma, _parse_nav2_map_yaml, _read_pgm_luma
from .models import (
    InitialPoseRequest,
    ManualVelocityCommand,
    NavigationGoal,
    NavigationGoalRequest,
    StartNavigationRequest,
)
from .motion_control import ensure_manual_motion_authorized
from .ros_bridge import RosBridgeError, RosRuntime
from .stack_control import StackControlError, StackController


logger = logging.getLogger(__name__)


def _now_ms() -> int:
    return int(time.time() * 1000)


def _iso_to_ms(value: str | None) -> int:
    if not value:
        return _now_ms()
    try:
        return int(datetime.fromisoformat(value).timestamp() * 1000)
    except Exception:
        return _now_ms()


def _battery_percent(value: float | None) -> int:
    if value is None or not math.isfinite(value):
        return 0
    if value <= 1.0:
        return int(max(0.0, min(100.0, value * 100.0)))
    return int(max(0.0, min(100.0, value)))


def _log_battery_missing(*, reason: str, extra: dict[str, Any]) -> None:
    logger.warning("battery missing: reason=%s extra=%s", reason, extra)


def _requires_motion_command(*values: float) -> bool:
    return any(abs(float(value)) > 1e-6 for value in values)


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _pointcloud_snapshot_to_ascii_pcd(pointcloud: Any) -> tuple[bytes, int]:
    points = getattr(pointcloud, "points", None) or []
    valid_points: list[tuple[float, float, float]] = []
    for point in points:
        if len(point) < 2:
            continue
        try:
            x = float(point[0])
            y = float(point[1])
            z = float(point[2]) if len(point) >= 3 else 0.0
        except Exception:
            continue
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            valid_points.append((x, y, z))

    if not valid_points:
        return b"", 0

    handle = io.StringIO()
    handle.write("# .PCD v0.7 - Point Cloud Data file format\n")
    handle.write("VERSION 0.7\n")
    handle.write("FIELDS x y z\n")
    handle.write("SIZE 4 4 4\n")
    handle.write("TYPE F F F\n")
    handle.write("COUNT 1 1 1\n")
    handle.write(f"WIDTH {len(valid_points)}\n")
    handle.write("HEIGHT 1\n")
    handle.write("VIEWPOINT 0 0 0 1 0 0 0\n")
    handle.write(f"POINTS {len(valid_points)}\n")
    handle.write("DATA ascii\n")
    for x, y, z in valid_points:
        handle.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
    return handle.getvalue().encode("ascii"), len(valid_points)


@dataclass
class _RegistryState:
    upstreams: dict[str, dict[str, Any]] = field(default_factory=dict)


@dataclass
class _AlarmState:
    alarms: dict[str, dict[str, Any]] = field(default_factory=dict)


@dataclass
class _LightState:
    status_by_device: dict[str, dict[str, Any]] = field(default_factory=dict)


class A2GrpcServices:
    def __init__(self, *, ros_runtime: RosRuntime, stack_controller: StackController) -> None:
        ensure_grpc_generated()
        from common import alarm_pb2, light_pb2, registry_pb2
        from common import alarm_pb2_grpc, light_pb2_grpc, registry_pb2_grpc
        from device import laser_navigation_pb2, robot_dog_pb2
        from device import laser_navigation_pb2_grpc, robot_dog_pb2_grpc

        self.alarm_pb2 = alarm_pb2
        self.light_pb2 = light_pb2
        self.registry_pb2 = registry_pb2
        self.laser_navigation_pb2 = laser_navigation_pb2
        self.robot_dog_pb2 = robot_dog_pb2

        self.alarm_pb2_grpc = alarm_pb2_grpc
        self.light_pb2_grpc = light_pb2_grpc
        self.registry_pb2_grpc = registry_pb2_grpc
        self.laser_navigation_pb2_grpc = laser_navigation_pb2_grpc
        self.robot_dog_pb2_grpc = robot_dog_pb2_grpc

        self.ros_runtime = ros_runtime
        self.stack_controller = stack_controller
        self.registry_state = _RegistryState()
        self.alarm_state = _AlarmState()
        self.light_state = _LightState()
        self.robot_mode: dict[str, int] = {}

    def add_to_server(self, server: grpc.aio.Server) -> None:
        self.alarm_pb2_grpc.add_AlarmServiceServicer_to_server(self._AlarmService(self), server)
        self.light_pb2_grpc.add_LightServiceServicer_to_server(self._LightService(self), server)
        self.registry_pb2_grpc.add_RegistryServiceServicer_to_server(self._RegistryService(self), server)
        self.laser_navigation_pb2_grpc.add_LaserNavigationServiceServicer_to_server(
            self._LaserNavigationService(self),
            server,
        )
        self.robot_dog_pb2_grpc.add_RobotDogServiceServicer_to_server(self._RobotDogService(self), server)

    async def _node_or_abort(self, context: grpc.aio.ServicerContext):
        node = self.ros_runtime.node
        if node is None:
            await context.abort(grpc.StatusCode.UNAVAILABLE, "ROS runtime is not started")
        return node

    class _RegistryService:
        def __init__(self, parent: "A2GrpcServices") -> None:
            self.p = parent

        async def Register(self, request, context):
            upstream = request.upstream
            addr = (upstream.addr or "").strip()
            if not addr:
                await context.abort(grpc.StatusCode.INVALID_ARGUMENT, "upstream.addr is required")
            payload = {
                "name": upstream.name,
                "type": upstream.type,
                "version": upstream.version,
                "addr": addr,
                "services": list(upstream.services),
                "meta": [
                    {"key": int(entry.key), "value": entry.value, "custom_key": entry.custom_key}
                    for entry in upstream.meta
                ],
            }
            self.p.registry_state.upstreams[addr] = payload
            return self.p.registry_pb2.RegistryRegisterResponse(upstream=upstream)

        async def Unregister(self, request, context):
            addr = (request.addr or "").strip()
            ok = bool(addr and self.p.registry_state.upstreams.pop(addr, None) is not None)
            return self.p.registry_pb2.RegistryUnregisterResponse(ok=ok)

        async def List(self, request, context):
            items = []
            for value in self.p.registry_state.upstreams.values():
                meta = [
                    self.p.registry_pb2.UpstreamMetaEntry(
                        key=value_entry.get("key", 0),
                        value=value_entry.get("value", ""),
                        custom_key=value_entry.get("custom_key", ""),
                    )
                    for value_entry in value.get("meta", [])
                ]
                items.append(
                    self.p.registry_pb2.UpstreamInfo(
                        name=value.get("name", ""),
                        type=value.get("type", ""),
                        version=value.get("version", ""),
                        addr=value.get("addr", ""),
                        services=list(value.get("services", [])),
                        meta=meta,
                    )
                )
            return self.p.registry_pb2.RegistryListResponse(
                upstreams=items,
                proxy_grpc_listen="",
                proxy_grpc_port=0,
            )

    class _AlarmService:
        def __init__(self, parent: "A2GrpcServices") -> None:
            self.p = parent

        def _derive_current_alarms(self, device_id: str) -> list[Any]:
            node = self.p.ros_runtime.node
            alarms: list[Any] = []
            if node is None:
                return alarms

            snapshot = node.build_snapshot(ros_thread_alive=bool(self.p.ros_runtime.thread and self.p.ros_runtime.thread.is_alive()))
            if snapshot.health.last_error:
                alarms.append(
                    self.p.alarm_pb2.Alarm(
                        id="health_last_error",
                        device_id=device_id,
                        severity=self.p.alarm_pb2.ALARM_SEVERITY_ERROR,
                        state=self.p.alarm_pb2.ALARM_STATE_ACTIVE,
                        title="Backend Health Error",
                        description=str(snapshot.health.last_error),
                        source="web_console.backend",
                        triggered_at=_now_ms(),
                        metadata={"ros_connected": str(bool(snapshot.health.ros_connected))},
                    )
                )
            if snapshot.status.system_ready is False:
                alarms.append(
                    self.p.alarm_pb2.Alarm(
                        id="system_not_ready",
                        device_id=device_id,
                        severity=self.p.alarm_pb2.ALARM_SEVERITY_WARNING,
                        state=self.p.alarm_pb2.ALARM_STATE_ACTIVE,
                        title="System Not Ready",
                        description="Robot system_ready is false",
                        source="a2_system",
                        triggered_at=_now_ms(),
                    )
                )
            return alarms

        async def GetAlarms(self, request, context):
            device_id = (request.device_id or "").strip() or "a2"
            alarms = self._derive_current_alarms(device_id)
            return self.p.alarm_pb2.GetAlarmsResponse(
                alarms=alarms,
                total=len(alarms),
                page=int(request.page or 0),
                page_size=int(request.page_size or 0),
            )

        async def AcknowledgeAlarm(self, request, context):
            alarm_id = (request.alarm_id or "").strip()
            if not alarm_id:
                await context.abort(grpc.StatusCode.INVALID_ARGUMENT, "alarm_id is required")
            derived = {a.id: a for a in self._derive_current_alarms("a2")}
            alarm = derived.get(alarm_id)
            if alarm is None:
                await context.abort(grpc.StatusCode.NOT_FOUND, "alarm not found")
            updated = self.p.alarm_pb2.Alarm()
            updated.CopyFrom(alarm)
            updated.state = self.p.alarm_pb2.ALARM_STATE_ACKNOWLEDGED
            updated.acknowledged_at = _now_ms()
            updated.acknowledged_by = request.acknowledged_by or ""
            return updated

        async def ClearAlarm(self, request, context):
            alarm_id = (request.alarm_id or "").strip()
            if not alarm_id:
                await context.abort(grpc.StatusCode.INVALID_ARGUMENT, "alarm_id is required")
            derived = {a.id: a for a in self._derive_current_alarms("a2")}
            alarm = derived.get(alarm_id)
            if alarm is None:
                await context.abort(grpc.StatusCode.NOT_FOUND, "alarm not found")
            updated = self.p.alarm_pb2.Alarm()
            updated.CopyFrom(alarm)
            updated.state = self.p.alarm_pb2.ALARM_STATE_CLEARED
            updated.cleared_at = _now_ms()
            updated.cleared_by = request.cleared_by or ""
            return updated

        async def WatchAlarms(self, request, context) -> AsyncIterator[Any]:
            device_id = (request.device_id or "").strip() or "a2"
            last_ids: set[str] = set()
            min_sev = int(request.min_severity or 0)
            while True:
                alarms = [a for a in self._derive_current_alarms(device_id) if int(a.severity) >= min_sev]
                for alarm in alarms:
                    if alarm.id not in last_ids:
                        last_ids.add(alarm.id)
                        yield alarm
                await asyncio.sleep(1.0)

        async def GetAlarmStatistics(self, request, context):
            device_id = (request.device_id or "").strip() or "a2"
            alarms = self._derive_current_alarms(device_id)
            total = len(alarms)
            critical = sum(1 for a in alarms if a.severity == self.p.alarm_pb2.ALARM_SEVERITY_CRITICAL)
            warning = sum(1 for a in alarms if a.severity == self.p.alarm_pb2.ALARM_SEVERITY_WARNING)
            info = sum(1 for a in alarms if a.severity == self.p.alarm_pb2.ALARM_SEVERITY_INFO)
            active = sum(1 for a in alarms if a.state == self.p.alarm_pb2.ALARM_STATE_ACTIVE)
            ack = sum(1 for a in alarms if a.state == self.p.alarm_pb2.ALARM_STATE_ACKNOWLEDGED)
            cleared = sum(1 for a in alarms if a.state == self.p.alarm_pb2.ALARM_STATE_CLEARED)
            return self.p.alarm_pb2.GetAlarmStatisticsResponse(
                total_count=total,
                critical_count=critical,
                warning_count=warning,
                info_count=info,
                active_count=active,
                acknowledged_count=ack,
                cleared_count=cleared,
                by_device={device_id: total},
            )

    class _LightService:
        def __init__(self, parent: "A2GrpcServices") -> None:
            self.p = parent

        def _status_from_cache(self, device_id: str) -> Any:
            cached = self.p.light_state.status_by_device.get(device_id) or {}
            rgb = cached.get("rgb") or {}
            return self.p.light_pb2.LightStatus(
                device_id=device_id,
                on=bool(cached.get("on", False)),
                intensity=int(cached.get("intensity", 0)),
                color_mode=int(cached.get("color_mode", 0)),
                rgb=self.p.light_pb2.LightColor(
                    r=int(rgb.get("r", 0)),
                    g=int(rgb.get("g", 0)),
                    b=int(rgb.get("b", 0)),
                ),
                color_temperature_kelvin=int(cached.get("color_temperature_kelvin", 0)),
                timestamp=int(cached.get("timestamp", 0)) or _now_ms(),
            )

        async def GetLightStatus(self, request, context):
            device_id = (request.device_id or "").strip() or "a2"
            return self._status_from_cache(device_id)

        async def SetLight(self, request, context):
            device_id = (request.device_id or "").strip() or "a2"
            intensity = int(getattr(request, "intensity", 0) or 0)
            color_mode = int(getattr(request, "color_mode", 0) or 0)
            rgb = getattr(request, "rgb", None)
            payload = {
                "device_id": device_id,
                "on": bool(getattr(request, "on", False)),
                "intensity": int(max(0, min(255, intensity))),
                "color_mode": int(color_mode),
                "rgb": {
                    "r": int(max(0, min(255, int(getattr(rgb, "r", 0) or 0)))),
                    "g": int(max(0, min(255, int(getattr(rgb, "g", 0) or 0)))),
                    "b": int(max(0, min(255, int(getattr(rgb, "b", 0) or 0)))),
                },
                "color_temperature_kelvin": int(max(0, min(65535, int(getattr(request, "color_temperature_kelvin", 0) or 0)))),
                "timestamp": _now_ms(),
            }
            self.p.light_state.status_by_device[device_id] = payload

            success = True
            message = "ok"
            node = self.p.ros_runtime.node
            if node is None:
                success = False
                message = "ROS runtime is not started"
            else:
                try:
                    node.set_light(
                        device_id=device_id,
                        on=payload["on"],
                        intensity=payload["intensity"],
                        color_mode=payload["color_mode"],
                        r=payload["rgb"]["r"],
                        g=payload["rgb"]["g"],
                        b=payload["rgb"]["b"],
                        color_temperature_kelvin=payload["color_temperature_kelvin"],
                    )
                except RosBridgeError as exc:
                    success = False
                    message = str(exc)
            return self.p.light_pb2.SetLightResponse(
                success=success,
                message=message,
                status=self._status_from_cache(device_id),
            )

    class _LaserNavigationService:
        def __init__(self, parent: "A2GrpcServices") -> None:
            self.p = parent

        async def GetPosition(self, request, context):
            node = await self.p._node_or_abort(context)
            snapshot = node.build_snapshot(ros_thread_alive=bool(self.p.ros_runtime.thread and self.p.ros_runtime.thread.is_alive()))
            x = float(snapshot.pose.x or 0.0)
            y = float(snapshot.pose.y or 0.0)
            theta = float(snapshot.pose.yaw or 0.0)
            confidence = 1.0 if snapshot.status.localization_ok else 0.0
            return self.p.laser_navigation_pb2.PositionResponse(
                x=x,
                y=y,
                theta=theta,
                confidence=float(confidence),
                timestamp=_iso_to_ms(snapshot.pose.stamp),
            )

        async def SetTarget(self, request, context):
            node = await self.p._node_or_abort(context)
            goal = NavigationGoal(
                x=float(request.target_x),
                y=float(request.target_y),
                yaw=float(request.target_theta),
                frame_id="map",
            )
            try:
                state = await asyncio.to_thread(node.send_navigation_goal, NavigationGoalRequest(goal=goal, map_id=None))
            except RosBridgeError as exc:
                return self.p.laser_navigation_pb2.SetTargetResponse(success=False, message=str(exc), task_id="", estimated_time=0.0)
            estimated = 0.0
            if isinstance(state.feedback, dict):
                value = state.feedback.get("estimated_time_remaining_sec")
                if value is not None:
                    try:
                        estimated = float(value)
                    except Exception:
                        estimated = 0.0
            return self.p.laser_navigation_pb2.SetTargetResponse(
                success=True,
                message=str(state.message or "ok"),
                task_id=str(state.updated_at or ""),
                estimated_time=estimated,
            )

        async def CancelTarget(self, request, context):
            node = await self.p._node_or_abort(context)
            try:
                state = await asyncio.to_thread(node.cancel_navigation)
            except RosBridgeError as exc:
                return self.p.laser_navigation_pb2.CancelTargetResponse(success=False, message=str(exc))
            return self.p.laser_navigation_pb2.CancelTargetResponse(success=True, message=str(state.message or "ok"))

        async def StartMapping(self, request, context):
            try:
                result = await asyncio.to_thread(self.p.stack_controller.start_mapping)
            except StackControlError as exc:
                return self.p.laser_navigation_pb2.StartMappingResponse(success=False, session_id="", message=str(exc))
            return self.p.laser_navigation_pb2.StartMappingResponse(success=True, session_id="mapping", message=str(result.get("message", "ok")))

        async def StopMapping(self, request, context):
            map_id = (request.map_name or "").strip() or (request.session_id or "").strip() or f"map_{int(time.time())}"
            if bool(request.save_map):
                node = await self.p._node_or_abort(context)
                try:
                    await asyncio.to_thread(node.save_managed_map, map_id)
                except Exception as exc:
                    return self.p.laser_navigation_pb2.StopMappingResponse(success=False, map_id=map_id, message=str(exc))
            try:
                result = await asyncio.to_thread(self.p.stack_controller.stop)
            except StackControlError as exc:
                return self.p.laser_navigation_pb2.StopMappingResponse(success=False, map_id=map_id, message=str(exc))
            return self.p.laser_navigation_pb2.StopMappingResponse(
                success=True,
                map_id=map_id,
                message=str((result or {}).get("message") or "ok"),
            )

        async def ListMaps(self, request, context):
            start = time.monotonic()
            peer = None
            try:
                peer = context.peer()
            except Exception:
                peer = None
            device_id = (getattr(request, "device_id", "") or "").strip()
            logger.info(
                "grpc ListMaps start peer=%s device_id=%s map_root=%s include_incompatible=%s",
                peer,
                device_id or "-",
                str(self.p.stack_controller.map_root),
                True,
            )
            try:
                maps = await asyncio.to_thread(self.p.stack_controller.list_maps, include_incompatible=True)
            except Exception:
                duration_ms = int((time.monotonic() - start) * 1000)
                logger.exception(
                    "grpc ListMaps failed peer=%s device_id=%s duration_ms=%s",
                    peer,
                    device_id or "-",
                    duration_ms,
                )
                raise
            items = []
            for m in maps:
                mapping_type = self.p.laser_navigation_pb2.MAPPING_TYPE_UNSPECIFIED
                if str(m.representation or "") == "pointcloud_map_3d" or bool(m.has_pointcloud_3d):
                    mapping_type = self.p.laser_navigation_pb2.MAPPING_TYPE_3D
                else:
                    mapping_type = self.p.laser_navigation_pb2.MAPPING_TYPE_2D
                items.append(
                    self.p.laser_navigation_pb2.MapInfo(
                        map_id=m.map_id,
                        map_name=m.map_id,
                        mapping_type=mapping_type,
                        created_at=_iso_to_ms(m.created_at),
                        updated_at=(
                            int((self.p.stack_controller.map_root / m.map_id / "metadata.yaml").stat().st_mtime * 1000)
                            if (self.p.stack_controller.map_root / m.map_id / "metadata.yaml").exists()
                            else _iso_to_ms(m.created_at)
                        ),
                    )
                )
            duration_ms = int((time.monotonic() - start) * 1000)
            logger.info(
                "grpc ListMaps ok peer=%s device_id=%s count=%s duration_ms=%s",
                peer,
                device_id or "-",
                len(items),
                duration_ms,
            )
            return self.p.laser_navigation_pb2.ListMapsResponse(maps=items)

        async def SelectMap(self, request, context):
            map_id = (request.map_id or "").strip()
            if not map_id:
                await context.abort(grpc.StatusCode.INVALID_ARGUMENT, "map_id is required")
            try:
                result = await asyncio.to_thread(self.p.stack_controller.start_navigation, map_id)
            except StackControlError as exc:
                return self.p.laser_navigation_pb2.SelectMapResponse(success=False, message=str(exc), current_map_id="")
            return self.p.laser_navigation_pb2.SelectMapResponse(success=True, message=str(result.get("message", "ok")), current_map_id=map_id)

        async def StartNavigation(self, request, context):
            map_id = (request.map_id or "").strip()
            if not map_id:
                await context.abort(grpc.StatusCode.INVALID_ARGUMENT, "map_id is required")
            stack_request = StartNavigationRequest(
                map_id=map_id,
                localization_mode=(request.localization_mode or "").strip() or "ndt",
                motion_mode=(request.motion_mode or "").strip() or "live_motion",
                enable_nav2_3d=bool(request.enable_nav2_3d),
                collision_monitor_profile=(request.collision_monitor_profile or "").strip() or "strict",
            )
            try:
                result = await asyncio.to_thread(self.p.stack_controller.start_navigation_from_request, stack_request)
            except StackControlError as exc:
                return self.p.laser_navigation_pb2.StartNavigationResponse(success=False, message=str(exc), current_map_id="")
            return self.p.laser_navigation_pb2.StartNavigationResponse(
                success=True,
                message=str(result.get("message", "ok")),
                current_map_id=map_id,
            )

        async def SetInitialPose(self, request, context):
            node = await self.p._node_or_abort(context)
            frame_id = (request.frame_id or "").strip() or "map"
            pose = NavigationGoal(
                x=float(request.x),
                y=float(request.y),
                yaw=float(request.theta),
                frame_id=frame_id,
            )
            map_id = (request.map_id or "").strip() or None
            validator = getattr(self.p.stack_controller, "validate_point_outside_virtual_obstacles", None)
            if map_id and callable(validator):
                navigation_config = getattr(getattr(node, "config", None), "navigation", None)
                padding = float(getattr(navigation_config, "initial_pose_clearance_m", 0.0) or 0.0)
                try:
                    await asyncio.to_thread(
                        validator,
                        map_id,
                        x=pose.x,
                        y=pose.y,
                        subject="初始位姿",
                        padding=padding,
                    )
                except StackControlError as exc:
                    return self.p.laser_navigation_pb2.SetInitialPoseResponse(success=False, message=str(exc))
            try:
                result = await asyncio.to_thread(node.set_initial_pose, InitialPoseRequest(pose=pose, map_id=map_id))
            except RosBridgeError as exc:
                return self.p.laser_navigation_pb2.SetInitialPoseResponse(success=False, message=str(exc))
            result_pose = result.get("pose") if isinstance(result, dict) else None
            response_pose = self.p.laser_navigation_pb2.PositionResponse(
                x=float((result_pose or {}).get("x", pose.x)),
                y=float((result_pose or {}).get("y", pose.y)),
                theta=float((result_pose or {}).get("theta", (result_pose or {}).get("yaw", pose.yaw))),
                confidence=1.0,
                timestamp=_now_ms(),
            )
            message = str(result.get("message", "ok")) if isinstance(result, dict) else "ok"
            return self.p.laser_navigation_pb2.SetInitialPoseResponse(success=True, message=message, pose=response_pose)

        async def GetMap(self, request, context):
            map_id = (request.map_id or "").strip()
            if not map_id:
                await context.abort(grpc.StatusCode.INVALID_ARGUMENT, "map_id is required")
            map_info = await asyncio.to_thread(self.p.stack_controller.get_map, map_id, include_incompatible=True)
            if map_info is None:
                await context.abort(grpc.StatusCode.NOT_FOUND, "map not found")

            fmt = int(request.format or 0)
            map_yaml_path = Path(map_info.map_yaml) if map_info.map_yaml else None
            if map_yaml_path is None or not map_yaml_path.exists():
                await context.abort(grpc.StatusCode.FAILED_PRECONDITION, "map.yaml is missing")
            try:
                image_rel, resolution, origin_tuple = _parse_nav2_map_yaml(map_yaml_path)
            except Exception as exc:
                await context.abort(grpc.StatusCode.FAILED_PRECONDITION, str(exc))
            image_path = (map_yaml_path.parent / image_rel).resolve()
            if not image_path.exists():
                await context.abort(grpc.StatusCode.FAILED_PRECONDITION, "map image not found")

            res = float(resolution or (map_info.resolution or 0.0) or 0.0)
            origin = self.p.laser_navigation_pb2.PositionResponse(
                x=float(origin_tuple[0]),
                y=float(origin_tuple[1]),
                theta=float(origin_tuple[2]),
                confidence=1.0,
                timestamp=0,
            )

            if fmt in (0, self.p.laser_navigation_pb2.MAP_FORMAT_PGM):
                if image_path.suffix.lower() == ".pgm":
                    width, height, _ = _read_pgm_luma(image_path)
                    raw = image_path.read_bytes()
                else:
                    img = PilImage.open(image_path).convert("L")
                    width, height = img.size
                    raw = image_path.read_bytes()
                meta = self.p.laser_navigation_pb2.MapMetadata(
                    width=float(width),
                    height=float(height),
                    resolution=res,
                    origin=origin,
                )
                return self.p.laser_navigation_pb2.GetMapResponse(
                    map_id=map_id,
                    map_name=map_id,
                    map_data=raw,
                    metadata=meta,
                )

            if fmt == self.p.laser_navigation_pb2.MAP_FORMAT_PNG:
                img = PilImage.open(image_path).convert("L")
                width, height = img.size
                buf = io.BytesIO()
                img.save(buf, format="PNG")
                meta = self.p.laser_navigation_pb2.MapMetadata(
                    width=float(width),
                    height=float(height),
                    resolution=res,
                    origin=origin,
                )
                return self.p.laser_navigation_pb2.GetMapResponse(
                    map_id=map_id,
                    map_name=map_id,
                    map_data=buf.getvalue(),
                    metadata=meta,
                )

            if fmt == self.p.laser_navigation_pb2.MAP_FORMAT_OCCUPANCY_GRID:
                if image_path.suffix.lower() == ".pgm":
                    width, height, luma = _read_pgm_luma(image_path)
                else:
                    img = PilImage.open(image_path).convert("L")
                    width, height = img.size
                    luma = img.tobytes()
                data_bytes = _occupancy_bytes_from_nav2_luma(width, height, luma)
                meta = self.p.laser_navigation_pb2.MapMetadata(
                    width=float(width),
                    height=float(height),
                    resolution=res,
                    origin=origin,
                )
                return self.p.laser_navigation_pb2.GetMapResponse(
                    map_id=map_id,
                    map_name=map_id,
                    map_data=data_bytes,
                    metadata=meta,
                )

            raw = image_path.read_bytes()
            return self.p.laser_navigation_pb2.GetMapResponse(
                map_id=map_id,
                map_name=map_id,
                map_data=raw,
                metadata=self.p.laser_navigation_pb2.MapMetadata(origin=origin, resolution=res),
            )

        async def ListMapPresets(self, request, context):
            return self.p.laser_navigation_pb2.ListMapPresetsResponse(presets=[])

        async def _build_scan_data_response(self, request, context):
            node = await self.p._node_or_abort(context)
            snapshot = node.build_snapshot(ros_thread_alive=bool(self.p.ros_runtime.thread and self.p.ros_runtime.thread.is_alive()))
            if bool(request.include_pcd):
                pointcloud = snapshot.pointcloud
                if hasattr(node, "get_navigation_pointcloud_snapshot"):
                    pointcloud = node.get_navigation_pointcloud_snapshot()
                pcd_data, pcd_points_count = _pointcloud_snapshot_to_ascii_pcd(pointcloud)
                if pcd_data:
                    return self.p.laser_navigation_pb2.ScanDataResponse(
                        ranges=[],
                        angles=[],
                        points_count=pcd_points_count,
                        angle_min=0.0,
                        angle_max=0.0,
                        range_min=0.0,
                        range_max=0.0,
                        timestamp=_iso_to_ms(getattr(pointcloud, "stamp", None)),
                        pcd_data=pcd_data,
                        pcd_url="",
                    )

            points = snapshot.pointcloud.points if snapshot.pointcloud.loaded else []
            angles: list[float] = []
            ranges: list[float] = []
            a_min = math.radians(float(request.angle_min))
            a_max = math.radians(float(request.angle_max))
            if a_max < a_min:
                a_min, a_max = a_max, a_min
            for p in points:
                if len(p) < 2:
                    continue
                x, y = float(p[0]), float(p[1])
                ang = math.atan2(y, x)
                if ang < a_min or ang > a_max:
                    continue
                r = math.hypot(x, y)
                angles.append(float(ang))
                ranges.append(float(r))
            return self.p.laser_navigation_pb2.ScanDataResponse(
                ranges=ranges,
                angles=angles,
                points_count=len(ranges),
                angle_min=float(a_min),
                angle_max=float(a_max),
                range_min=0.0,
                range_max=max(ranges) if ranges else 0.0,
                timestamp=_iso_to_ms(snapshot.pointcloud.stamp),
                pcd_data=b"",
                pcd_url="",
            )

        async def GetScanData(self, request, context):
            return await self._build_scan_data_response(request, context)

        async def WatchScanData(self, request, context) -> AsyncIterator[Any]:
            interval_ms = int(getattr(request, "interval_ms", 0) or 400)
            interval_ms = max(100, min(5000, interval_ms))
            last_key: tuple[int, int, int] | None = None

            while True:
                response = await self._build_scan_data_response(request, context)
                has_payload = bool(getattr(response, "pcd_data", b"")) or bool(getattr(response, "ranges", []))
                key = (int(response.timestamp), int(response.points_count), len(response.pcd_data))
                if has_payload and key != last_key:
                    last_key = key
                    yield response
                await asyncio.sleep(interval_ms / 1000.0)

        async def GetPath(self, request, context):
            return self.p.laser_navigation_pb2.GetPathResponse(
                success=False,
                path=[],
                total_distance=0.0,
                estimated_time=0.0,
                error_message="path planning is not available",
            )

        async def WatchNavigationStatus(self, request, context) -> AsyncIterator[Any]:
            device_id = (request.device_id or "").strip() or "a2"
            while True:
                node = self.p.ros_runtime.node
                if node is None:
                    await asyncio.sleep(1.0)
                    continue
                snapshot = node.build_snapshot(ros_thread_alive=bool(self.p.ros_runtime.thread and self.p.ros_runtime.thread.is_alive()))
                pos = self.p.laser_navigation_pb2.PositionResponse(
                    x=float(snapshot.pose.x or 0.0),
                    y=float(snapshot.pose.y or 0.0),
                    theta=float(snapshot.pose.yaw or 0.0),
                    confidence=1.0 if snapshot.status.localization_ok else 0.0,
                    timestamp=_iso_to_ms(snapshot.pose.stamp),
                )
                target = None
                if snapshot.navigation.goal is not None:
                    target = self.p.laser_navigation_pb2.PositionResponse(
                        x=float(snapshot.navigation.goal.x),
                        y=float(snapshot.navigation.goal.y),
                        theta=float(snapshot.navigation.goal.yaw),
                        confidence=1.0,
                        timestamp=_now_ms(),
                    )
                state = self.p.laser_navigation_pb2.NAVIGATION_STATE_IDLE
                nav_state = str(snapshot.navigation.state or "").lower()
                if nav_state == "navigating":
                    state = self.p.laser_navigation_pb2.NAVIGATION_STATE_NAVIGATING
                elif nav_state in {"failed", "canceled"}:
                    state = self.p.laser_navigation_pb2.NAVIGATION_STATE_ERROR
                battery_level = float(_battery_percent(snapshot.battery.percentage)) / 100.0
                is_moving = bool((snapshot.status.velocity_linear_x or 0.0) != 0.0 or (snapshot.status.velocity_angular_z or 0.0) != 0.0)
                yield self.p.laser_navigation_pb2.NavigationStatus(
                    device_id=device_id,
                    current_position=pos,
                    state=state,
                    is_moving=is_moving,
                    task_id=str(snapshot.navigation.updated_at or ""),
                    target_position=target,
                    remaining_path=[],
                    battery_level=battery_level,
                    timestamp=_now_ms(),
                )
                await asyncio.sleep(0.5)

    class _RobotDogService:
        def __init__(self, parent: "A2GrpcServices") -> None:
            self.p = parent

        def _motion_response(self, response_type: Any, result: Any):
            return response_type(
                success=bool(getattr(result, "success", False)),
                message=str(getattr(result, "message", "") or ""),
                sdk_code=int(getattr(result, "sdk_code", 0)),
                error_code=str(getattr(result, "error_code", "") or ""),
                runtime_mode=str(getattr(result, "runtime_mode", "") or ""),
                state=str(getattr(result, "state", "") or ""),
            )

        def _motion_failure_response(self, response_type: Any, message: str, error_code: str = "ros_bridge_error"):
            return response_type(
                success=False,
                message=message,
                sdk_code=0,
                error_code=error_code,
                runtime_mode="",
                state="",
            )

        def _motion_auth_constant(self, name: str, fallback: int) -> int:
            return int(getattr(self.p.robot_dog_pb2, name, fallback))

        def _motion_auth_response(self, response_type: Any, **values: Any):
            return response_type(
                success=bool(values.get("success", False)),
                message=str(values.get("message", "") or ""),
                error_code=str(values.get("error_code", "") or ""),
                state=int(values.get("state", 0) or 0),
                required_action=int(values.get("required_action", 0) or 0),
                standing=bool(values.get("standing", False)),
                motion_authorized=bool(values.get("motion_authorized", False)),
                manual_start_required=bool(values.get("manual_start_required", False)),
                motion_mode=int(values.get("motion_mode", 0) or 0),
                gait_type=int(values.get("gait_type", 0) or 0),
                runtime_mode=str(values.get("runtime_mode", "") or ""),
                timestamp=int(values.get("timestamp", _now_ms()) or _now_ms()),
                sdk_code=int(values.get("sdk_code", 0) or 0),
            )

        def _snapshot_and_control_state(self, node: Any) -> tuple[Any | None, Any | None, Any | None]:
            snapshot = None
            if hasattr(node, "build_snapshot"):
                snapshot = node.build_snapshot(
                    ros_thread_alive=bool(self.p.ros_runtime.thread and self.p.ros_runtime.thread.is_alive())
                )
            status = getattr(snapshot, "status", None)
            raw_state = getattr(status, "raw_state", None)
            control_state = getattr(node, "control_state", None) or getattr(snapshot, "control_state", None)
            return snapshot, raw_state, control_state

        def _infer_motion_authorization(self, node: Any) -> dict[str, Any]:
            _, raw_state, control_state = self._snapshot_and_control_state(node)
            unknown_state = self._motion_auth_constant("MOTION_AUTHORIZATION_STATE_UNKNOWN", 1)
            stand_down_state = self._motion_auth_constant("MOTION_AUTHORIZATION_STATE_STAND_DOWN", 2)
            standing_not_authorized_state = self._motion_auth_constant(
                "MOTION_AUTHORIZATION_STATE_STANDING_NOT_AUTHORIZED",
                3,
            )
            authorized_state = self._motion_auth_constant("MOTION_AUTHORIZATION_STATE_AUTHORIZED", 5)
            moving_state = self._motion_auth_constant("MOTION_AUTHORIZATION_STATE_MOVING", authorized_state)
            none_action = self._motion_auth_constant("MOTION_AUTHORIZATION_ACTION_NONE", 1)
            stand_up_action = self._motion_auth_constant("MOTION_AUTHORIZATION_ACTION_STAND_UP", 2)

            runtime_mode = str(getattr(control_state, "runtime_mode", "") or "")
            last_command = str(getattr(control_state, "last_command", "") or "").lower()
            last_error_code = str(getattr(control_state, "last_error_code", "") or "")
            try:
                sdk_code = int(getattr(control_state, "last_sdk_code", 0) or 0)
            except Exception:
                sdk_code = 0

            motion_mode = 0
            gait_type = 0
            if raw_state is not None:
                try:
                    motion_mode = int(getattr(raw_state, "motion_mode", 0) or 0)
                except Exception:
                    motion_mode = 0
                try:
                    gait_type = int(getattr(raw_state, "gait_type", 0) or 0)
                except Exception:
                    gait_type = 0
            if gait_type == 0:
                try:
                    gait_type = int(getattr(control_state, "gait_type", 0) or 0)
                except Exception:
                    gait_type = 0

            if raw_state is not None and getattr(raw_state, "connected", True) is False:
                return {
                    "success": False,
                    "message": "motion authorization state unavailable: robot state is disconnected",
                    "error_code": "state_unavailable",
                    "state": unknown_state,
                    "required_action": none_action,
                    "standing": False,
                    "motion_authorized": False,
                    "manual_start_required": False,
                    "motion_mode": motion_mode,
                    "gait_type": gait_type,
                    "runtime_mode": runtime_mode,
                    "sdk_code": sdk_code,
                }

            body_height = getattr(raw_state, "body_height", None)
            standing: bool | None
            last_command_ok = sdk_code == 0 and last_error_code in {"", "ok"}
            standing_motion_modes = {0, 1, 2, 3, 8}
            nonstanding_motion_modes = {5, 7, 10}
            if motion_mode in nonstanding_motion_modes:
                standing = False
            elif motion_mode in standing_motion_modes:
                standing = True
            elif last_command_ok and last_command in {"stand_down", "damp"}:
                standing = False
            elif last_command_ok and last_command in {
                "balance_stand",
                "body_height",
                "move",
                "recovery_stand",
                "set_auto_recovery",
                "speed_level",
                "stand_up",
                "switch_gait",
                "walk",
            }:
                standing = True
            elif isinstance(body_height, (int, float)) and math.isfinite(float(body_height)) and float(body_height) > 0.2:
                standing = True
            else:
                standing = None

            if standing is None:
                return {
                    "success": False,
                    "message": "motion authorization state unavailable",
                    "error_code": "state_unavailable",
                    "state": unknown_state,
                    "required_action": none_action,
                    "standing": False,
                    "motion_authorized": False,
                    "manual_start_required": False,
                    "motion_mode": motion_mode,
                    "gait_type": gait_type,
                    "runtime_mode": runtime_mode,
                    "sdk_code": sdk_code,
                }

            if not standing:
                return {
                    "success": False,
                    "message": "stand up before requesting motion authorization",
                    "error_code": "stand_up_required",
                    "state": stand_down_state,
                    "required_action": stand_up_action,
                    "standing": False,
                    "motion_authorized": False,
                    "manual_start_required": False,
                    "motion_mode": motion_mode,
                    "gait_type": gait_type,
                    "runtime_mode": runtime_mode,
                    "sdk_code": sdk_code,
                }

            locomotion_ready_modes = {3}
            motion_authorized = (
                motion_mode in locomotion_ready_modes
                or (last_command in {"balance_stand", "move", "walk"} and sdk_code == 0 and not last_error_code)
            )
            if motion_authorized:
                return {
                    "success": True,
                    "message": "motion authorization available",
                    "error_code": "ok",
                    "state": moving_state if last_command in {"move", "walk"} else authorized_state,
                    "required_action": none_action,
                    "standing": True,
                    "motion_authorized": True,
                    "manual_start_required": False,
                    "motion_mode": motion_mode,
                    "gait_type": gait_type,
                    "runtime_mode": runtime_mode,
                    "sdk_code": sdk_code,
                }

            return {
                "success": False,
                "message": "call AuthorizeMotion after the robot is standing",
                "error_code": "motion_authorization_required",
                "state": standing_not_authorized_state,
                "required_action": none_action,
                "standing": True,
                "motion_authorized": False,
                "manual_start_required": False,
                "motion_mode": motion_mode,
                "gait_type": gait_type,
                "runtime_mode": runtime_mode,
                "sdk_code": sdk_code,
            }

        def _publish_zero_velocity(self, node: Any) -> None:
            from geometry_msgs.msg import Twist

            msg = Twist()
            for publisher_name in ("cancel_stop_publisher", "direct_cmd_publisher"):
                publisher = getattr(node, publisher_name, None)
                if publisher is not None:
                    publisher.publish(msg)

        async def _call_motion_command(
            self,
            context: grpc.aio.ServicerContext,
            *,
            command: str,
            int_value: int = 0,
            float_value: float = 0.0,
            bool_value: bool = False,
        ) -> Any:
            node = await self.p._node_or_abort(context)
            if not hasattr(node, "call_motion_command"):
                raise RosBridgeError("a2 motion command bridge is not available")
            return await asyncio.to_thread(
                node.call_motion_command,
                command,
                int(int_value),
                float(float_value),
                bool(bool_value),
            )

        async def _motion_rpc(
            self,
            request: Any,
            context: grpc.aio.ServicerContext,
            response_type: Any,
            *,
            command: str,
            int_value: int = 0,
            float_value: float = 0.0,
            bool_value: bool = False,
        ) -> Any:
            try:
                result = await self._call_motion_command(
                    context,
                    command=command,
                    int_value=int_value,
                    float_value=float_value,
                    bool_value=bool_value,
                )
            except RosBridgeError as exc:
                return self._motion_failure_response(response_type, str(exc))
            return self._motion_response(response_type, result)

        async def SetMode(self, request, context):
            device_id = (request.device_id or "").strip() or "a2"
            self.p.robot_mode[device_id] = int(request.mode or 0)
            return self.p.robot_dog_pb2.SetModeResponse(success=True, message="ok")

        async def GetMode(self, request, context):
            device_id = (request.device_id or "").strip() or "a2"
            mode = int(self.p.robot_mode.get(device_id, self.p.robot_dog_pb2.ROBOT_DOG_MODE_API))
            return self.p.robot_dog_pb2.GetModeResponse(mode=mode)

        async def Move(self, request, context):
            node = await self.p._node_or_abort(context)
            try:
                if _requires_motion_command(request.velocity_x, request.velocity_y, request.angular_velocity):
                    await asyncio.to_thread(self.p.stack_controller.ensure_manual_control_standby)
                    await asyncio.to_thread(ensure_manual_motion_authorized, node)
                result = await asyncio.to_thread(
                    node.publish_manual_velocity,
                    ManualVelocityCommand(
                        linear_x=float(request.velocity_x),
                        linear_y=float(request.velocity_y),
                        angular_z=float(request.angular_velocity),
                    ),
                )
            except StackControlError as exc:
                return self.p.robot_dog_pb2.MoveResponse(success=False, message=str(exc), task_id="")
            except RosBridgeError as exc:
                return self.p.robot_dog_pb2.MoveResponse(success=False, message=str(exc), task_id="")
            return self.p.robot_dog_pb2.MoveResponse(
                success=True,
                message=str(getattr(result, "message", "") or "ok"),
                task_id=str(_now_ms()),
            )

        async def Walk(self, request, context):
            node = await self.p._node_or_abort(context)
            try:
                if _requires_motion_command(request.x, request.y, request.theta):
                    await asyncio.to_thread(self.p.stack_controller.ensure_manual_control_standby)
                    await asyncio.to_thread(ensure_manual_motion_authorized, node)
                result = await asyncio.to_thread(
                    node.publish_manual_velocity,
                    ManualVelocityCommand(
                        linear_x=float(request.x),
                        linear_y=float(request.y),
                        angular_z=float(request.theta),
                    ),
                )
            except StackControlError as exc:
                return self.p.robot_dog_pb2.WalkResponse(success=False, message=str(exc), task_id="")
            except RosBridgeError as exc:
                return self.p.robot_dog_pb2.WalkResponse(success=False, message=str(exc), task_id="")
            return self.p.robot_dog_pb2.WalkResponse(
                success=True,
                message=str(getattr(result, "message", "") or "ok"),
                task_id=str(_now_ms()),
            )

        async def Stop(self, request, context):
            node = await self.p._node_or_abort(context)
            try:
                result = await asyncio.to_thread(
                    node.publish_manual_velocity,
                    ManualVelocityCommand(linear_x=0.0, linear_y=0.0, angular_z=0.0),
                )
            except RosBridgeError as exc:
                return self.p.robot_dog_pb2.StopResponse(success=False, message=str(exc))
            return self.p.robot_dog_pb2.StopResponse(
                success=True,
                message=str(getattr(result, "message", "") or "ok"),
            )

        async def GetBattery(self, request, context):
            node = await self.p._node_or_abort(context)
            device_id = (request.device_id or "").strip() or "a2"
            ros_thread_alive = bool(self.p.ros_runtime.thread and self.p.ros_runtime.thread.is_alive())
            snapshot = node.build_snapshot(ros_thread_alive=ros_thread_alive)
            battery_stale = bool(getattr(snapshot.battery, "stale", False))
            has_data = bool(snapshot.battery.available) and not battery_stale and snapshot.battery.percentage is not None
            if not has_data:
                reason = "unknown"
                if not ros_thread_alive:
                    reason = "ros_thread_dead"
                elif not bool(snapshot.battery.available):
                    reason = "battery_not_present"
                elif battery_stale:
                    reason = "battery_stale"
                elif snapshot.battery.percentage is None:
                    reason = "battery_percentage_missing"
                _log_battery_missing(
                    reason=reason,
                    extra={
                        "device_id": device_id,
                        "ros_thread_alive": ros_thread_alive,
                        "battery_available": bool(snapshot.battery.available),
                        "battery_stale": battery_stale,
                        "battery_percentage": snapshot.battery.percentage,
                        "battery_voltage": snapshot.battery.voltage,
                        "battery_charging": snapshot.battery.charging,
                        "battery_stamp": snapshot.battery.stamp,
                    },
                )
            health_code = snapshot.battery.health
            health = self.p.robot_dog_pb2.BATTERY_HEALTH_UNSPECIFIED
            if health_code == 1:
                health = self.p.robot_dog_pb2.BATTERY_HEALTH_GOOD
            elif health_code in {2, 3, 4, 5, 6, 7, 8}:
                health = self.p.robot_dog_pb2.BATTERY_HEALTH_POOR
            return self.p.robot_dog_pb2.BatteryResponse(
                percentage=_battery_percent(snapshot.battery.percentage) if has_data else -1,
                is_charging=bool(snapshot.battery.charging) if has_data else False,
                estimated_minutes=0 if has_data else -1,
                health=health if has_data else self.p.robot_dog_pb2.BATTERY_HEALTH_UNSPECIFIED,
            )

        async def GetPose(self, request, context):
            node = await self.p._node_or_abort(context)
            snapshot = node.build_snapshot(ros_thread_alive=bool(self.p.ros_runtime.thread and self.p.ros_runtime.thread.is_alive()))
            x = y = z = 0.0
            yaw = 0.0
            if snapshot.status.raw_state and snapshot.status.raw_state.position:
                pos = snapshot.status.raw_state.position
                if len(pos) >= 1:
                    x = float(pos[0])
                if len(pos) >= 2:
                    y = float(pos[1])
                if len(pos) >= 3:
                    z = float(pos[2])
            if snapshot.status.raw_state and snapshot.status.raw_state.rpy and len(snapshot.status.raw_state.rpy) >= 3:
                yaw = float(snapshot.status.raw_state.rpy[2])
            qx, qy, qz, qw = _yaw_to_quat(yaw)
            return self.p.robot_dog_pb2.PoseResponse(
                position_x=x,
                position_y=y,
                position_z=z,
                rotation_x=qx,
                rotation_y=qy,
                rotation_z=qz,
                rotation_w=qw,
            )

        async def SetPosture(self, request, context):
            posture = int(request.posture or 0)
            if posture == int(self.p.robot_dog_pb2.POSTURE_TYPE_STAND):
                try:
                    result = await self._call_motion_command(context, command="stand_up")
                except RosBridgeError as exc:
                    return self.p.robot_dog_pb2.PostureResponse(success=False, message=str(exc))
                return self.p.robot_dog_pb2.PostureResponse(
                    success=bool(getattr(result, "success", False)),
                    message=str(getattr(result, "message", "") or "ok"),
                )
            if posture == int(self.p.robot_dog_pb2.POSTURE_TYPE_LIE):
                try:
                    result = await self._call_motion_command(context, command="stand_down")
                except RosBridgeError as exc:
                    return self.p.robot_dog_pb2.PostureResponse(success=False, message=str(exc))
                return self.p.robot_dog_pb2.PostureResponse(
                    success=bool(getattr(result, "success", False)),
                    message=str(getattr(result, "message", "") or "ok"),
                )
            return self.p.robot_dog_pb2.PostureResponse(
                success=False,
                message="unsupported posture; use StandUp, StandDown, BalanceStand, RecoveryStand, or Damp",
            )

        async def GetSensors(self, request, context):
            node = await self.p._node_or_abort(context)
            snapshot = node.build_snapshot(ros_thread_alive=bool(self.p.ros_runtime.thread and self.p.ros_runtime.thread.is_alive()))
            sensors = []
            if snapshot.status.raw_state:
                if snapshot.status.raw_state.body_height is not None:
                    sensors.append(self.p.robot_dog_pb2.SensorData(name="body_height", value=float(snapshot.status.raw_state.body_height), unit="m"))
                if snapshot.status.raw_state.yaw_speed is not None:
                    sensors.append(self.p.robot_dog_pb2.SensorData(name="yaw_speed", value=float(snapshot.status.raw_state.yaw_speed), unit="rad/s"))
            return self.p.robot_dog_pb2.SensorsResponse(sensors=sensors, timestamp=_now_ms())

        async def WatchRobotDogStatus(self, request, context) -> AsyncIterator[Any]:
            device_id = (request.device_id or "").strip() or "a2"
            while True:
                node = self.p.ros_runtime.node
                if node is None:
                    await asyncio.sleep(1.0)
                    continue
                pose = await self.GetPose(self.p.robot_dog_pb2.PoseRequest(device_id=device_id), context)
                battery = await self.GetBattery(self.p.robot_dog_pb2.BatteryRequest(device_id=device_id), context)
                sensors = await self.GetSensors(self.p.robot_dog_pb2.SensorsRequest(device_id=device_id), context)
                mode = int(self.p.robot_mode.get(device_id, self.p.robot_dog_pb2.ROBOT_DOG_MODE_API))
                yield self.p.robot_dog_pb2.RobotDogStatus(
                    device_id=device_id,
                    pose=pose,
                    battery=battery,
                    sensors=list(sensors.sensors),
                    state=self.p.robot_dog_pb2.ROBOT_DOG_STATE_IDLE,
                    timestamp=_now_ms(),
                    mode=mode,
                )
                await asyncio.sleep(1.0)

        async def BalanceStand(self, request, context):
            return await self._motion_rpc(
                request,
                context,
                self.p.robot_dog_pb2.BalanceStandResponse,
                command="balance_stand",
            )

        async def StandUp(self, request, context):
            return await self._motion_rpc(
                request,
                context,
                self.p.robot_dog_pb2.StandUpResponse,
                command="stand_up",
            )

        async def StandDown(self, request, context):
            return await self._motion_rpc(
                request,
                context,
                self.p.robot_dog_pb2.StandDownResponse,
                command="stand_down",
            )

        async def RecoveryStand(self, request, context):
            return await self._motion_rpc(
                request,
                context,
                self.p.robot_dog_pb2.RecoveryStandResponse,
                command="recovery_stand",
            )

        async def Damp(self, request, context):
            return await self._motion_rpc(
                request,
                context,
                self.p.robot_dog_pb2.DampResponse,
                command="damp",
            )

        async def SetAutoRecovery(self, request, context):
            return await self._motion_rpc(
                request,
                context,
                self.p.robot_dog_pb2.SetAutoRecoveryResponse,
                command="set_auto_recovery",
                bool_value=bool(request.enabled),
            )

        async def SwitchGait(self, request, context):
            return await self._motion_rpc(
                request,
                context,
                self.p.robot_dog_pb2.SwitchGaitResponse,
                command="switch_gait",
                int_value=int(request.gait_type),
            )

        async def SetSpeedLevel(self, request, context):
            return await self._motion_rpc(
                request,
                context,
                self.p.robot_dog_pb2.SetSpeedLevelResponse,
                command="speed_level",
                int_value=int(request.level),
            )

        async def SetBodyHeight(self, request, context):
            return await self._motion_rpc(
                request,
                context,
                self.p.robot_dog_pb2.SetBodyHeightResponse,
                command="body_height",
                float_value=float(request.height),
            )

        async def GetControlState(self, request, context):
            node = await self.p._node_or_abort(context)
            device_id = (request.device_id or "").strip() or "a2"
            state = getattr(node, "control_state", None)
            if state is None and hasattr(node, "build_snapshot"):
                snapshot = node.build_snapshot(
                    ros_thread_alive=bool(self.p.ros_runtime.thread and self.p.ros_runtime.thread.is_alive())
                )
                state = getattr(snapshot, "control_state", None)
            if state is None:
                state = type("ControlStateFallback", (), {})()
            return self.p.robot_dog_pb2.GetControlStateResponse(
                device_id=device_id,
                runtime_mode=str(getattr(state, "runtime_mode", "") or ""),
                state=str(getattr(state, "state", "") or ""),
                ready=bool(getattr(state, "ready", False)),
                reason=str(getattr(state, "reason", "") or ""),
                interface_name=str(getattr(state, "interface_name", "") or ""),
                gait_control_enabled=bool(getattr(state, "gait_control_enabled", False)),
                gait_type=int(getattr(state, "gait_type", 0)),
                speed_level=int(getattr(state, "speed_level", 0)),
                body_height=float(getattr(state, "body_height", 0.0)),
                auto_recovery=bool(getattr(state, "auto_recovery", False)),
                last_command=str(getattr(state, "last_command", "") or ""),
                last_sdk_code=int(getattr(state, "last_sdk_code", 0)),
                last_error_code=str(getattr(state, "last_error_code", "") or ""),
                last_error_reason=str(getattr(state, "last_error_reason", "") or ""),
                timestamp=_iso_to_ms(getattr(state, "stamp", None)),
            )

        async def GetMotionAuthorization(self, request, context):
            node = await self.p._node_or_abort(context)
            values = self._infer_motion_authorization(node)
            values["timestamp"] = _now_ms()
            return self._motion_auth_response(self.p.robot_dog_pb2.GetMotionAuthorizationResponse, **values)

        async def AuthorizeMotion(self, request, context):
            node = await self.p._node_or_abort(context)
            values = self._infer_motion_authorization(node)
            if not values.get("standing", False) or values.get("motion_authorized", False):
                values["timestamp"] = _now_ms()
                return self._motion_auth_response(self.p.robot_dog_pb2.AuthorizeMotionResponse, **values)

            try:
                await asyncio.to_thread(self.p.stack_controller.ensure_manual_control_standby)
                result = await self._call_motion_command(context, command="balance_stand")
            except (RosBridgeError, StackControlError) as exc:
                return self._motion_auth_response(
                    self.p.robot_dog_pb2.AuthorizeMotionResponse,
                    success=False,
                    message=str(exc),
                    error_code="stack_control_error" if isinstance(exc, StackControlError) else "ros_bridge_error",
                    state=values.get("state", 0),
                    required_action=values.get("required_action", 0),
                    standing=values.get("standing", False),
                    motion_authorized=False,
                    manual_start_required=False,
                    motion_mode=values.get("motion_mode", 0),
                    gait_type=values.get("gait_type", 0),
                    runtime_mode=values.get("runtime_mode", ""),
                    timestamp=_now_ms(),
                    sdk_code=0,
                )

            success = bool(getattr(result, "success", False))
            error_code = str(getattr(result, "error_code", "") or ("ok" if success else "motion_authorization_failed"))
            return self._motion_auth_response(
                self.p.robot_dog_pb2.AuthorizeMotionResponse,
                success=success,
                message=str(
                    getattr(result, "message", "")
                    or ("motion authorization accepted: balance_stand" if success else "motion authorization failed")
                ),
                error_code=error_code,
                state=self._motion_auth_constant("MOTION_AUTHORIZATION_STATE_AUTHORIZED", 5)
                if success
                else values.get("state", 0),
                required_action=self._motion_auth_constant("MOTION_AUTHORIZATION_ACTION_NONE", 1),
                standing=True,
                motion_authorized=success,
                manual_start_required=False,
                motion_mode=values.get("motion_mode", 0),
                gait_type=values.get("gait_type", 0),
                runtime_mode=str(getattr(result, "runtime_mode", "") or values.get("runtime_mode", "")),
                timestamp=_now_ms(),
                sdk_code=int(getattr(result, "sdk_code", 0) or 0),
            )

        async def ReleaseMotionAuthorization(self, request, context):
            node = await self.p._node_or_abort(context)
            current = self._infer_motion_authorization(node)
            self._publish_zero_velocity(node)
            try:
                result = await self._call_motion_command(context, command="stop")
            except RosBridgeError as exc:
                return self._motion_auth_response(
                    self.p.robot_dog_pb2.ReleaseMotionAuthorizationResponse,
                    success=False,
                    message=str(exc),
                    error_code="ros_bridge_error",
                    state=current.get("state", 0),
                    required_action=self._motion_auth_constant("MOTION_AUTHORIZATION_ACTION_STOP", 4),
                    standing=current.get("standing", False),
                    motion_authorized=False,
                    manual_start_required=False,
                    motion_mode=current.get("motion_mode", 0),
                    gait_type=current.get("gait_type", 0),
                    runtime_mode=current.get("runtime_mode", ""),
                    timestamp=_now_ms(),
                    sdk_code=0,
                )
            success = bool(getattr(result, "success", False))
            error_code = str(getattr(result, "error_code", "") or ("ok" if success else "motion_stop_failed"))
            return self._motion_auth_response(
                self.p.robot_dog_pb2.ReleaseMotionAuthorizationResponse,
                success=success,
                message=str(getattr(result, "message", "") or ("motion stop accepted" if success else "motion stop failed")),
                error_code=error_code,
                state=self._motion_auth_constant("MOTION_AUTHORIZATION_STATE_STANDING_NOT_AUTHORIZED", 3)
                if current.get("standing", False)
                else current.get("state", 0),
                required_action=self._motion_auth_constant("MOTION_AUTHORIZATION_ACTION_STOP", 4),
                standing=current.get("standing", False),
                motion_authorized=False,
                manual_start_required=False,
                motion_mode=current.get("motion_mode", 0),
                gait_type=current.get("gait_type", 0),
                runtime_mode=str(getattr(result, "runtime_mode", "") or current.get("runtime_mode", "")),
                timestamp=_now_ms(),
                sdk_code=int(getattr(result, "sdk_code", 0) or 0),
            )


class GrpcServer:
    def __init__(self, *, ros_runtime: RosRuntime, stack_controller: StackController, host: str, port: int) -> None:
        self.ros_runtime = ros_runtime
        self.stack_controller = stack_controller
        self.host = host
        self.port = int(port)
        self._server: grpc.aio.Server | None = None

    async def start(self) -> None:
        if self._server is not None:
            return
        server = grpc.aio.server()
        services = A2GrpcServices(ros_runtime=self.ros_runtime, stack_controller=self.stack_controller)
        services.add_to_server(server)
        server.add_insecure_port(f"{self.host}:{self.port}")
        await server.start()
        self._server = server

    async def stop(self) -> None:
        if self._server is None:
            return
        await self._server.stop(grace=2)
        self._server = None
