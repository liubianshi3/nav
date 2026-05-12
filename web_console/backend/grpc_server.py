from __future__ import annotations

import asyncio
import math
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, AsyncIterator

import grpc

from .grpc_codegen import ensure_grpc_generated
from .models import NavigationGoal, NavigationGoalRequest
from .ros_bridge import RosBridgeError, RosRuntime
from .stack_control import StackControlError, StackController


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


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


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
            node = self.p.ros_runtime.node
            map_id = (request.map_name or "").strip() or (request.session_id or "").strip() or f"map_{int(time.time())}"
            try:
                await asyncio.to_thread(self.p.stack_controller.stop)
            except StackControlError as exc:
                return self.p.laser_navigation_pb2.StopMappingResponse(success=False, map_id="", message=str(exc))
            if bool(request.save_map) and node is not None:
                try:
                    await asyncio.to_thread(node.save_managed_map, map_id)
                except Exception as exc:
                    return self.p.laser_navigation_pb2.StopMappingResponse(success=False, map_id=map_id, message=str(exc))
            return self.p.laser_navigation_pb2.StopMappingResponse(success=True, map_id=map_id, message="ok")

        async def ListMaps(self, request, context):
            maps = await asyncio.to_thread(self.p.stack_controller.list_maps, include_incompatible=True)
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
                        updated_at=_iso_to_ms(m.created_at),
                    )
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

        async def GetMap(self, request, context):
            map_id = (request.map_id or "").strip()
            if not map_id:
                await context.abort(grpc.StatusCode.INVALID_ARGUMENT, "map_id is required")
            map_info = await asyncio.to_thread(self.p.stack_controller.get_map, map_id, include_incompatible=True)
            if map_info is None:
                await context.abort(grpc.StatusCode.NOT_FOUND, "map not found")

            fmt = int(request.format or 0)
            if fmt == self.p.laser_navigation_pb2.MAP_FORMAT_OCCUPANCY_GRID:
                node = await self.p._node_or_abort(context)
                snapshot = node.get_map_snapshot()
                data_bytes = bytes((v & 0xFF) for v in snapshot.data)
                origin = self.p.laser_navigation_pb2.PositionResponse(
                    x=float(snapshot.origin.x),
                    y=float(snapshot.origin.y),
                    theta=float(snapshot.origin.yaw),
                    confidence=1.0,
                    timestamp=_iso_to_ms(snapshot.stamp),
                )
                meta = self.p.laser_navigation_pb2.MapMetadata(
                    width=float(snapshot.width),
                    height=float(snapshot.height),
                    resolution=float(snapshot.resolution),
                    origin=origin,
                )
                return self.p.laser_navigation_pb2.GetMapResponse(
                    map_id=map_id,
                    map_name=map_id,
                    map_data=data_bytes,
                    metadata=meta,
                )

            map_yaml_path = Path(map_info.map_yaml) if map_info.map_yaml else None
            if map_yaml_path is None or not map_yaml_path.exists():
                await context.abort(grpc.StatusCode.FAILED_PRECONDITION, "map.yaml is missing")

            payload = (map_yaml_path.read_text(encoding="utf-8") or "").splitlines()
            image_rel = ""
            for line in payload:
                if ":" not in line:
                    continue
                key, value = line.split(":", 1)
                if key.strip() == "image":
                    image_rel = value.strip().strip("'\"")
                    break
            if not image_rel:
                await context.abort(grpc.StatusCode.FAILED_PRECONDITION, "map.yaml missing image field")

            image_path = (map_yaml_path.parent / image_rel).resolve()
            if not image_path.exists():
                await context.abort(grpc.StatusCode.FAILED_PRECONDITION, "map image not found")

            raw = image_path.read_bytes()
            return self.p.laser_navigation_pb2.GetMapResponse(
                map_id=map_id,
                map_name=map_id,
                map_data=raw,
                metadata=self.p.laser_navigation_pb2.MapMetadata(),
            )

        async def ListMapPresets(self, request, context):
            return self.p.laser_navigation_pb2.ListMapPresetsResponse(presets=[])

        async def GetScanData(self, request, context):
            node = await self.p._node_or_abort(context)
            snapshot = node.build_snapshot(ros_thread_alive=bool(self.p.ros_runtime.thread and self.p.ros_runtime.thread.is_alive()))
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
            from geometry_msgs.msg import Twist

            msg = Twist()
            msg.linear.x = float(request.velocity_x)
            msg.linear.y = float(request.velocity_y)
            msg.angular.z = float(request.angular_velocity)
            node.cancel_stop_publisher.publish(msg)
            return self.p.robot_dog_pb2.MoveResponse(success=True, message="ok", task_id=str(_now_ms()))

        async def Walk(self, request, context):
            node = await self.p._node_or_abort(context)
            from geometry_msgs.msg import Twist

            msg = Twist()
            msg.linear.x = float(request.x)
            msg.linear.y = float(request.y)
            msg.angular.z = float(request.theta)
            node.cancel_stop_publisher.publish(msg)
            return self.p.robot_dog_pb2.WalkResponse(success=True, message="ok", task_id=str(_now_ms()))

        async def Stop(self, request, context):
            node = await self.p._node_or_abort(context)
            from geometry_msgs.msg import Twist

            msg = Twist()
            node.cancel_stop_publisher.publish(msg)
            return self.p.robot_dog_pb2.StopResponse(success=True, message="ok")

        async def GetBattery(self, request, context):
            node = await self.p._node_or_abort(context)
            snapshot = node.build_snapshot(ros_thread_alive=bool(self.p.ros_runtime.thread and self.p.ros_runtime.thread.is_alive()))
            return self.p.robot_dog_pb2.BatteryResponse(
                percentage=_battery_percent(snapshot.battery.percentage),
                is_charging=bool(snapshot.battery.charging),
                estimated_minutes=0,
                health=self.p.robot_dog_pb2.BATTERY_HEALTH_UNSPECIFIED,
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
            return self.p.robot_dog_pb2.PostureResponse(success=False, message="posture control is not available")

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
