from __future__ import annotations

import argparse
import asyncio
import subprocess
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

import uvicorn
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.encoders import jsonable_encoder
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from .config import load_config
from .grpc_server import GrpcServer
from .models import (
    DashboardSnapshot,
    InitialPoseRequest,
    MapMediaListing,
    NavigationGoalRequest,
    RunTaskRouteRequest,
    SaveMapRequest,
    SaveTaskRouteRequest,
    StartNavigationRequest,
    TaskRouteDetail,
    TaskRouteSummary,
    TaskRouteStatus,
    VirtualObstacleListing,
    VirtualObstacleUpsertRequest,
)
from .ros_bridge import RosBridgeError, RosRuntime
from .stack_control import StackControlError, StackController
from .utils import is_lan_or_loopback
from .ws import WebSocketManager


def _route_updated_at(route_path: str | None) -> str | None:
    if not route_path:
        return None
    path = Path(route_path)
    if not path.exists():
        return None
    return datetime.fromtimestamp(path.stat().st_mtime).isoformat()


def _build_route_detail(route_id: str, route_path: str | None, route_yaml: str) -> TaskRouteDetail:
    payload = yaml.safe_load(route_yaml or "") or {}
    if not isinstance(payload, dict):
        payload = {}
    raw_waypoints = payload.get("waypoints")
    waypoint_count = len(raw_waypoints) if isinstance(raw_waypoints, list) else 0
    mission_name = payload.get("mission_name")
    return TaskRouteDetail(
        route_id=route_id,
        route_path=route_path,
        mission_name=str(mission_name).strip() if mission_name else None,
        waypoint_count=waypoint_count,
        updated_at=_route_updated_at(route_path),
        route_yaml=route_yaml,
    )


def _build_route_summary(route_id: str, route_path: str | None, route_yaml: str) -> TaskRouteSummary:
    detail = _build_route_detail(route_id, route_path, route_yaml)
    return TaskRouteSummary(
        route_id=detail.route_id,
        route_path=detail.route_path,
        mission_name=detail.mission_name,
        waypoint_count=detail.waypoint_count,
        updated_at=detail.updated_at,
    )


def _validate_route_yaml_against_virtual_obstacles(
    stack_controller: StackController,
    map_id: str | None,
    route_yaml: str,
    *,
    clearance_m: float,
) -> None:
    normalized_map_id = (map_id or "").strip()
    if not normalized_map_id:
        return
    payload = yaml.safe_load(route_yaml or "") or {}
    if not isinstance(payload, dict):
        raise StackControlError("路线 YAML 必须是包含 waypoints 的映射")
    waypoints = payload.get("waypoints")
    if not isinstance(waypoints, list):
        return
    for index, waypoint in enumerate(waypoints, start=1):
        if not isinstance(waypoint, dict):
            raise StackControlError(f"路线点 #{index} 不是合法映射")
        try:
            x = float(waypoint["x"])
            y = float(waypoint["y"])
        except KeyError as exc:
            raise StackControlError(f"路线点 #{index} 缺少字段: {exc}") from exc
        except (TypeError, ValueError) as exc:
            raise StackControlError(f"路线点 #{index} 坐标不是数字") from exc
        waypoint_id = str(waypoint.get("id") or waypoint.get("name") or f"wp_{index:02d}")
        stack_controller.validate_point_outside_virtual_obstacles(
            normalized_map_id,
            x=x,
            y=y,
            subject=f"路线点 {waypoint_id}",
            padding=clearance_m,
        )


def create_app(config_path: str | None = None) -> FastAPI:
    config = load_config(config_path)
    ws_manager = WebSocketManager()
    ros_runtime = RosRuntime(config, ws_manager)
    stack_controller = StackController(config)
    app = FastAPI(title="A2 Web Console", version="0.1.0")
    app.state.config = config
    app.state.ws_manager = ws_manager
    app.state.ros_runtime = ros_runtime
    app.state.stack_controller = stack_controller

    if config.server.cors_origins:
        app.add_middleware(
            CORSMiddleware,
            allow_origins=config.server.cors_origins,
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

    @app.middleware("http")
    async def lan_guard(request: Request, call_next):
        if config.server.allow_non_lan_access:
            return await call_next(request)
        client_host = request.client.host if request.client else None
        if not is_lan_or_loopback(client_host):
            return JSONResponse({"detail": "仅允许局域网访问"}, status_code=403)
        return await call_next(request)

    @app.on_event("startup")
    async def on_startup() -> None:
        ws_manager.set_loop(asyncio.get_running_loop())
        ros_runtime.start()
        if getattr(config, "grpc", None) is not None and bool(config.grpc.enabled):
            grpc_server = GrpcServer(
                ros_runtime=ros_runtime,
                stack_controller=stack_controller,
                host=str(config.grpc.host),
                port=int(config.grpc.port),
            )
            await grpc_server.start()
            app.state.grpc_server = grpc_server

    @app.on_event("shutdown")
    async def on_shutdown() -> None:
        grpc_server = getattr(app.state, "grpc_server", None)
        if grpc_server is not None:
            await grpc_server.stop()
        ros_runtime.stop()

    @app.get("/api/health")
    async def get_health():
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        health = node.get_health_dict()
        health["ros_thread_alive"] = bool(ros_runtime.thread and ros_runtime.thread.is_alive())
        return health

    @app.get("/api/snapshot", response_model=DashboardSnapshot)
    async def get_snapshot():
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        ros_thread_alive = bool(ros_runtime.thread and ros_runtime.thread.is_alive())
        return node.build_snapshot(ros_thread_alive=ros_thread_alive)

    @app.post("/api/navigation/goal")
    async def send_goal(request: NavigationGoalRequest):
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        if request.map_id:
            try:
                stack_controller.validate_point_outside_virtual_obstacles(
                    request.map_id,
                    x=request.goal.x,
                    y=request.goal.y,
                    subject="导航目标",
                    padding=config.navigation.goal_clearance_m,
                )
            except StackControlError as exc:
                raise HTTPException(status_code=409, detail=str(exc)) from exc
        try:
            state = await asyncio.to_thread(node.send_navigation_goal, request)
        except RosBridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"ok": True, "navigation": jsonable_encoder(state)}

    @app.post("/api/navigation/cancel")
    async def cancel_goal():
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        try:
            state = await asyncio.to_thread(node.cancel_navigation)
        except RosBridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"ok": True, "navigation": jsonable_encoder(state)}

    @app.post("/api/localization/initialpose")
    async def set_initial_pose(request: InitialPoseRequest):
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        if request.map_id:
            try:
                stack_controller.validate_point_outside_virtual_obstacles(
                    request.map_id,
                    x=request.pose.x,
                    y=request.pose.y,
                    subject="初始位姿",
                    padding=config.navigation.initial_pose_clearance_m,
                )
            except StackControlError as exc:
                raise HTTPException(status_code=409, detail=str(exc)) from exc
        try:
            result = await asyncio.to_thread(node.set_initial_pose, request)
        except RosBridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"ok": True, **jsonable_encoder(result)}

    @app.get("/api/stack/status")
    async def get_stack_status():
        return stack_controller.status()

    @app.post("/api/stack/stop")
    async def stop_stack():
        try:
            result = await asyncio.to_thread(stack_controller.stop)
        except StackControlError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"ok": True, **result, "stack": jsonable_encoder(stack_controller.status())}

    @app.post("/api/stack/start-mapping")
    async def start_mapping_stack():
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        mapping_profile = stack_controller.mapping_source_profile()
        native: dict | None = None
        try:
            result = await asyncio.to_thread(stack_controller.start_mapping)
            if mapping_profile == "native_global_map":
                native = await asyncio.to_thread(node.start_native_mapping)
        except RosBridgeError as exc:
            try:
                await asyncio.to_thread(stack_controller.stop)
            except StackControlError:
                pass
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except StackControlError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {
            "ok": True,
            **result,
            "mapping_profile": mapping_profile,
            "native_slam": jsonable_encoder(native) if native is not None else None,
            "stack": jsonable_encoder(stack_controller.status()),
        }

    @app.post("/api/stack/start-navigation")
    async def start_navigation_stack(request: StartNavigationRequest):
        try:
            result = await asyncio.to_thread(stack_controller.start_navigation, request.map_id)
        except StackControlError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"ok": True, **result, "stack": jsonable_encoder(stack_controller.status())}

    @app.get("/api/maps")
    async def list_maps():
        return {"maps": jsonable_encoder(stack_controller.list_maps())}

    @app.get("/api/maps/{map_id}/media", response_model=MapMediaListing)
    async def list_map_media(map_id: str):
        try:
            return await asyncio.to_thread(stack_controller.list_map_media, map_id)
        except StackControlError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @app.get("/api/maps/{map_id}/obstacles", response_model=VirtualObstacleListing)
    async def list_map_obstacles(map_id: str):
        try:
            return await asyncio.to_thread(stack_controller.list_virtual_obstacles, map_id)
        except StackControlError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @app.post("/api/maps/{map_id}/obstacles", response_model=VirtualObstacleListing)
    async def save_map_obstacle(map_id: str, request: VirtualObstacleUpsertRequest):
        try:
            return await asyncio.to_thread(stack_controller.save_virtual_obstacle, map_id, request)
        except StackControlError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.delete("/api/maps/{map_id}/obstacles/{obstacle_id}", response_model=VirtualObstacleListing)
    async def delete_map_obstacle(map_id: str, obstacle_id: str):
        try:
            return await asyncio.to_thread(stack_controller.delete_virtual_obstacle, map_id, obstacle_id)
        except StackControlError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.get("/api/maps/{map_id}/artifacts/{artifact_name}")
    async def get_map_artifact(map_id: str, artifact_name: str):
        map_info = stack_controller.get_map(map_id)
        if map_info is None:
            raise HTTPException(status_code=404, detail=f"地图不存在: {map_id}")
        map_dir = stack_controller.map_root / map_id
        artifact_path = (map_dir / artifact_name).resolve()
        if not artifact_path.exists() or not artifact_path.is_file():
            raise HTTPException(status_code=404, detail=f"资产不存在: {artifact_name}")
        if map_dir.resolve() not in artifact_path.parents:
            raise HTTPException(status_code=403, detail="非法资产路径")
        return FileResponse(artifact_path)

    @app.get("/api/maps/{map_id}/files/{relative_path:path}")
    async def get_map_file(map_id: str, relative_path: str):
        try:
            file_path = await asyncio.to_thread(stack_controller.resolve_map_file, map_id, relative_path)
        except StackControlError as exc:
            detail = str(exc)
            status_code = 403 if "非法" in detail else 404
            raise HTTPException(status_code=status_code, detail=detail) from exc
        return FileResponse(file_path)

    @app.post("/api/stack/transition-to-navigation")
    async def transition_to_navigation(request: SaveMapRequest):
        """Orchestrate mapping → navigation transition in one call.

        Sequence: save map → project PCD→2D → stop mapping → start navigation.
        Each step has timeout + retry. Intermediate state is preserved on failure.
        """
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")

        map_id = request.map_id.strip() or datetime.now().strftime("map_%Y%m%d_%H%M%S")
        steps: list[dict] = []
        t_start = time.monotonic()

        async def _step(name: str, fn, *, timeout: float = 30.0, retries: int = 2) -> bool:
            for attempt in range(retries + 1):
                step_start = time.monotonic()
                try:
                    result = await asyncio.wait_for(
                        asyncio.to_thread(fn), timeout=timeout
                    )
                    steps.append({
                        "step": name, "ok": True, "attempt": attempt + 1,
                        "duration_sec": round(time.monotonic() - step_start, 2),
                    })
                    return True
                except asyncio.TimeoutError:
                    if attempt >= retries:
                        steps.append({
                            "step": name, "ok": False, "attempt": attempt + 1,
                            "error": "timeout",
                            "duration_sec": round(time.monotonic() - step_start, 2),
                        })
                        return False
                    await asyncio.sleep(1.0)
                except Exception as exc:
                    if attempt >= retries:
                        steps.append({
                            "step": name, "ok": False, "attempt": attempt + 1,
                            "error": str(exc)[:200],
                            "duration_sec": round(time.monotonic() - step_start, 2),
                        })
                        return False
                    await asyncio.sleep(1.0)
            return False

        # Step 1: Save map via ROS service
        ok = await _step("save_map", lambda: node.save_managed_map(map_id), timeout=20.0)
        if not ok:
            return {
                "ok": False, "map_id": map_id,
                "message": "保存地图失败",
                "steps": steps,
                "duration_sec": round(time.monotonic() - t_start, 1),
            }

        # Optional Step 2: Evaluate map quality (PCD) → JSON report
        maps_dir = Path(stack_controller.config.map_root).expanduser().resolve()
        map_dir = maps_dir / map_id
        pcd_path = map_dir / "pointcloud_map_3d.pcd"
        workspace = Path(stack_controller.config.workspace).expanduser().resolve()
        quality_tool = workspace / "install" / "a2_system" / "lib" / "a2_system" / "check_map_quality.py"

        if pcd_path.exists() and quality_tool.exists():
            def _run_quality():
                return subprocess.run(
                    [
                        "python3",
                        str(quality_tool),
                        str(pcd_path),
                        "--output",
                        str(map_dir / "map_quality.json"),
                        "--voxel-size",
                        "1.0",
                    ],
                    capture_output=True,
                    text=True,
                    timeout=60,
                )

            ok = await _step("map_quality", _run_quality, timeout=65.0)
            if not ok:
                # Quality failure should not fully block transition; surface in steps
                pass

        # Step 3: Project PCD → 2D
        tool_path = workspace / "install" / "a2_system" / "lib" / "a2_system" / "pcd_to_2d_map.py"

        if pcd_path.exists() and tool_path.exists():
            ok = await _step(
                "project_pcd_to_2d",
                lambda: subprocess.run(
                    ["python3", str(tool_path), str(pcd_path), "--output", str(map_dir), "--resolution", "0.05"],
                    capture_output=True, text=True, timeout=60,
                ),
                timeout=65.0,
            )
            if not ok:
                # Projection failed but map is saved — non-fatal, navigation can use saved 2D map if present
                pass

        # Step 4: Stop mapping stack
        ok = await _step("stop_mapping", stack_controller.stop_if_running, timeout=15.0)
        if not ok:
            return {
                "ok": False, "map_id": map_id,
                "message": "停止建图栈失败",
                "steps": steps,
                "duration_sec": round(time.monotonic() - t_start, 1),
            }

        # Step 5: Start navigation stack
        await asyncio.sleep(2.0)  # Let processes fully exit
        nav_info = None
        try:
            nav_info = await asyncio.to_thread(stack_controller.start_navigation, map_id)
        except StackControlError as exc:
            steps.append({"step": "start_navigation", "ok": False, "error": str(exc)})
            return {
                "ok": False, "map_id": map_id,
                "message": f"导航启动失败: {exc}",
                "steps": steps,
                "duration_sec": round(time.monotonic() - t_start, 1),
            }

        steps.append({
            "step": "start_navigation", "ok": True,
            "nav_info": nav_info,
        })

        return {
            "ok": True,
            "map_id": map_id,
            "map_yaml": str(map_dir / "map.yaml"),
            "map_quality": str((map_dir / "map_quality.json").resolve()) if (map_dir / "map_quality.json").exists() else None,
            "message": f"建图→导航切换完成: {map_id}",
            "steps": steps,
            "duration_sec": round(time.monotonic() - t_start, 1),
        }

    @app.post("/api/maps/save")
    async def save_map(request: SaveMapRequest):
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        native_save: dict | None = None
        mapping_profile = stack_controller.mapping_source_profile()
        try:
            if stack_controller.status().mode == "mapping" and mapping_profile == "native_global_map":
                native_save = await asyncio.to_thread(node.request_native_map_save, request.map_id)
            await asyncio.to_thread(node.save_managed_map, request.map_id)
            if native_save is not None:
                await asyncio.to_thread(
                    stack_controller.attach_native_pointcloud_artifact,
                    request.map_id,
                    native_save["path"],
                    pointcloud_topic_3d=config.ros.pointcloud_topic,
                )
            saved = stack_controller.get_map(request.map_id)
            if saved is None:
                raise StackControlError(f"地图已请求保存，但未在磁盘找到: {request.map_id}")
        except RosBridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        except StackControlError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {
            "ok": True,
            "map": jsonable_encoder(saved),
            "maps": jsonable_encoder(stack_controller.list_maps()),
            "native_slam_save": jsonable_encoder(native_save) if native_save is not None else None,
        }

    @app.post("/api/maps/project-2d")
    async def project_pcd_to_2d(request: SaveMapRequest):
        """Run pcd_to_2d_map.py to generate Nav2-compatible 2D map from saved PCD."""
        maps_dir = Path(stack_controller.config.map_root).expanduser().resolve()
        map_dir = maps_dir / request.map_id
        if not map_dir.exists():
            raise HTTPException(status_code=404, detail=f"地图目录不存在: {map_dir}")
        pcd_path = map_dir / "pointcloud_map_3d.pcd"
        if not pcd_path.exists():
            raise HTTPException(status_code=404, detail=f"PCD文件不存在: {pcd_path}")
        tool_path = Path(stack_controller.config.workspace).expanduser().resolve() / "install" / "a2_system" / "lib" / "a2_system" / "pcd_to_2d_map.py"
        if not tool_path.exists():
            raise HTTPException(status_code=503, detail=f"投影工具未找到: {tool_path}")
        try:
            result = await asyncio.to_thread(
                subprocess.run,
                ["python3", str(tool_path), str(pcd_path), "--output", str(map_dir), "--resolution", "0.05"],
                capture_output=True,
                text=True,
                timeout=60,
            )
        except subprocess.TimeoutExpired:
            raise HTTPException(status_code=500, detail="投影超时（>60秒）")
        if result.returncode != 0:
            raise HTTPException(status_code=500, detail=f"投影失败: {result.stderr.strip() or result.stdout.strip()}")
        map_yaml = map_dir / "map.yaml"
        return {
            "ok": True,
            "map_id": request.map_id,
            "map_yaml": str(map_yaml),
            "navigation_ready": map_yaml.exists(),
            "stdout": result.stdout.strip(),
        }

    @app.post("/api/maps/quality")
    async def evaluate_map_quality(request: SaveMapRequest):
        """Run check_map_quality.py on saved PCD and return JSON report content."""
        maps_dir = Path(stack_controller.config.map_root).expanduser().resolve()
        map_dir = maps_dir / request.map_id
        if not map_dir.exists():
            raise HTTPException(status_code=404, detail=f"地图目录不存在: {map_dir}")
        pcd_path = map_dir / "pointcloud_map_3d.pcd"
        if not pcd_path.exists():
            raise HTTPException(status_code=404, detail=f"PCD文件不存在: {pcd_path}")
        tool_path = Path(stack_controller.config.workspace).expanduser().resolve() / "install" / "a2_system" / "lib" / "a2_system" / "check_map_quality.py"
        if not tool_path.exists():
            raise HTTPException(status_code=503, detail=f"质量评估工具未找到: {tool_path}")

        try:
            result = await asyncio.to_thread(
                subprocess.run,
                [
                    "python3",
                    str(tool_path),
                    str(pcd_path),
                    "--output",
                    str(map_dir / "map_quality.json"),
                    "--voxel-size",
                    "1.0",
                ],
                capture_output=True,
                text=True,
                timeout=60,
            )
        except subprocess.TimeoutExpired:
            raise HTTPException(status_code=500, detail="质量评估超时（>60秒）")
        if result.returncode != 0:
            raise HTTPException(status_code=500, detail=f"质量评估失败: {result.stderr.strip() or result.stdout.strip()}")
        report_path = map_dir / "map_quality.json"
        report = json.loads(report_path.read_text(encoding="utf-8")) if report_path.exists() else None
        return {
            "ok": True,
            "map_id": request.map_id,
            "report_path": str(report_path) if report_path else None,
            "report": report,
            "stdout": result.stdout.strip(),
        }

    @app.get("/api/tasks/routes")
    async def list_task_routes():
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        try:
            route_ids = await asyncio.to_thread(node.task_list_routes)
            summaries: list[TaskRouteSummary] = []
            for route_id in route_ids:
                detail = await asyncio.to_thread(node.task_get_route, route_id)
                summaries.append(
                    _build_route_summary(detail["route_id"], detail["route_path"], detail["route_yaml"])
                )
            status = await asyncio.to_thread(node.task_route_status)
        except RosBridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"routes": jsonable_encoder(summaries), "status": jsonable_encoder(status)}

    @app.get("/api/tasks/routes/status", response_model=TaskRouteStatus)
    async def get_task_route_status():
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        try:
            return await asyncio.to_thread(node.task_route_status)
        except RosBridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.get("/api/tasks/routes/{route_id}", response_model=TaskRouteDetail)
    async def get_task_route(route_id: str):
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        try:
            detail = await asyncio.to_thread(node.task_get_route, route_id)
        except RosBridgeError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        return _build_route_detail(detail["route_id"], detail["route_path"], detail["route_yaml"])

    @app.post("/api/tasks/routes", response_model=TaskRouteDetail)
    async def save_task_route(request: SaveTaskRouteRequest):
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        try:
            _validate_route_yaml_against_virtual_obstacles(
                stack_controller,
                request.map_id,
                request.route_yaml,
                clearance_m=config.navigation.goal_clearance_m,
            )
            detail = await asyncio.to_thread(node.task_save_route, request.route_id, request.route_yaml)
        except (RosBridgeError, StackControlError) as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return _build_route_detail(detail["route_id"], detail["route_path"], detail["route_yaml"])

    @app.delete("/api/tasks/routes/{route_id}")
    async def delete_task_route(route_id: str):
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        try:
            items = await asyncio.to_thread(node.task_delete_route, route_id)
            status = await asyncio.to_thread(node.task_route_status)
        except RosBridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"ok": True, "items": items, "status": jsonable_encoder(status)}

    @app.post("/api/tasks/routes/run", response_model=TaskRouteStatus)
    async def run_task_route(request: RunTaskRouteRequest):
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        try:
            detail = await asyncio.to_thread(node.task_get_route, request.route_id)
            _validate_route_yaml_against_virtual_obstacles(
                stack_controller,
                request.map_id,
                detail["route_yaml"],
                clearance_m=config.navigation.goal_clearance_m,
            )
            await asyncio.to_thread(
                node.task_run_route,
                route_id=request.route_id,
                mission_name=request.mission_name or "",
                dry_run=request.dry_run,
                stop_on_failure=request.stop_on_failure,
                save_map_on_finish=request.save_map_on_finish,
                save_map_on_failure=request.save_map_on_failure,
            )
            return await asyncio.to_thread(node.task_route_status)
        except (RosBridgeError, StackControlError) as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/tasks/routes/stop", response_model=TaskRouteStatus)
    async def stop_task_route():
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        try:
            return await asyncio.to_thread(node.task_stop_route)
        except RosBridgeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.websocket(config.server.websocket_path)
    async def websocket_endpoint(websocket: WebSocket):
        if not config.server.allow_non_lan_access and not is_lan_or_loopback(websocket.client.host if websocket.client else None):
            await websocket.close(code=1008)
            return
        await ws_manager.connect(websocket)
        try:
            node = ros_runtime.node
            if node is not None:
                ros_thread_alive = bool(ros_runtime.thread and ros_runtime.thread.is_alive())
                snapshot = node.build_snapshot(ros_thread_alive=ros_thread_alive)
                await websocket.send_json({"type": "snapshot", "payload": jsonable_encoder(snapshot)})
            while True:
                await websocket.receive_text()
        except WebSocketDisconnect:
            await ws_manager.disconnect(websocket)
        except Exception:
            await ws_manager.disconnect(websocket)

    static_dir = config.static_dir
    index_file = static_dir / "index.html"
    assets_dir = static_dir / "assets"
    if index_file.exists():
        if assets_dir.exists():
            app.mount("/assets", StaticFiles(directory=assets_dir), name="assets")

        @app.get("/")
        async def root():
            return FileResponse(index_file)

        @app.get("/{full_path:path}")
        async def spa_fallback(full_path: str):
            if full_path.startswith("api/") or full_path == config.server.websocket_path.lstrip("/"):
                raise HTTPException(status_code=404, detail="Not found")
            candidate = static_dir / full_path
            if candidate.exists() and candidate.is_file():
                return FileResponse(candidate)
            return FileResponse(index_file)
    else:
        @app.get("/")
        async def root_placeholder():
            return {
                "message": "前端静态文件尚未构建。请先执行 scripts/build_frontend.sh。",
                "static_dir": str(static_dir),
            }

    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="A2 web console backend")
    parser.add_argument("--config", default=None, help="Path to YAML config")
    parser.add_argument("--host", default=None, help="Override listen host")
    parser.add_argument("--port", type=int, default=None, help="Override listen port")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = load_config(args.config)
    if args.host:
        config.server.host = args.host
    if args.port:
        config.server.port = args.port
    app = create_app(str(config.config_path) if config.config_path else args.config)
    uvicorn.run(
        app,
        host=config.server.host,
        port=config.server.port,
    )


if __name__ == "__main__":
    main()
