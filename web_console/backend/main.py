from __future__ import annotations

import argparse
import asyncio

import uvicorn
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.encoders import jsonable_encoder
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from .config import load_config
from .models import DashboardSnapshot, InitialPoseRequest, NavigationGoalRequest, SaveMapRequest, StartNavigationRequest
from .ros_bridge import RosBridgeError, RosRuntime
from .stack_control import StackControlError, StackController
from .utils import is_lan_or_loopback
from .ws import WebSocketManager


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

    @app.on_event("shutdown")
    async def on_shutdown() -> None:
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
        return node.build_snapshot()

    @app.post("/api/navigation/goal")
    async def send_goal(request: NavigationGoalRequest):
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
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
        try:
            result = await asyncio.to_thread(stack_controller.start_mapping)
        except StackControlError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"ok": True, **result, "stack": jsonable_encoder(stack_controller.status())}

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

    @app.post("/api/maps/save")
    async def save_map(request: SaveMapRequest):
        node = ros_runtime.node
        if node is None:
            raise HTTPException(status_code=503, detail="ROS runtime 未启动")
        try:
            saved = await asyncio.to_thread(stack_controller.save_map, request.map_id, node.get_map_snapshot())
        except StackControlError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"ok": True, "map": jsonable_encoder(saved), "maps": jsonable_encoder(stack_controller.list_maps())}

    @app.websocket(config.server.websocket_path)
    async def websocket_endpoint(websocket: WebSocket):
        if not config.server.allow_non_lan_access and not is_lan_or_loopback(websocket.client.host if websocket.client else None):
            await websocket.close(code=1008)
            return
        await ws_manager.connect(websocket)
        try:
            node = ros_runtime.node
            if node is not None:
                snapshot = node.build_snapshot()
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
