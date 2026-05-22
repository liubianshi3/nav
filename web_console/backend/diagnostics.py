from __future__ import annotations

import re
import time
from collections import deque
from pathlib import Path

from .models import (
    DashboardSnapshot,
    DiagnosticItem,
    DiagnosticSummary,
    DiagnosticsSnapshot,
    LogEntry,
    StackStatus,
)

_LOG_CATEGORY_PATTERNS: dict[str, list[str]] = {
    "navigation": [
        "planner_server", "controller_server", "bt_navigator", "recoveries",
        "waypoint", "nav2", "plan failed", "no valid path", "goal",
        "navigate_to_pose", "global planner", "local planner",
    ],
    "localization": [
        "ndt", "relocalization", "map->odom", "tf ", "transform",
        "localization", "pose stale", "pose_", "initialpose",
        "ndt_adapter", "dlio/odom",
    ],
    "mapping": [
        "dlio", "pointcloud", "map_manager", "save map", "pcd",
        "traversability", "occupancy", "grid", "octomap",
        "slam_toolbox", "ground_seg",
    ],
    "control": [
        "cmd_vel", "cmd_vel_safe", "control_bridge", "sdk",
        "unitree_agent", "ipc", "uds",
        "cmd_timeout", "collision_monitor", "safety",
        "a2_control_bridge", "motion", "gait",
    ],
    "web": [
        "websocket", "frontend", "api", "snapshot", "health",
        "web_console", "fastapi", "uvicorn",
    ],
}


def _classify_log(source_name: str, message: str) -> str:
    text = f"{source_name} {message}".lower()
    for cat, keywords in _LOG_CATEGORY_PATTERNS.items():
        for kw in keywords:
            if kw in text:
                return cat
    return "system"


_LOG_LEVEL_RE = re.compile(r"\[(ERROR|WARN|WARNING|INFO|DEBUG)\]", re.IGNORECASE)
_BRACKET_RE = re.compile(r"\[([^\]]+)\]")


def _parse_log_line(line: str) -> LogEntry | None:
    line = line.strip()
    if not line:
        return None

    level_match = _LOG_LEVEL_RE.search(line)
    level = "INFO"
    if level_match:
        level = level_match.group(1).upper()
        if level == "WARNING":
            level = "WARN"

    source = ""
    message = line
    bracketed = _BRACKET_RE.findall(line[:240])
    if bracketed:
        if bracketed[0].upper() in {"ERROR", "WARN", "WARNING", "INFO", "DEBUG"}:
            source = bracketed[2] if len(bracketed) >= 3 else bracketed[-1]
        else:
            source = bracketed[0]
        message = _BRACKET_RE.sub("", line, count=len(bracketed)).lstrip(" :")

    category = _classify_log(source, message)

    return LogEntry(
        level=level,
        source=source,
        category=category,
        message=message,
        time="",
    )


def read_logs(
    log_file: str | None,
    source: str = "all",
    tail: int = 200,
    level: str = "all",
    search: str = "",
) -> list[LogEntry]:
    if not log_file:
        return []
    path = Path(log_file)
    if not path.exists():
        return []

    tail = min(max(tail, 10), 500)
    try:
        with path.open(encoding="utf-8", errors="replace") as f:
            lines = list(deque((ln for ln in f if ln.strip()), maxlen=tail))
    except OSError:
        return []

    entries: list[LogEntry] = []
    for line in lines:
        entry = _parse_log_line(line)
        if entry is None:
            continue
        if source != "all" and entry.category != source:
            continue
        if level != "all" and entry.level.lower() != level.lower():
            continue
        if search and search.lower() not in entry.message.lower():
            continue
        entries.append(entry)

    return entries


# ── Navigation checklist ──────────────────────────────────────────────


def _nav_goal_check(snapshot: DashboardSnapshot) -> DiagnosticItem:
    nav = snapshot.navigation
    goal = nav.goal
    item = DiagnosticItem(key="goal", label="Web 目标")

    if nav.state == "idle" and goal is None:
        item.state = "ok"
        item.reason = "未设置导航目标"
        return item
    if goal is None:
        item.state = "error"
        item.reason = "目标未发送"
        item.suggestion = "在地图上点击或在3D视图中双击选择目标点"
        return item

    missing = []
    if goal.x is None:
        missing.append("x")
    if goal.y is None:
        missing.append("y")
    if missing:
        item.state = "error"
        item.reason = f"目标坐标缺失: {', '.join(missing)}"
        item.suggestion = "重新在地图上选择目标点"
        return item

    if goal.frame_id and goal.frame_id != "map":
        item.state = "warn"
        item.reason = f"目标 frame_id={goal.frame_id}，应为 map"
        item.suggestion = "确认目标在 map 坐标系下发送"

    item.evidence.append(f"x={goal.x:.2f}, y={goal.y:.2f}, yaw={goal.yaw:.2f}")

    snap_map = snapshot.map
    if snap_map and snap_map.loaded and snap_map.width > 0 and snap_map.height > 0:
        if snap_map.resolution <= 0:
            item.state = "unknown"
            item.reason = "地图 resolution 异常"
            item.evidence.append(f"resolution={snap_map.resolution}")
            item.suggestion = "检查 map.yaml 中 resolution 配置"
            return item
        mx = (goal.x - snap_map.origin.x) / snap_map.resolution
        my = (goal.y - snap_map.origin.y) / snap_map.resolution
        if mx < 0 or mx >= snap_map.width or my < 0 or my >= snap_map.height:
            item.state = "error"
            item.reason = "目标点不在地图范围内"
            item.evidence.append(
                f"目标({goal.x:.2f},{goal.y:.2f}) 超出地图边界 "
                f"[{snap_map.width}x{snap_map.height}]"
            )
            item.suggestion = "在地图范围内重新选择目标点"
            return item

        pix = int(my) * snap_map.width + int(mx)
        if 0 <= pix < len(snap_map.data):
            val = snap_map.data[pix]
            if val >= 50:
                item.state = "error"
                item.reason = "目标点落在 occupied 区域（障碍物）"
                item.suggestion = "选择 free 区域的目标点"
            elif val < 0:
                item.state = "warn"
                item.reason = "目标点落在 unknown 区域"
                item.suggestion = "选择已知 free 区域的目标点，或先探索该区域"

    if item.state == "unknown":
        item.state = "ok"
        item.reason = "目标坐标有效"
    return item


def _nav_localization_check(snapshot: DashboardSnapshot) -> DiagnosticItem:
    item = DiagnosticItem(key="localization", label="定位 / TF")
    status = snapshot.status

    ndt_healthy = status.ndt_healthy
    ndt_score = status.ndt_score
    loc_ok = status.localization_ok
    pose = snapshot.pose

    item.evidence.append(f"localization_ok={loc_ok}")
    if ndt_healthy is not None:
        item.evidence.append(f"ndt_healthy={ndt_healthy}")
    if ndt_score is not None:
        item.evidence.append(f"ndt_score={ndt_score:.3f}")

    if pose and pose.available and not pose.stale:
        item.evidence.append(f"pose_age=ok x={pose.x:.2f} y={pose.y:.2f}")
    elif pose and pose.stale:
        item.evidence.append(f"pose_stale=True")
    else:
        item.evidence.append("pose_unavailable")

    if loc_ok is False or ndt_healthy is False:
        item.state = "error"
        item.reason = "定位未就绪"
        if ndt_healthy is False:
            item.reason += "：NDT 未收敛或 score 过高"
        item.suggestion = "重新设置初始位姿，检查 /a2/relocalization/status 和 map→odom TF"
        return item

    if pose and pose.stale:
        item.state = "error"
        item.reason = "机器人位姿已过期"
        item.suggestion = "检查定位节点是否在发布 pose，确认 TF 链路畅通"
        return item

    if pose and pose.available:
        item.state = "ok"
        item.reason = f"定位正常，当前位置 ({pose.x:.2f}, {pose.y:.2f})"
    else:
        item.state = "warn"
        item.reason = "定位状态未知，pose 未收到"
        item.suggestion = "确认定位模式是否为 NDT，等待首次定位收敛"
    return item


def _nav_map_check(snapshot: DashboardSnapshot) -> DiagnosticItem:
    item = DiagnosticItem(key="map", label="2D 地图")
    snap_map = snapshot.map

    if not snap_map or not snap_map.loaded:
        item.state = "error"
        item.reason = "2D 地图未加载"
        item.suggestion = "检查 map_server 是否已加载 /map，或先保存建图结果"
        return item

    checks = []
    if snap_map.resolution <= 0:
        checks.append("resolution 无效")
    if snap_map.width <= 0 or snap_map.height <= 0:
        checks.append("尺寸无效")
    if checks:
        item.state = "error"
        item.reason = f"地图参数异常：{', '.join(checks)}"
        return item

    item.evidence = [
        f"{snap_map.width}x{snap_map.height} res={snap_map.resolution:.3f}",
        f"origin=({snap_map.origin.x:.2f}, {snap_map.origin.y:.2f})",
    ]

    pose = snapshot.pose
    if pose and pose.available and pose.x is not None and pose.y is not None:
        mx = (pose.x - snap_map.origin.x) / snap_map.resolution
        my = (pose.y - snap_map.origin.y) / snap_map.resolution
        if mx < 0 or mx >= snap_map.width or my < 0 or my >= snap_map.height:
            item.state = "warn"
            item.reason = "机器人当前位置不在地图范围内"
            item.evidence.append(f"pose ({pose.x:.2f},{pose.y:.2f}) 超出地图边界")
            item.suggestion = "确认机器人初始位姿是否正确设置为地图内位置"
            return item

    item.state = "ok"
    item.reason = "地图已加载且参数有效"
    return item


def _nav_planner_check(snapshot: DashboardSnapshot) -> DiagnosticItem:
    item = DiagnosticItem(key="planner", label="全局规划")
    nav = snapshot.navigation
    feedback = nav.feedback or {}

    if nav.state == "idle":
        item.state = "skipped"
        item.reason = "无导航任务"
        return item

    plan_error = str(feedback.get("error", "") or "")
    if "plan failed" in plan_error.lower() or "no valid path" in plan_error.lower():
        item.state = "error"
        item.reason = f"全局规划失败：{plan_error}"
        item.suggestion = "检查目标点是否可达，地图是否完整覆盖路径区域"
        return item

    if "goal occupied" in plan_error.lower():
        item.state = "error"
        item.reason = "目标点被障碍物占据"
        item.evidence.append("goal occupied")
        item.suggestion = "重新选择 free 区域的目标点"
        return item

    if nav.state == "navigating":
        item.state = "ok"
        item.reason = "全局路径已生成"
    elif nav.state == "succeeded":
        item.state = "ok"
        item.reason = "导航完成"
    elif nav.state == "failed":
        item.state = "error"
        item.reason = f"导航失败: {nav.message or '未知原因'}"
        item.suggestion = "查看控制器和定位状态，确认地图完整"
    elif nav.state == "canceled":
        item.state = "warn"
        item.reason = "导航已取消"
    else:
        item.state = "unknown"

    return item


def _nav_controller_check(snapshot: DashboardSnapshot) -> DiagnosticItem:
    item = DiagnosticItem(key="controller", label="局部控制")
    nav = snapshot.navigation

    if nav.state in ("idle", "canceled"):
        item.state = "skipped"
        item.reason = "无导航任务"
        return item

    feedback = nav.feedback or {}
    ctrl_error = str(feedback.get("error", "") or "").lower()
    if any(kw in ctrl_error for kw in ("oscillation", "stuck", "no valid control")):
        item.state = "error"
        item.reason = f"局部控制器异常：{ctrl_error}"
        item.suggestion = "检查机器人附近是否有障碍物卡住，尝试重新规划"
        return item

    status = snapshot.status
    vel_x = status.velocity_linear_x
    vel_z = status.velocity_angular_z

    if vel_x is not None and vel_z is not None:
        item.evidence.append(f"cmd_vel=({vel_x:.3f}, {vel_z:.3f})")

    if nav.state == "navigating":
        item.state = "ok"
        item.reason = "局部控制器正在跟踪路径"
        if vel_x is not None and abs(vel_x) < 0.01 and abs(vel_z) < 0.01:
            item.state = "warn"
            item.reason = "控制器有输出但速度为零（可能在原地调整方向或已接近目标）"
    elif nav.state == "failed":
        item.state = "error"
        item.reason = "导航失败，控制器已停止"
    else:
        item.state = "unknown"

    return item


def _nav_safety_check(snapshot: DashboardSnapshot) -> DiagnosticItem:
    item = DiagnosticItem(key="safety", label="安全层 / 碰撞监控")
    safety = snapshot.status.safety_status
    control = snapshot.control_state

    has_safety_data = safety and safety.ready is not None
    has_control_data = control and control.state

    if safety and safety.reason:
        item.evidence.append(f"safety={safety.reason}")
    if control and control.reason:
        item.evidence.append(f"control_state={control.state} reason={control.reason}")

    if not has_safety_data and not has_control_data:
        item.state = "unknown"
        item.reason = "未订阅到 safety/control 状态数据"
        item.suggestion = "确认 safety_manager 和 control_bridge 节点是否运行"
        return item

    if safety and safety.state == "stop":
        item.state = "warn"
        item.reason = "safety 状态为 stop，运动可能被拦截"
        item.suggestion = "检查碰撞监控区域是否有障碍物"
        return item

    if control and control.last_error_reason:
        item.state = "warn"
        item.reason = f"控制错误：{control.last_error_reason}"
        item.suggestion = "确认机器人状态正常，必要时重新授权运动"
        return item

    if safety and safety.ready is not None and not safety.ready:
        item.state = "warn"
        item.reason = "safety 未就绪"
        return item

    if safety and safety.ready and (not control or not control.last_error_reason):
        item.state = "ok"
        item.reason = "安全层正常"
    else:
        item.state = "unknown"

    return item


def _nav_control_bridge_check(snapshot: DashboardSnapshot) -> DiagnosticItem:
    item = DiagnosticItem(key="control_bridge", label="控制桥 / Unitree agent")
    control_status = snapshot.status.control_status
    sdk_status = snapshot.status.sdk_status

    has_control_data = control_status and control_status.ready is not None
    has_sdk_data = sdk_status and sdk_status.ready is not None

    if control_status and control_status.reason:
        item.evidence.append(f"control={control_status.reason}")
    if sdk_status and sdk_status.reason:
        item.evidence.append(f"agent_state={sdk_status.reason}")

    if not has_control_data and not has_sdk_data:
        item.state = "unknown"
        item.reason = "未订阅到 control/agent 状态数据"
        item.suggestion = "确认 a2_control_bridge_ros 和 a2_sdk_bridge_ros 节点是否运行"
        return item

    if control_status and control_status.state in ("timeout", "cmd_timeout", "offline"):
        item.state = "error"
        item.reason = "控制桥超时或离线"
        item.suggestion = "检查 /cmd_vel_safe、UDS socket 和 unitree_agent 是否可用"
        return item

    if sdk_status and sdk_status.ready is not None and not sdk_status.ready:
        item.state = "error"
        item.reason = "unitree_agent 状态未就绪"
        item.suggestion = "检查 /run/a2/unitree_agent.sock、unitree_agent 容器和机器人链路"
        return item

    if (has_control_data and control_status.ready) and (has_sdk_data and sdk_status.ready):
        item.state = "ok"
        item.reason = "ROS 控制桥和 unitree_agent 状态正常"
    else:
        item.state = "unknown"
    return item


def _build_navigation_checklist(snapshot: DashboardSnapshot) -> list[DiagnosticItem]:
    return [
        _nav_goal_check(snapshot),
        _nav_localization_check(snapshot),
        _nav_map_check(snapshot),
        _nav_planner_check(snapshot),
        _nav_controller_check(snapshot),
        _nav_safety_check(snapshot),
        _nav_control_bridge_check(snapshot),
    ]


# ── Mapping checklist ─────────────────────────────────────────────────


def _map_lidar_check(snapshot: DashboardSnapshot) -> DiagnosticItem:
    item = DiagnosticItem(key="lidar_input", label="LiDAR 点云输入")
    lidar = snapshot.status.lidar_status
    pc = snapshot.pointcloud

    if lidar and lidar.reason:
        item.evidence.append(f"lidar={lidar.reason}")
    if pc and pc.loaded:
        item.evidence.append(f"points={pc.points_total}")

    if lidar and lidar.ready is False:
        item.state = "error"
        item.reason = "雷达点云未就绪"
        item.suggestion = "检查 JT128 驱动是否运行，topic 是否有数据"
        return item

    if lidar and lidar.ready is True:
        item.state = "ok"
        item.reason = f"雷达点云正常，{pc.points_total if pc else 0} 点"
    else:
        item.state = "unknown"
    return item


def _map_imu_odom_check(snapshot: DashboardSnapshot) -> DiagnosticItem:
    item = DiagnosticItem(key="imu_odom", label="IMU / 里程计")
    raw = snapshot.status.raw_state

    if raw is None:
        item.state = "unknown"
        item.reason = "raw_state 未收到"
        return item

    imu_ok = raw.imu_valid
    odom_ok = raw.odom_valid
    item.evidence.append(f"imu_valid={imu_ok} odom_valid={odom_ok}")

    if imu_ok is False or odom_ok is False:
        item.state = "error"
        missing = []
        if imu_ok is False:
            missing.append("IMU")
        if odom_ok is False:
            missing.append("Odom")
        item.reason = f"{', '.join(missing)} 无效"
        item.suggestion = "检查 a2_sdk_bridge_ros 状态、unitree_agent IPC 和传感器数据链路"
    elif imu_ok is True and odom_ok is True:
        item.state = "ok"
        item.reason = "IMU 和里程计正常"
    else:
        item.state = "unknown"
        item.reason = "IMU 或里程计状态未知"
    return item


def _map_dlio_check(snapshot: DashboardSnapshot, stack: StackStatus | None) -> DiagnosticItem:
    item = DiagnosticItem(key="dlio", label="DLIO 在线状态")
    item.evidence.append(f"stack_mode={stack.mode if stack else 'N/A'}")

    if stack and stack.nodes:
        for node in stack.nodes:
            if "dlio" in node.key.lower():
                item.evidence.append(f"dlio_node={node.state}")
                if node.running:
                    item.state = "ok"
                    item.reason = "DLIO 节点运行中"
                else:
                    item.state = "error"
                    item.reason = f"DLIO 节点状态：{node.state}"
                    item.suggestion = "启动建图栈或检查 DLIO 节点日志"
                return item

    if snapshot.status.raw_state and snapshot.status.raw_state.connected:
        item.state = "warn"
        item.reason = "机器人已连接但未检测到 DLIO 节点"
        item.suggestion = "是否在建图模式下？检查 stack status"
    else:
        item.state = "unknown"
        item.reason = "DLIO 状态未知"
    return item


def _map_pointcloud_growth_check(snapshot: DashboardSnapshot) -> DiagnosticItem:
    item = DiagnosticItem(key="pointcloud_input", label="点云接收")
    pc = snapshot.pointcloud

    if pc and pc.loaded and pc.points_total > 0:
        item.state = "ok"
        item.reason = f"收到 {pc.points_total} 个点"
        item.evidence.append(f"source={pc.source_topic}")
    elif pc and pc.loaded:
        item.state = "warn"
        item.reason = "3D 点云已加载但点数为 0"
        item.suggestion = "检查点云 source topic 和 DLIO 累计点云输出"
    else:
        item.state = "unknown"
        item.reason = "未收到 3D 点云数据"
    return item


def _map_web_3d_check(snapshot: DashboardSnapshot) -> DiagnosticItem:
    item = DiagnosticItem(key="web_3d_pc", label="Web 3D 点云通道")
    pc = snapshot.pointcloud

    if pc and pc.loaded:
        item.state = "ok"
        item.reason = f"Web 正在接收点云 ({pc.points_total} 点)"
    else:
        item.state = "warn"
        item.reason = "Web 未收到 3D 点云"
        item.suggestion = "检查 WebSocket 连接和 pointcloud 事件推送"
    return item


def _map_traversability_check(snapshot: DashboardSnapshot) -> DiagnosticItem:
    item = DiagnosticItem(key="traversability", label="2D 栅格生成")
    snap_map = snapshot.map

    if snap_map and snap_map.loaded:
        item.state = "ok"
        item.reason = f"2D 栅格已加载 ({snap_map.width}x{snap_map.height})"
    else:
        item.state = "warn"
        item.reason = "2D 栅格未生成"
        item.suggestion = "检查 traversability 投影和 occupancy mapper 是否运行"
    return item


def _map_save_check(snapshot: DashboardSnapshot) -> DiagnosticItem:
    item = DiagnosticItem(key="map_save", label="保存地图")
    mm_status = snapshot.status.map_manager_status

    if mm_status and mm_status.reason:
        item.evidence.append(f"map_manager={mm_status.reason}")

    if mm_status and mm_status.state in ("error", "failed"):
        item.state = "error"
        item.reason = f"地图保存失败：{mm_status.reason or '未知原因'}"
        item.suggestion = "检查点云是否为空，map_manager 服务是否可用"
    elif mm_status and mm_status.state == "saving" or (mm_status and mm_status.ready):
        item.state = "ok"
        item.reason = "map_manager 正常"
    else:
        item.state = "unknown"
    return item


def _build_mapping_checklist(snapshot: DashboardSnapshot, stack: StackStatus | None) -> list[DiagnosticItem]:
    return [
        _map_lidar_check(snapshot),
        _map_imu_odom_check(snapshot),
        _map_dlio_check(snapshot, stack),
        _map_pointcloud_growth_check(snapshot),
        _map_web_3d_check(snapshot),
        _map_traversability_check(snapshot),
        _map_save_check(snapshot),
    ]


# ── Summary ───────────────────────────────────────────────────────────

_SEVERITY_ORDER: dict[str, int] = {"error": 0, "warn": 1, "ok": 2, "skipped": 3, "unknown": 4}


def _summary_from_checklist(
    checklist: list[DiagnosticItem],
    mode_label: str,
) -> DiagnosticSummary:
    worst: DiagnosticItem | None = None
    for item in checklist:
        if item.state in ("error", "warn"):
            if worst is None or _SEVERITY_ORDER.get(item.state, 99) < _SEVERITY_ORDER.get(worst.state, 99):
                worst = item
    if worst:
        return DiagnosticSummary(
            severity=worst.state,
            title=f"{mode_label}异常：{worst.label}",
            reason=worst.reason,
            evidence=worst.evidence,
            suggestion=worst.suggestion,
        )
    ok_count = sum(1 for i in checklist if i.state == "ok")
    skipped_count = sum(1 for i in checklist if i.state == "skipped")
    if ok_count + skipped_count >= len(checklist):
        return DiagnosticSummary(
            severity="ok",
            title=f"{mode_label}链路正常",
            reason=f"{ok_count}/{len(checklist)} 项检查通过",
        )
    return DiagnosticSummary(
        severity="unknown",
        title=f"{mode_label}部分状态未知",
        reason=f"{ok_count}/{len(checklist)} 项检查通过",
    )


def _generate_summary(
    mode: str,
    nav_checklist: list[DiagnosticItem],
    mapping_checklist: list[DiagnosticItem],
) -> DiagnosticSummary:
    if mode == "mapping":
        return _summary_from_checklist(mapping_checklist, "建图")
    if mode in ("navigation", "nav"):
        return _summary_from_checklist(nav_checklist, "导航")
    return DiagnosticSummary(
        severity="ok",
        title=f"系统状态：{mode or 'stopped'}",
        reason=f"当前模式：{mode or 'stopped'}",
    )


# ── Public API ────────────────────────────────────────────────────────


def build_diagnostics(
    snapshot: DashboardSnapshot,
    stack_status: StackStatus | None = None,
    *,
    data_start_monotonic: float | None = None,
    log_file: str | None = None,
) -> DiagnosticsSnapshot:
    mode = "stopped"
    if stack_status:
        mode = stack_status.mode

    nav_checklist = _build_navigation_checklist(snapshot)
    mapping_checklist = _build_mapping_checklist(snapshot, stack_status)
    summary = _generate_summary(mode, nav_checklist, mapping_checklist)

    generated_at = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())
    data_age = 0.0
    if data_start_monotonic is not None:
        data_age = time.monotonic() - data_start_monotonic

    snapshot_available = snapshot is not None
    logs_available = bool(log_file and Path(log_file).exists())

    return DiagnosticsSnapshot(
        summary=summary,
        navigation=nav_checklist,
        mapping=mapping_checklist,
        mode=mode,
        diagnostics_generated_at=generated_at,
        data_age_sec=round(data_age, 3),
        snapshot_available=snapshot_available,
        logs_available=logs_available,
    )
