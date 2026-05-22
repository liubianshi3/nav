from __future__ import annotations

from typing import Any


RETARGETABLE_BACKENDS = {"nav2", "pose_topic_3d", "cmd_vel_direct"}


def active_navigation_goal_conflict_reason(
    *,
    backend: str,
    has_active_action_goal: bool,
    has_active_pose_goal: bool,
) -> str | None:
    if not has_active_action_goal and not has_active_pose_goal:
        return None
    if backend in RETARGETABLE_BACKENDS:
        return None
    return "已有导航任务正在执行"


def _field(status: Any, name: str) -> str:
    fields = getattr(status, "fields", {}) or {}
    value = fields.get(name, "")
    return str(value)


def ndt_waiting_for_initialpose(status: Any) -> bool:
    state = str(getattr(status, "state", "") or "").lower()
    reason = str(getattr(status, "reason", "") or "").lower()
    initial_guess_count = _field(status, "initial_guess_count")
    return (
        state in {"waiting_seed", "waiting_initial_guess"}
        or reason in {"send_initialpose", "initial_guess_missing"}
        or initial_guess_count == "0"
    )


def localization_goal_block_reason(*, localization_ok: bool | None, relocalization_status: Any) -> str | None:
    if localization_ok is True:
        return None
    if ndt_waiting_for_initialpose(relocalization_status):
        return "NDT 等待初始位姿，请先设置初始位姿"
    return "定位未就绪，禁止发送导航目标"
