from __future__ import annotations

import math
from typing import Any

from .ros_bridge import RosBridgeError


def _safe_int(value: Any, fallback: int = 0) -> int:
    try:
        return int(value or 0)
    except Exception:
        return fallback


def _safe_float(value: Any, fallback: float = 0.0) -> float:
    try:
        number = float(value)
    except Exception:
        return fallback
    return number if math.isfinite(number) else fallback


def _manual_motion_state(node: Any) -> dict[str, Any]:
    snapshot = None
    if hasattr(node, "build_snapshot"):
        snapshot = node.build_snapshot(ros_thread_alive=True)

    raw_state = getattr(getattr(snapshot, "status", None), "raw_state", None)
    control_state = getattr(snapshot, "control_state", None) or getattr(node, "control_state", None)

    motion_mode = _safe_int(getattr(raw_state, "motion_mode", 0))
    gait_type = _safe_int(getattr(raw_state, "gait_type", 0))
    sdk_code = _safe_int(getattr(control_state, "last_sdk_code", 0))
    last_error_code = str(getattr(control_state, "last_error_code", "") or "")
    last_command = str(getattr(control_state, "last_command", "") or "").lower()
    body_height = _safe_float(getattr(raw_state, "body_height", None), fallback=0.0)

    standing_motion_modes = {0, 1, 2, 3, 8}
    nonstanding_motion_modes = {5, 7, 10}
    command_ok = sdk_code == 0 and last_error_code in {"", "ok"}
    standing: bool | None
    if raw_state is not None and getattr(raw_state, "connected", True) is False:
        standing = False
    elif motion_mode in nonstanding_motion_modes:
        standing = False
    elif motion_mode in standing_motion_modes:
        standing = True
    elif command_ok and last_command in {"stand_down", "damp"}:
        standing = False
    elif command_ok and last_command in {
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
    elif body_height > 0.2:
        standing = True
    else:
        standing = None

    authorized = motion_mode == 3 or (command_ok and last_command in {"balance_stand", "move", "walk"})
    return {
        "standing": standing,
        "authorized": bool(authorized),
        "motion_mode": motion_mode,
        "gait_type": gait_type,
        "last_command": last_command,
    }


def get_manual_motion_authorization(node: Any) -> dict[str, Any]:
    state = _manual_motion_state(node)
    if state["authorized"]:
        return {
            "success": True,
            "message": "motion authorization available",
            "error_code": "ok",
            "state": "AUTHORIZED",
            "required_action": "NONE",
            "standing": True,
            "motion_authorized": True,
            "manual_start_required": False,
            "motion_mode": state["motion_mode"],
            "gait_type": state["gait_type"],
        }

    if state["standing"] is False:
        return {
            "success": False,
            "message": "stand up before requesting motion authorization",
            "error_code": "stand_up_required",
            "state": "STAND_DOWN",
            "required_action": "STAND_UP",
            "standing": False,
            "motion_authorized": False,
            "manual_start_required": False,
            "motion_mode": state["motion_mode"],
            "gait_type": state["gait_type"],
        }

    if state["standing"] is None:
        return {
            "success": False,
            "message": "motion authorization state unavailable",
            "error_code": "state_unavailable",
            "state": "UNKNOWN",
            "required_action": "NONE",
            "standing": False,
            "motion_authorized": False,
            "manual_start_required": False,
            "motion_mode": state["motion_mode"],
            "gait_type": state["gait_type"],
        }

    return {
        "success": False,
        "message": "call AuthorizeMotion after the robot is standing",
        "error_code": "motion_authorization_required",
        "state": "STANDING_NOT_AUTHORIZED",
        "required_action": "NONE",
        "standing": True,
        "motion_authorized": False,
        "manual_start_required": False,
        "motion_mode": state["motion_mode"],
        "gait_type": state["gait_type"],
    }


def _require_success(result: Any, command: str) -> None:
    if result is None:
        raise RosBridgeError(f"a2 motion command {command} 未返回结果")
    if not bool(getattr(result, "success", False)):
        message = str(getattr(result, "message", "") or f"a2 motion command {command} failed")
        raise RosBridgeError(message)


def ensure_manual_motion_authorized(node: Any) -> dict[str, Any]:
    state = get_manual_motion_authorization(node)
    if state["motion_authorized"]:
        return {"ok": True, "started": False, "authorized": False, "message": "motion already authorized"}

    if not state["standing"]:
        raise RosBridgeError(str(state["message"]))

    authorize = node.call_motion_command("balance_stand")
    _require_success(authorize, "balance_stand")
    return {"ok": True, "started": False, "authorized": True, "message": "motion authorized"}
