from __future__ import annotations

import ipaddress
import math
from datetime import datetime, timezone
from typing import Any


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def dump_model(model: Any) -> Any:
    if hasattr(model, "model_dump"):
        return model.model_dump()
    if hasattr(model, "dict"):
        return model.dict()
    return model


def deep_copy_model(model: Any) -> Any:
    if hasattr(model, "model_copy"):
        return model.model_copy(deep=True)
    if hasattr(model, "copy"):
        return model.copy(deep=True)
    return model


def parse_status_string(raw: str | None) -> tuple[str | None, dict[str, str]]:
    if raw is None:
        return None, {}
    text = raw.strip()
    if not text:
        return None, {}
    fields: dict[str, str] = {}
    for token in text.split(";"):
        if not token or "=" not in token:
            continue
        key, value = token.split("=", 1)
        fields[key.strip()] = value.strip()
    return text, fields


def parse_optional_bool(value: Any) -> bool | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"true", "1", "yes", "ready"}:
        return True
    if text in {"false", "0", "no", "not_ready"}:
        return False
    return None


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def is_lan_or_loopback(host: str | None) -> bool:
    if not host:
        return False
    try:
        ip = ipaddress.ip_address(host)
    except ValueError:
        return host in {"localhost"}
    return bool(ip.is_private or ip.is_loopback or ip.is_link_local)
