"""
Parse status strings used throughout a2_system_ws into structured dicts.

The convention used by nearly every node is::

    mode=<runtime_mode>;state=<state>;ready=<bool>;reason=<text>;k=v;...

This module converts those strings into dicts and maps them to standard
diagnostic_msgs/DiagnosticStatus values.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


def parse_status_string(status: str) -> Dict[str, str]:
    """Parse ``key=value;key=value;...`` into a dict.  Missing semicolons and
    empty segments are tolerated."""
    result: Dict[str, str] = {}
    for segment in status.split(";"):
        segment = segment.strip()
        if not segment or "=" not in segment:
            continue
        key, _, value = segment.partition("=")
        result[key.strip()] = value.strip()
    return result


def status_level(parsed: Dict[str, str]) -> Tuple[int, str]:
    """Map a parsed status dict to a (DiagnosticStatus level, short label).

    Mapping rules (priority order):
      - ``ready=true``  → OK
      - ``ready=false`` and state contains ``stale/timeout/error/fault/
        blocked/rejected/aborted/canceled`` → ERROR
      - ``ready=false`` and state contains ``waiting/preparing/idle`` → WARN
      - ``ready=false`` otherwise → WARN
      - no ``ready`` key → STALE
    """
    ready_str = parsed.get("ready", "").lower()
    if ready_str == "true":
        return DiagnosticStatus.OK, "OK"

    state = parsed.get("state", "").lower()
    reason = parsed.get("reason", "").lower()

    if ready_str == "false":
        error_states = {
            "stale", "timeout", "error", "fault", "blocked", "rejected",
            "aborted", "canceled", "missing", "disabled",
        }
        if any(s in state for s in error_states) or any(
            s in reason for s in error_states
        ):
            return DiagnosticStatus.ERROR, "ERROR"
        return DiagnosticStatus.WARN, "WARN"

    # No ready key - treat as stale / unknown
    return DiagnosticStatus.STALE, "STALE"


def build_diagnostic_status(
    name: str,
    hardware_id: str,
    parsed: Dict[str, str],
) -> DiagnosticStatus:
    """Build a single DiagnosticStatus from parsed key-value pairs.

    Args:
        name: ROS node / component name, e.g. ``safety_supervisor``.
        hardware_id: Hardware identifier, e.g. ``a2_robot``.
        parsed: Dict from :func:`parse_status_string`.
    """
    ds = DiagnosticStatus()
    ds.name = name
    ds.hardware_id = hardware_id

    level, label = status_level(parsed)
    ds.level = level

    # Build human-readable message
    state = parsed.get("state", "unknown")
    reason = parsed.get("reason", "")
    ds.message = f"[{label}] state={state}" + (f" reason={reason}" if reason else "")

    # Convert all key-value pairs to diagnostic values
    ds.values = [KeyValue(key=str(k), value=str(v)) for k, v in parsed.items()]

    return ds


def aggregate_level(statuses: List[DiagnosticStatus]) -> int:
    """Worst-level-wins aggregation.  STALE (3) > ERROR (2) > WARN (1) > OK (0)."""
    if not statuses:
        return DiagnosticStatus.STALE
    return max(s.level for s in statuses)


def build_aggregated_status(
    statuses: List[DiagnosticStatus],
    name: str = "a2_system",
    hardware_id: str = "a2_robot",
) -> DiagnosticStatus:
    """Produce a single global DiagnosticStatus summarizing many sub-statuses.

    The level is the worst among all inputs.
    The message lists each sub-status with its level.
    """
    ds = DiagnosticStatus()
    ds.name = name
    ds.hardware_id = hardware_id

    if not statuses:
        ds.level = DiagnosticStatus.STALE
        ds.message = "No diagnostic sources connected"
        return ds

    ds.level = aggregate_level(statuses)

    level_names = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}
    lines = [
        f"{s.name}: [{level_names.get(s.level[0] if isinstance(s.level, bytes) else s.level, '?')}] {s.message}"
        for s in statuses
    ]
    ds.message = "; ".join(lines)
    ds.values = [
        KeyValue(key=s.name, value=str(s.level)) for s in sorted(statuses, key=lambda s: s.name)
    ]

    return ds
