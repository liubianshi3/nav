from __future__ import annotations

from pathlib import Path

import yaml


def _parse_nav2_map_yaml(map_yaml_path: Path) -> tuple[str, float, tuple[float, float, float]]:
    payload = yaml.safe_load(map_yaml_path.read_text(encoding="utf-8")) or {}
    if not isinstance(payload, dict):
        raise ValueError("map.yaml is not a mapping")
    image_rel = str(payload.get("image") or "").strip().strip("'\"")
    if not image_rel:
        raise ValueError("map.yaml missing image field")
    resolution = float(payload.get("resolution") or 0.0)
    origin_raw = payload.get("origin") or [0.0, 0.0, 0.0]
    if not isinstance(origin_raw, (list, tuple)) or len(origin_raw) < 2:
        origin_raw = [0.0, 0.0, 0.0]
    x = float(origin_raw[0] or 0.0)
    y = float(origin_raw[1] or 0.0)
    yaw = float(origin_raw[2] or 0.0) if len(origin_raw) >= 3 else 0.0
    return image_rel, resolution, (x, y, yaw)


def _read_pgm_luma(path: Path) -> tuple[int, int, bytes]:
    raw = path.read_bytes()
    if not raw.startswith(b"P5"):
        raise ValueError("unsupported PGM format")
    parts: list[bytes] = []
    idx = 0
    length = len(raw)

    def _skip_ws(i: int) -> int:
        while i < length and raw[i:i + 1] in b" \t\r\n":
            i += 1
        return i

    def _read_token(i: int) -> tuple[bytes, int]:
        i = _skip_ws(i)
        if i < length and raw[i:i + 1] == b"#":
            while i < length and raw[i:i + 1] not in b"\n":
                i += 1
            return _read_token(i)
        start = i
        while i < length and raw[i:i + 1] not in b" \t\r\n":
            i += 1
        if i <= start:
            raise ValueError("invalid PGM header")
        return raw[start:i], i

    token, idx = _read_token(idx)
    parts.append(token)
    for _ in range(3):
        token, idx = _read_token(idx)
        parts.append(token)
    magic, width_b, height_b, maxval_b = parts
    if magic != b"P5":
        raise ValueError("unsupported PGM format")
    width = int(width_b)
    height = int(height_b)
    maxval = int(maxval_b)
    if maxval != 255:
        raise ValueError("unsupported PGM maxval")
    idx = _skip_ws(idx)
    expected = width * height
    data = raw[idx:idx + expected]
    if len(data) != expected:
        raise ValueError("truncated PGM data")
    return width, height, data


def _occupancy_bytes_from_nav2_luma(width: int, height: int, luma: bytes) -> bytes:
    if width <= 0 or height <= 0:
        return b""
    if len(luma) != width * height:
        raise ValueError("unexpected luma length")
    out = bytearray(width * height)
    for y_top in range(height):
        y_ros = height - 1 - y_top
        base_in = y_top * width
        base_out = y_ros * width
        for x in range(width):
            pixel = int(luma[base_in + x])
            if pixel == 0:
                value = 100
            elif pixel >= 250:
                value = 0
            else:
                value = -1
            out[base_out + x] = value & 0xFF
    return bytes(out)

