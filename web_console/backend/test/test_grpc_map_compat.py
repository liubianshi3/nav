from __future__ import annotations

import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from backend.map_formats import (
    _occupancy_bytes_from_nav2_luma,
    _parse_nav2_map_yaml,
    _read_pgm_luma,
)


def test_parse_nav2_map_yaml_reads_image_resolution_origin(tmp_path: Path) -> None:
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text(
        yaml.safe_dump(
            {"image": "map.pgm", "resolution": 0.05, "origin": [1.0, 2.0, 0.25]},
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    image_rel, resolution, origin = _parse_nav2_map_yaml(map_yaml)
    assert image_rel == "map.pgm"
    assert resolution == 0.05
    assert origin == (1.0, 2.0, 0.25)


def test_occupancy_bytes_from_pgm_matches_map_manager_thresholds_and_flip(tmp_path: Path) -> None:
    pgm = tmp_path / "map.pgm"
    width, height = 2, 2
    luma_top_down = bytes(
        [
            0,
            254,
            205,
            254,
        ]
    )
    pgm.write_bytes(b"P5\n2 2\n255\n" + luma_top_down)

    got_w, got_h, luma = _read_pgm_luma(pgm)
    assert (got_w, got_h) == (width, height)
    assert luma == luma_top_down

    occupancy = _occupancy_bytes_from_nav2_luma(width, height, luma)
    assert occupancy == bytes([255, 0, 100, 0])
