#!/usr/bin/env python3
import sys
import os
import math
import collections
from datetime import datetime
from pathlib import Path
import numpy as np

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False


def parse_yaml_manual(content: str) -> dict:
    """Manually parse a simple map.yaml file in case PyYAML is missing."""
    data = {}
    for line in content.splitlines():
        if "#" in line:
            line, _, _ = line.partition("#")
        line = line.strip()
        if not line or ":" not in line:
            continue
        key, _, val = line.partition(":")
        key = key.strip()
        val = val.strip()
        if not val:
            continue
        
        # Parse list or primitive values
        if val.startswith("[") and val.endswith("]"):
            parts = val[1:-1].split(",")
            parsed_list = []
            for p in parts:
                p = p.strip()
                try:
                    parsed_list.append(float(p))
                except ValueError:
                    parsed_list.append(p)
            data[key] = parsed_list
        else:
            try:
                if "." in val or "e" in val.lower():
                    data[key] = float(val)
                else:
                    data[key] = int(val)
            except ValueError:
                if (val.startswith('"') and val.endswith('"')) or (val.startswith("'") and val.endswith("'")):
                    val = val[1:-1]
                data[key] = val
    return data


def load_yaml(path: Path) -> dict:
    content = path.read_text(encoding="utf-8")
    if HAS_YAML:
        try:
            return yaml.safe_load(content) or {}
        except Exception:
            pass
    return parse_yaml_manual(content)


def read_pgm_p5(path: Path) -> tuple[np.ndarray, int, int]:
    """Read binary P5 PGM file. Returns (grid, width, height)."""
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"Not a P5 PGM file (magic: {magic})")
            
        def next_token():
            while True:
                line = f.readline()
                if not line:
                    raise ValueError("Unexpected EOF in PGM header")
                line = line.strip()
                if line.startswith(b"#"):
                    continue
                if line:
                    return line
                    
        dims = next_token().split()
        while len(dims) < 2:
            dims.extend(next_token().split())
        width = int(dims[0])
        height = int(dims[1])
        
        max_val_line = next_token().split()
        max_val = int(max_val_line[0])
        
        data = f.read()
        if len(data) < width * height:
            raise ValueError(f"PGM data size {len(data)} is less than expected {width * height}")
            
        grid = np.frombuffer(data[:width * height], dtype=np.uint8).reshape((height, width))
        return grid, width, height


def read_pcd_ascii(path: Path) -> tuple[np.ndarray, list[str]] | None:
    """Read points from ASCII PCD file. Returns (points, fields) or None if not ASCII."""
    if not path.exists():
        return None
    fields = []
    header_lines = 0
    is_ascii = False
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            header_lines += 1
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if not parts:
                continue
            key = parts[0].upper()
            if key == "FIELDS":
                fields = parts[1:]
            elif key == "DATA":
                if len(parts) > 1 and parts[1].lower() == "ascii":
                    is_ascii = True
                break
    
    if not is_ascii:
        return None
        
    usecols = (0, 1, 2)
    if fields and "x" in fields and "y" in fields and "z" in fields:
        usecols = (fields.index("x"), fields.index("y"), fields.index("z"))
    
    try:
        pts = np.loadtxt(path, skiprows=header_lines, usecols=usecols)
        if pts.ndim == 1:
            pts = pts.reshape((-1, 3))
        return pts, fields
    except Exception:
        # Fallback to line by line manual parsing
        pts = []
        with path.open("r", encoding="ascii", errors="replace") as f:
            for _ in range(header_lines):
                f.readline()
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) < 3:
                    continue
                try:
                    if fields and "x" in fields and "y" in fields and "z" in fields:
                        xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z")
                        pts.append((float(parts[xi]), float(parts[yi]), float(parts[zi])))
                    else:
                        pts.append((float(parts[0]), float(parts[1]), float(parts[2])))
                except (ValueError, IndexError):
                    continue
        if not pts:
            return None
        return np.array(pts, dtype=np.float64), fields


def find_largest_free_component(free_mask: np.ndarray) -> int:
    """Find size of the largest connected component of free cells using fast BFS."""
    h, w = free_mask.shape
    visited = np.zeros((h, w), dtype=bool)
    total_free = np.sum(free_mask)
    if total_free == 0:
        return 0
    
    flat_free_mask = free_mask.ravel()
    flat_visited = visited.ravel()
    
    largest_size = 0
    free_indices = np.flatnonzero(flat_free_mask)
    
    for idx in free_indices:
        if flat_visited[idx]:
            continue
            
        # BFS using flat index for efficiency
        q = collections.deque([idx])
        flat_visited[idx] = True
        comp_size = 0
        
        while q:
            curr = q.popleft()
            comp_size += 1
            
            cy, cx = curr // w, curr % w
            for ny, nx in [(cy-1, cx), (cy+1, cx), (cy, cx-1), (cy, cx+1)]:
                if 0 <= ny < h and 0 <= nx < w:
                    n_idx = ny * w + nx
                    if flat_free_mask[n_idx] and not flat_visited[n_idx]:
                        flat_visited[n_idx] = True
                        q.append(n_idx)
                        
        if comp_size > largest_size:
            largest_size = comp_size
            if largest_size > total_free // 2:
                break
                
    return largest_size


def get_max_runs(mask: np.ndarray) -> np.ndarray:
    """Compute maximum contiguous run of True values for each row and column."""
    h, w = mask.shape
    max_runs = []
    
    # Process rows
    for i in range(h):
        row = mask[i]
        if not np.any(row):
            continue
        padded = np.zeros(w + 2, dtype=int)
        padded[1:-1] = row.astype(int)
        diff = np.diff(padded)
        starts = np.flatnonzero(diff == 1)
        ends = np.flatnonzero(diff == -1)
        if len(starts) > 0 and len(ends) > 0:
            max_runs.append(int(np.max(ends - starts)))
            
    # Process columns
    for j in range(w):
        col = mask[:, j]
        if not np.any(col):
            continue
        padded = np.zeros(h + 2, dtype=int)
        padded[1:-1] = col.astype(int)
        diff = np.diff(padded)
        starts = np.flatnonzero(diff == 1)
        ends = np.flatnonzero(diff == -1)
        if len(starts) > 0 and len(ends) > 0:
            max_runs.append(int(np.max(ends - starts)))
            
    return np.array(max_runs)


def to_yaml_str(data: dict, indent=0) -> str:
    """Convert dict to standard YAML format string."""
    lines = []
    spacer = " " * indent
    for k, v in data.items():
        if isinstance(v, dict):
            lines.append(f"{spacer}{k}:")
            lines.append(to_yaml_str(v, indent + 2))
        elif isinstance(v, list):
            if not v:
                lines.append(f"{spacer}{k}: []")
            elif all(isinstance(x, (int, float)) for x in v):
                lines.append(f"{spacer}{k}: [{', '.join(str(x) for x in v)}]")
            else:
                lines.append(f"{spacer}{k}:")
                for x in v:
                    lines.append(f"{spacer}- {x}")
        elif v is None:
            lines.append(f"{spacer}{k}: null")
        elif isinstance(v, bool):
            lines.append(f"{spacer}{k}: {str(v).lower()}")
        elif isinstance(v, str):
            if any(char in v for char in [":", "#", "[", "]", "{", "}", ",", " ", "*", "&", "!"]):
                escaped = v.replace('"', '\\"')
                lines.append(f'{spacer}{k}: "{escaped}"')
            else:
                lines.append(f"{spacer}{k}: {v}")
        elif isinstance(v, (int, float)):
            if isinstance(v, float):
                if v.is_integer():
                    lines.append(f"{spacer}{k}: {int(v)}")
                else:
                    lines.append(f"{spacer}{k}: {v:.4f}")
            else:
                lines.append(f"{spacer}{k}: {v}")
        else:
            lines.append(f"{spacer}{k}: {v}")
    return "\n".join(lines)


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 evaluate_map_quality.py /path/to/map_directory", file=sys.stderr)
        sys.exit(1)
        
    map_dir = Path(sys.argv[1])
    if not map_dir.is_dir():
        print(f"Error: Map directory does not exist: {map_dir}", file=sys.stderr)
        sys.exit(1)
        
    yaml_path = map_dir / "map.yaml"
    if not yaml_path.exists():
        print(f"Error: map.yaml not found in {map_dir}", file=sys.stderr)
        sys.exit(1)
        
    # 1. Parse YAML metadata
    try:
        metadata = load_yaml(yaml_path)
    except Exception as exc:
        print(f"Error: Failed to parse map.yaml: {exc}", file=sys.stderr)
        sys.exit(1)
        
    image_rel = metadata.get("image", "map.pgm")
    resolution = metadata.get("resolution", 0.05)
    origin = metadata.get("origin", [0.0, 0.0, 0.0])
    occupied_thresh = metadata.get("occupied_thresh", 0.65)
    free_thresh = metadata.get("free_thresh", 0.25)
    mode = metadata.get("mode", "trinary")
    
    pgm_path = map_dir / image_rel
    if not pgm_path.exists():
        # Fallback to standard name
        pgm_path = map_dir / "map.pgm"
        if not pgm_path.exists():
            print(f"Error: PGM file not found: {pgm_path}", file=sys.stderr)
            sys.exit(1)
            
    # 2. Read PGM image
    try:
        grid, width, height = read_pgm_p5(pgm_path)
    except Exception as exc:
        print(f"Error: Failed to read PGM image: {exc}", file=sys.stderr)
        sys.exit(1)
        
    total_pixels = width * height
    issues = []
    
    # 3. Apply thresholding based on occupancy probability rules
    p = (255.0 - grid) / 255.0
    if mode == "trinary":
        # ROS 205 pixel mapping as unknown
        unknown_mask = (grid == 205)
        occupied_mask = (~unknown_mask) & (p > occupied_thresh)
        free_mask = (~unknown_mask) & (p < free_thresh)
        unknown_mask = unknown_mask | (~occupied_mask & ~free_mask)
    else:
        occupied_mask = (p > occupied_thresh)
        free_mask = (p < free_thresh)
        unknown_mask = ~occupied_mask & ~free_mask
        
    free_count = int(np.sum(free_mask))
    occupied_count = int(np.sum(occupied_mask))
    unknown_count = int(np.sum(unknown_mask))
    
    # M1. Free Space Ratio
    free_ratio = free_count / total_pixels if total_pixels > 0 else 0.0
    unknown_ratio = unknown_count / total_pixels if total_pixels > 0 else 0.0
    score_free = 100.0 if free_ratio >= 0.70 else max(0.0, 100.0 * (free_ratio / 0.70))
    score_unknown_m1 = 100.0 if unknown_ratio <= 0.15 else max(0.0, 100.0 * (1.0 - (unknown_ratio - 0.15) / (0.45 - 0.15)))
    score_m1 = min(score_free, score_unknown_m1)
    if free_ratio < 0.60:
        issues.append(f"Free space ratio is below 60% (value: {free_ratio:.4f})")
    if unknown_ratio > 0.30:
        issues.append(f"Unknown space ratio is above 30% (value: {unknown_ratio:.4f})")
        
    # M2. Unknown Space Ratio
    score_m2 = 100.0 if unknown_ratio <= 0.10 else (0.0 if unknown_ratio >= 0.40 else 100.0 * (1.0 - (unknown_ratio - 0.10) / (0.40 - 0.10)))
    
    # M3. Obstacle Boundary Sharpness
    free_up = np.zeros_like(free_mask)
    free_down = np.zeros_like(free_mask)
    free_left = np.zeros_like(free_mask)
    free_right = np.zeros_like(free_mask)
    free_up[:-1, :] = free_mask[1:, :]
    free_down[1:, :] = free_mask[:-1, :]
    free_left[:, :-1] = free_mask[:, 1:]
    free_right[:, 1:] = free_mask[:, :-1]
    free_neighbors = free_up | free_down | free_left | free_right
    boundary_cells_mask = occupied_mask & free_neighbors
    boundary_cells = int(np.sum(boundary_cells_mask))
    ratio_m3 = boundary_cells / occupied_count if occupied_count > 0 else 0.0
    score_m3 = 100.0 if ratio_m3 >= 0.50 else (0.0 if ratio_m3 <= 0.10 else 100.0 * (ratio_m3 - 0.10) / (0.50 - 0.10))
    if ratio_m3 < 0.40 and occupied_count > 0:
        issues.append(f"Obstacle boundary sharpness is below 40% (value: {ratio_m3:.4f})")
        
    # M4. Free Space Connectivity
    largest_free_component = find_largest_free_component(free_mask)
    ratio_m4 = largest_free_component / free_count if free_count > 0 else 0.0
    score_m4 = 100.0 if ratio_m4 >= 0.95 else (0.0 if ratio_m4 <= 0.50 else 100.0 * (ratio_m4 - 0.50) / (0.95 - 0.50))
    if ratio_m4 < 0.90 and free_count > 0:
        issues.append(f"Free space connectivity is below 90% (value: {ratio_m4:.4f})")
        
    # M5. Narrow Corridor Check
    runs = get_max_runs(free_mask)
    runs = runs[runs >= 3]  # 过滤噪声：小于 3 像素的 run 不算走廊
    if len(runs) == 0:
        min_corridor = 0.0
        median_corridor = 0.0
    else:
        min_corridor = float(np.min(runs)) * resolution
        median_corridor = float(np.median(runs)) * resolution
    score_m5 = 100.0 if min_corridor >= 0.8 else (0.0 if min_corridor <= 0.2 else 100.0 * (min_corridor - 0.2) / (0.8 - 0.2))
    if min_corridor < 0.6:
        issues.append(f"Minimum corridor width is below robot width 0.6m (value: {min_corridor:.4f}m)")
        
    # M6. Resolution Consistency
    score_m6 = 100.0 if 0.03 <= resolution <= 0.07 else (50.0 if 0.02 <= resolution <= 0.10 else 0.0)
    if resolution > 0.10:
        issues.append(f"Resolution is coarser than 0.10m (value: {resolution}m)")
    elif resolution < 0.01:
        issues.append(f"Resolution is finer than 0.01m (value: {resolution}m)")
        
    # Check map dimensions vs metadata.yaml consistency
    metadata_yaml_path = map_dir / "metadata.yaml"
    if metadata_yaml_path.exists():
        try:
            meta = load_yaml(metadata_yaml_path)
            if meta:
                meta_w = meta.get("width")
                meta_h = meta.get("height")
                meta_res = meta.get("resolution")
                if meta_w is not None and meta_w != width:
                    issues.append(f"PGM width ({width}) does not match metadata.yaml width ({meta_w})")
                if meta_h is not None and meta_h != height:
                    issues.append(f"PGM height ({height}) does not match metadata.yaml height ({meta_h})")
                if meta_res is not None and abs(meta_res - resolution) > 1e-5:
                    issues.append(f"YAML resolution ({resolution}) does not match metadata.yaml resolution ({meta_res})")
        except Exception as e:
            issues.append(f"Failed to read/parse metadata.yaml: {e}")
            
    # M7. 3D-2D Projection Quality (optional)
    pcd_path = map_dir / "pointcloud_map_3d.pcd"
    pcd_data = read_pcd_ascii(pcd_path)
    score_m7 = None
    density = None
    
    if pcd_data is not None:
        points, fields = pcd_data
        pcd_x_min, pcd_y_min, _ = points.min(axis=0)
        pcd_x_max, pcd_y_max, _ = points.max(axis=0)
        
        # PGM extent
        pgm_x_min = origin[0]
        pgm_x_max = origin[0] + width * resolution
        pgm_y_min = origin[1]
        pgm_y_max = origin[1] + height * resolution
        
        box_match = (
            (abs(pcd_x_min - pgm_x_min) <= 2.0) and
            (abs(pcd_x_max - pgm_x_max) <= 2.0) and
            (abs(pcd_y_min - pgm_y_min) <= 2.0) and
            (abs(pcd_y_max - pgm_y_max) <= 2.0)
        )
        
        xy_area = (pcd_x_max - pcd_x_min) * (pcd_y_max - pcd_y_min)
        density = len(points) / xy_area if xy_area > 0 else 0.0
        
        score_m7 = 100.0 if density >= 200.0 else (50.0 + (density - 50.0) / 150.0 * 50.0 if density >= 50.0 else (0.0 + (density - 10.0) / 40.0 * 50.0 if density >= 10.0 else 0.0))
        if not box_match:
            issues.append("3D PCD XY bounding box does not match 2D map extent (tolerance ±2.0m)")
            score_m7 = max(0.0, score_m7 - 30.0)
        if density < 50.0:
            issues.append(f"PCD point density is below 50 points/m2 (value: {density:.2f})")
            
    # Calculate weighted total score
    weights = {
        "M1": 20.0,
        "M2": 15.0,
        "M3": 15.0,
        "M4": 20.0,
        "M5": 10.0,
        "M6": 10.0,
        "M7": 10.0
    }
    
    total_score = 0.0
    sum_weights = 0.0
    
    total_score += score_m1 * weights["M1"]
    sum_weights += weights["M1"]
    
    total_score += score_m2 * weights["M2"]
    sum_weights += weights["M2"]
    
    total_score += score_m3 * weights["M3"]
    sum_weights += weights["M3"]
    
    total_score += score_m4 * weights["M4"]
    sum_weights += weights["M4"]
    
    total_score += score_m5 * weights["M5"]
    sum_weights += weights["M5"]
    
    total_score += score_m6 * weights["M6"]
    sum_weights += weights["M6"]
    
    if score_m7 is not None:
        total_score += score_m7 * weights["M7"]
        sum_weights += weights["M7"]
        
    final_score = total_score / sum_weights if sum_weights > 0 else 0.0
    
    # Determine grade
    if final_score >= 95.0:
        grade = "A+"
    elif final_score >= 90.0:
        grade = "A"
    elif final_score >= 85.0:
        grade = "B+"
    elif final_score >= 75.0:
        grade = "B"
    elif final_score >= 70.0:
        grade = "C+"
    elif final_score >= 60.0:
        grade = "C"
    elif final_score >= 50.0:
        grade = "D+"
    elif final_score >= 45.0:
        grade = "D"
    else:
        grade = "F"
        
    # Determine Industrial level
    if final_score >= 95.0:
        ind_level = "L5_survey_grade"
    elif final_score >= 85.0:
        ind_level = "L4_production_grade"
    elif final_score >= 70.0:
        ind_level = "L3_reliable_navigation"
    elif final_score >= 45.0:
        ind_level = "L2_basic_navigation"
    else:
        ind_level = "L1_exploration"
        
    # Construct Report
    metrics_report = {
        "M1_free_ratio": {
            "value": float(free_ratio),
            "score": float(score_m1),
            "standard": "free >= 60%, industrial nav-grade"
        },
        "M2_unknown_ratio": {
            "value": float(unknown_ratio),
            "score": float(score_m2),
            "standard": "unknown <= 20%"
        },
        "M3_boundary_sharpness": {
            "value": float(ratio_m3),
            "score": float(score_m3),
            "standard": "boundary_cells >= 40% of occupied"
        },
        "M4_connectivity": {
            "value": float(ratio_m4),
            "score": float(score_m4),
            "standard": "largest component >= 90% of free"
        },
        "M5_corridor_check": {
            "value": float(min_corridor),
            "score": float(score_m5),
            "standard": "min corridor >= 0.6m"
        },
        "M6_resolution_consistency": {
            "value": float(resolution),
            "score": float(score_m6),
            "standard": "0.02m <= res <= 0.10m"
        },
        "M7_projection_quality": {
            "value": float(density) if density is not None else None,
            "score": float(score_m7) if score_m7 is not None else None,
            "standard": "density >= 50 (outdoor) / 200 (indoor)" if score_m7 is not None else "skipped (no ASCII PCD)"
        }
    }
    
    report = {
        "map_directory": str(map_dir.resolve()),
        "timestamp": datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "resolution_m": float(resolution),
        "dimensions": [int(width), int(height)],
        "pixel_counts": {
            "free": int(free_count),
            "occupied": int(occupied_count),
            "unknown": int(unknown_count)
        },
        "metrics": metrics_report,
        "total_score": float(final_score),
        "grade": grade,
        "industrial_level": ind_level,
        "issues": issues
    }
    
    # Render with comments
    yaml_output = to_yaml_str(report)
    yaml_lines = yaml_output.splitlines()
    for idx, line in enumerate(yaml_lines):
        if line.startswith("grade:"):
            yaml_lines[idx] = line + "  # A(>=90), B(>=75), C(>=60), D(>=45), F(<45)"
        elif line.startswith("industrial_level:"):
            yaml_lines[idx] = line + "  # L1_exploration: score<45, L2_basic_navigation: 45<=score<70, L3_reliable_navigation: 70<=score<85, L4_production_grade: 85<=score<95, L5_survey_grade: score>=95"
            
    print("\n".join(yaml_lines))


if __name__ == "__main__":
    main()
