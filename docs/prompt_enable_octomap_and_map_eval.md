# Task: Enable OctoMap + LTU Projection & Create Map Quality Evaluator

## Context

This is an A2 robot ROS2 Humble workspace at `/home/dell/a2_system_ws`.
The robot runs inside a Docker container (`a2-nav`) on a remote machine (ssh alias: `a2`).
Docker workspace root inside container: `/opt/a2_system_ws`.

Current problem: 3D→2D map projection is disabled. We need to enable it and build a map quality evaluator.

---

## Part 1: Enable OctoMap Building (3 changes)

### Change 1.1 — Remove `--no-octomap` from 3D stack script

**File**: `/home/dell/a2_system_ws/src/a2_system/tools/start_jt128_3d_stack.sh`

Around line 374-377, current code:
```bash
"$DLIO_MAPPING_SCRIPT" \
  --iface "$LIDAR_IFACE" \
  --no-web \
  --no-octomap
```

Change to:
```bash
"$DLIO_MAPPING_SCRIPT" \
  --iface "$LIDAR_IFACE" \
  --no-web \
  --start-octomap
```

**No other lines should change in this file.**

### Change 1.2 — Add `octomap_server` to Docker image

**File**: `/home/dell/a2_system_ws/Dockerfile`

Find the `apt-get install` block that installs ROS packages. Add `ros-humble-octomap-server` to that list.
If there is already an `octomap` related package, add it adjacent. If not, add it next to other `ros-humble-*` packages.

**Only add one line/entry. Do not reorganize or reformat existing lines.**

### Change 1.3 — Ensure `ltu_octomap_to_2d_grid_cpp` is compiled

**File**: `/home/dell/a2_system_ws/src/a2_system/CMakeLists.txt`

Check if `ltu_octomap_to_2d_grid_cpp` is already listed as a target. Look for lines like:
```cmake
add_executable(ltu_octomap_to_2d_grid_cpp ...)
```
and
```cmake
install(TARGETS ltu_octomap_to_2d_grid_cpp ...)
```

There is also a dependency on a header `MapConverter.hh` from the LTU-RAI library. Check:
1. Is `MapConverter.hh` present somewhere under `/home/dell/a2_system_ws/src/a2_system/`?
2. Is the include path configured in CMakeLists.txt?

**If `ltu_octomap_to_2d_grid_cpp` is already a target but just not being built (e.g. wrapped in a condition), enable it.**
**If `MapConverter.hh` is missing, document this clearly but do NOT create a stub. Just report it.**

---

## Part 2: Map Quality Evaluator Script

Create a **single Python script**: `/home/dell/a2_system_ws/scripts/evaluate_map_quality.py`

### Input
```bash
python3 evaluate_map_quality.py /path/to/map_directory
```

The `map_directory` contains:
- `map.pgm` — Nav2 2D occupancy grid (PGM P5 format)
- `map.yaml` — Nav2 map metadata (resolution, origin, thresholds, mode)
- `pointcloud_map_3d.pcd` (optional) — 3D point cloud
- `map_quality_report.yaml` (optional) — if LTU projection was used

### Evaluation Criteria & Scoring

The script should evaluate and score these metrics (0-100 per metric, weighted total):

#### M1. Free Space Ratio (weight: 20%)
- Read `map.pgm` + `map.yaml`, apply `occupied_thresh` and `free_thresh` from yaml
- Count free / occupied / unknown pixels
- **Industrial standard**: free ≥ 60%, unknown ≤ 30%
- Score: 100 if free≥70% & unknown≤15%; linear falloff to 0

#### M2. Unknown Space Ratio (weight: 15%)
- **Industrial standard**: unknown ≤ 20% for a well-explored map
- Score: 100 if unknown≤10%; 0 if unknown≥40%

#### M3. Obstacle Boundary Sharpness (weight: 15%)
- For each occupied cell, count how many of its 4-neighbors are free
- Ratio = boundary_cells / total_occupied_cells
- **Industrial standard**: ≥ 40% of occupied cells should be boundary cells (not floating blobs)
- Score: 100 if ratio≥50%; 0 if ratio≤10%

#### M4. Free Space Connectivity (weight: 20%)
- BFS/flood-fill from the largest connected free-space region
- largest_component_ratio = largest_free_component / total_free_cells
- **Industrial standard**: ≥ 90% of free cells should be in one connected component
- Score: 100 if ratio≥95%; 0 if ratio≤50%

#### M5. Map Completeness — Narrow Corridor Check (weight: 10%)
- For each row and column, find the maximum continuous free-cell run
- Report the median and minimum corridor widths (in meters, using resolution)
- **Industrial standard**: minimum corridor ≥ robot_width (0.6m for A2)
- Score: 100 if min_corridor≥0.8m; 0 if min_corridor≤0.2m

#### M6. Resolution Consistency (weight: 10%)
- Check if the stated resolution in `map.yaml` matches the actual PGM dimensions vs metric extent
- Also flag if resolution > 0.10m (too coarse) or < 0.01m (overkill)
- **Industrial standard**: 0.02m ≤ resolution ≤ 0.10m
- Score: 100 if 0.03≤res≤0.07; 50 if 0.02≤res≤0.10; 0 otherwise

#### M7. 3D-2D Projection Quality (weight: 10%, skip if no PCD)
- If `pointcloud_map_3d.pcd` exists, load it, count total points
- Check that the PCD XY bounding box roughly matches the PGM extent (±2m tolerance)
- Check point density: points_per_m² on the XY plane
- **Industrial standard**: ≥ 50 points/m² for outdoor, ≥ 200 points/m² for indoor
- Score: 100 if density≥200; 50 if density≥50; 0 if density<10

### Output Format

Print to stdout in YAML:
```yaml
map_directory: /path/to/map
timestamp: 2026-05-22T14:00:00
resolution_m: 0.05
dimensions: [width, height]
pixel_counts:
  free: 12345
  occupied: 678
  unknown: 910
metrics:
  M1_free_ratio:
    value: 0.85
    score: 100
    standard: "free >= 60%, industrial nav-grade"
  M2_unknown_ratio:
    value: 0.06
    score: 100
    standard: "unknown <= 20%"
  # ... etc
total_score: 87.5
grade: B+  # A(>=90), B(>=75), C(>=60), D(>=45), F(<45)
industrial_level: "L2_basic_navigation"
# L1_exploration: score<45, map only good for rough exploration
# L2_basic_navigation: 45<=score<70, can do point-to-point nav with caution
# L3_reliable_navigation: 70<=score<85, reliable for autonomous nav
# L4_production_grade: 85<=score<95, production deployment ready
# L5_survey_grade: score>=95, survey/inspection grade
issues: []  # list of string warnings
```

### Requirements for the script
- Python 3.10+, only use stdlib + numpy (numpy is available in the environment)
- No ROS dependency — pure offline tool
- Handle PGM P5 binary format correctly
- Parse map.yaml for thresholds (default: occupied_thresh=0.65, free_thresh=0.25, mode=trinary)
- Under trinary mode: pixel→probability via `p = (255 - pixel) / 255.0`, then `p > occupied_thresh → occupied`, `p < free_thresh → free`, else `unknown`
- For PCD loading, support ASCII PCD format (FIELDS x y z, DATA ascii). If binary PCD, skip M7 gracefully
- Script should be executable: add shebang `#!/usr/bin/env python3`
- Total file should be ≤ 400 lines

---

## Part 3: Verification Checklist

After completing Parts 1 and 2, provide this checklist for reviewer verification:

### Build verification
```bash
# 1. Check CMakeLists.txt has ltu target
grep -n "ltu_octomap_to_2d_grid" /home/dell/a2_system_ws/src/a2_system/CMakeLists.txt

# 2. Check Dockerfile has octomap_server
grep -n "octomap-server" /home/dell/a2_system_ws/Dockerfile

# 3. Check --no-octomap is removed
grep -n "no-octomap\|start-octomap" /home/dell/a2_system_ws/src/a2_system/tools/start_jt128_3d_stack.sh

# 4. Run evaluator on existing map
python3 /home/dell/a2_system_ws/scripts/evaluate_map_quality.py \
  /home/dell/a2_system_ws/runtime/maps/site_map_20260522_1113

# 5. Confirm no other files were modified
git -C /home/dell/a2_system_ws diff --name-only
```

Expected results:
1. Should show `add_executable(ltu_octomap_to_2d_grid_cpp ...)` and install target
2. Should show `ros-humble-octomap-server`
3. Should show `--start-octomap` (NOT `--no-octomap`)
4. Should print YAML report with total_score and grade
5. Should show exactly these files (±1 for CMakeLists.txt if ltu was already there):
   - `src/a2_system/tools/start_jt128_3d_stack.sh`
   - `Dockerfile`
   - `scripts/evaluate_map_quality.py` (new)
   - `src/a2_system/CMakeLists.txt` (only if changed)

### Critical constraints
- Do NOT modify any file not listed above
- Do NOT add comments to existing code
- Do NOT reformat existing code
- Do NOT delete any existing lines except the `--no-octomap` → `--start-octomap` replacement
- Match existing code style (2-space indent for shell, 4-space for Python)
- If `MapConverter.hh` is missing, just report it in the checklist — do not create stubs
