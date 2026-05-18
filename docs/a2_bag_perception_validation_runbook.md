# A2 Perception Bag Validation Runbook

## 1. 采集目的

验证 JT128 + ground_segmentation_cpp 管道的 self-filter 和 obstacle segmentation：

- **self-filter**：近零点、自车点、雷达支架/机身点是否被正确过滤，不泄漏到 STOP polygon
- **obstacle**：真实障碍是否保留、位置是否正确
- **ground**：地面点分布是否合理
- **traversability**：costmap 是否与场景一致，假障碍是否产生

## 2. 采集场景

| # | 场景名 | 摆放 | 目的 |
|---|--------|------|------|
| 1 | `empty_front_clear` | 前方 ≥3m 无障碍，地面平整 | 验证空场景 STOP polygon 不应有假点 |
| 2 | `box_front_1m` | 正前方 ~1m 放纸箱/障碍 | 验证真实障碍进入 obstacle 点云 |
| 3 | `low_obstacle_front` | 前方 0.8-1.2m 放低矮障碍（木条/门槛模拟物） | 验证低矮障碍不被误分为 ground |
| 4 | `side_obstacle_or_wall` | 侧前方有墙面/桌腿，正前方通畅 | 验证侧向障碍定位正确，不污染正前方 STOP 区 |

**所有场景要求机器人静止，急停在手边。**

## 3. 摆放要求

### empty_front_clear
- 前方 ≥3m 无任何物体
- 地面平整（水泥/瓷砖/短毛地毯）
- 侧方如有墙距 ≥1.5m

### box_front_1m
- 纸箱尺寸建议 30×30×30cm 以上
- 正前方 1m（从 base_link origin 测量）
- 纸箱不靠墙，背景空旷更佳
- 记录纸箱实际尺寸和位置

### low_obstacle_front
- 障碍高度 5-15cm（模拟门槛、低木条）
- 正前方 0.8-1.2m
- 记录障碍实际高度、材质
- 可并用 2-3 个不同高度障碍物

### side_obstacle_or_wall
- 侧前方 0.7-1.5m 放墙面/桌腿/椅子
- 正前方保持通畅 ≥2m
- 记录障碍实际位置（相对于 base_link 的 x, y）

## 4. 采集命令

### 4.1 采集前检查

```bash
ssh unitree@192.168.31.49
docker exec -it a2_system_ws.real bash

source /opt/ros/humble/setup.bash
source /opt/a2_system_ws/install/setup.bash

# 检查雷达工作
ros2 topic hz /jt128/front/points
ros2 topic echo /jt128/front/points --once --field header.frame_id

# 检查 TF
ros2 run tf2_ros tf2_echo base_link jt128_front_link
```

期望：
- `/jt128/front/points` 频率 ~10Hz
- frame_id 为 `jt128_front_link`
- `base_link → jt128_front_link` TF 存在

### 4.2 流程 A：raw-only bag（推荐）

**只录原始传感器数据**，不录算法输出。算法输出在离线回放时用当前源码重新生成。

```bash
mkdir -p /opt/a2_system_ws/runtime/bag_validation/raw

export SCENE="empty_front_clear"
export TAG=$(date +%Y%m%d_%H%M%S)
export BAG_DIR="/opt/a2_system_ws/runtime/bag_validation/raw/${TAG}_${SCENE}"

ros2 bag record \
  -o "${BAG_DIR}" \
  /tf \
  /tf_static \
  /jt128/front/points \
  /a2/imu/data
```

录制 15-20 秒后 `Ctrl+C`。

### 4.3 流程 B：full bag（已有输出话题时快速诊断）

如果 bag 已经包含算法输出话题，可直接分析。但报告必须标注 `--mode recorded`（默认），表明验证的是 bag 内已有输出，不是当前源码。

```bash
ros2 bag record \
  -o "${BAG_DIR}" \
  /tf \
  /tf_static \
  /jt128/front/points \
  /a2/imu/data \
  /a2/perception/ground_segmentation/status \
  /a2/ground/points \
  /a2/obstacle/points \
  /a2/traversability \
  /a2/traversability/obstacle_points
```

### 4.4 写 notes.md

每组 bag 同目录创建 `notes.md`：

```markdown
scene: empty_front_clear
date: 2026-05-17
operator: <name>
robot_pose: 静止，base_link origin 在房间中央
ground_type: 水泥地面
front_clear_distance: >3m
obstacle_type: 无
obstacle_position: N/A
obstacle_height: N/A
expected_result: STOP polygon 内 obstacle 点 p95≤3, max≤10
```

## 5. 离线回放与再生输出（流程 A）

### 5.1 启动回放

使用独立 ROS_DOMAIN_ID 避免干扰现场系统：

```bash
export ROS_DOMAIN_ID=91
source /opt/ros/humble/setup.bash
source /opt/a2_system_ws/install/setup.bash
```

**终端 1：播放 raw bag**

```bash
ros2 bag play /opt/a2_system_ws/runtime/bag_validation/raw/<BAG_NAME> \
  --clock \
  --loop \
  --rate 1.0
```

**终端 2：启动 ground_segmentation（当前源码）**

```bash
ros2 run a2_ground_segmentation_cpp ground_segmentation_cpp_node \
  --ros-args \
  -r __node:=ground_segmentation \
  -p input_topic:=/jt128/front/points \
  -p ground_topic:=/a2/ground/points \
  -p obstacle_topic:=/a2/obstacle/points \
  -p traversability_topic:=/a2/traversability \
  -p target_frame:=map \
  -p base_frame:=base_link \
  -p input_min_range_m:=0.15 \
  -p self_filter_enabled:=true \
  -p self_filter_min_x:=-0.45 \
  -p self_filter_max_x:=0.45 \
  -p self_filter_min_y:=-0.35 \
  -p self_filter_max_y:=0.35 \
  -p self_filter_min_z:=-0.20 \
  -p self_filter_max_z:=0.45
```

**终端 3：启动 traversability → obstacle cloud（当前源码）**

```bash
ros2 run a2_system traversability_to_obstacle_cloud.py \
  --ros-args \
  -p traversability_topic:=/a2/traversability \
  -p output_topic:=/a2/traversability/obstacle_points \
  -p output_frame:=base_link \
  -p unknown_policy:=ignore \
  -p publish_unknown_as_obstacle:=false \
  -p lethal_threshold:=70
```

**终端 4：录制再生输出**

```bash
mkdir -p /opt/a2_system_ws/runtime/bag_validation/processed

ros2 bag record \
  -o /opt/a2_system_ws/runtime/bag_validation/processed/<BAG_NAME>_processed \
  /tf \
  /tf_static \
  /jt128/front/points \
  /a2/perception/ground_segmentation/status \
  /a2/ground/points \
  /a2/obstacle/points \
  /a2/traversability \
  /a2/traversability/obstacle_points
```

### 5.2 分析再生输出

```bash
python3 /opt/a2_system_ws/src/a2_system/scripts/analyze_perception_bag.py \
  /opt/a2_system_ws/runtime/bag_validation/processed/<BAG_NAME>_processed \
  --scene empty_front_clear \
  --mode regenerated \
  --output-dir /opt/a2_system_ws/runtime/bag_validation/reports
```

## 6. 快速诊断（流程 B，已有输出话题的 bag）

```bash
python3 /opt/a2_system_ws/src/a2_system/scripts/analyze_perception_bag.py \
  /opt/a2_system_ws/runtime/bag_validation/raw/<BAG_NAME> \
  --scene empty_front_clear \
  --mode recorded \
  --output-dir /opt/a2_system_ws/runtime/bag_validation/reports
```

## 7. 分析脚本输出

输出文件：
- `runtime/bag_validation/reports/<BAG_NAME>_summary.json` — 机器可读
- `runtime/bag_validation/reports/<BAG_NAME>_report.md` — 人类可读，含 ✅/❌ pass/fail

### 7.1 Scene 自动推断

脚本从 bag 目录名推断 scene：
- 目录名含 `empty_front_clear` → `empty_front_clear`
- 目录名含 `box_front_1m` → `box_front_1m`
- 等等

如果推断失败，用 `--scene <name>` 显式指定。

### 7.2 Mode 说明

| mode | 含义 |
|------|------|
| `recorded` | 验证 bag 内已有输出（不保证是当前源码生成） |
| `regenerated` | 验证当前源码重新生成的输出 |

## 8. RViz 查看方法

```bash
rviz2 -d /opt/a2_system_ws/src/a2_bringup/rviz/a2_bag_perception_validation.rviz
```

| 显示项 | 颜色 | 用途 |
|--------|------|------|
| JT128 Raw | 灰色（intensity） | 原始点云 |
| Ground | 绿色 | 地面点（被过滤的） |
| Obstacle | 红色 | 障碍点（保留的） |
| Traversability Costmap | costmap 调色板 | 可通行性代价 |
| Traversability Obstacle | 橙色 | 从 traversability 转换的障碍点 |

**Debug 层（默认关闭，需要 V2 traversability 发布对应 OccupancyGrid）：**
- Debug Slope / Roughness / Step / Confidence / Reason

**关键观察位置**：
- 切换到 `base_link` 参考系，看 obstacle 点在前方 STOP polygon（x=[-0.3, 0.5], y=[-0.4, 0.4]）内的分布
- 如有障碍物，确认 obstacle 点集中在真实障碍位置
- 检查 base_link 附近（self box）是否有大量残留点

## 9. 每个指标怎么看

| 指标 | 含义 | 好 | 坏 |
|------|------|----|----|
| STOP polygon p95 | 95% 帧的 STOP 区内 obstacle 点数 | empty: ≤3 | empty: >10 |
| STOP polygon max | 任一帧 STOP 区内最大 obstacle 点数 | empty: ≤10 | empty: >20 |
| Self-filter dropped p50 | self-filter 每帧丢弃点中位数 | >0（filter 在工作中） | =0（filter 未工作） |
| near_zero_005 ratio | range<0.05m 点占输入比例 | <5% | >10%（JT128 异常近点） |
| Forward obstacle 0-0.5m | 紧前方 obstacle 点 | empty: near 0 | empty: >5 |
| Traversability mean cost | 平均 cost | <80 | >90（大面积高 cost） |
| Lethal cells p95 | 95% 帧的 lethal 单元格数 | scene-dependent | 空场景 >50 |
| Ground z mean | 地面平均高度 | robot 实际地面高度 ±0.05 | 明显偏移 |

**注：STOP/self/forward 统计均在 obstacle cloud 转换到 base_link 后进行。**

## 10. 通过/失败标准

### empty_front_clear
- STOP polygon p95 ≤ 3
- STOP polygon max ≤ 10
- Forward 0-0.5m p95 ≤ 3
- TF missing ratio == 0

### box_front_1m
- Forward 0.5-1.5m 有 obstacle 点（p50>0 或 p95>10）
- STOP polygon p95 ≤ 5（1m 外的障碍不应持续触发 STOP）
- TF missing ratio == 0

### low_obstacle_front
- Forward 0.5-1.5m obstacle p95 > 0
- Traversability lethal cells p95 > 0 或 max_cost p95 ≥ 70
- TF missing ratio == 0

### side_obstacle_or_wall
- STOP polygon p95 ≤ 5
- Forward 0-0.5m p95 ≤ 5
- TF missing ratio == 0

### unknown
- 不输出 PASS/FAIL 判定，只输出 metrics。

### TF 缺失特殊规则
如果 obstacle frame 不是 `base_link` 且 TF 缺失：
- STOP/self/forward 统计标记为 INVALID（使用未转换原始坐标）
- 报告输出 `tf_missing_frames` 和 `tf_missing_ratio_pct`
- 场景验收中 TF missing ratio == 0 检查 FAIL

## 11. 常见失败点排查

| 现象 | 可能原因 | 排查 |
|------|---------|------|
| STOP polygon 大量点（空场景） | self_filter 未生效 | 检查 `self_filter_enabled` 参数和 status topic |
| 真实障碍未检测 | ground_segmentation 把障碍当 ground | 检查 `general_max_slope_deg`，尝试降低 |
| 低矮障碍被标为 ground | `min_height_threshold` 太高 | 检查该参数，当前默认 0.15m |
| 大量 near-zero 点 | JT128 返回异常近点 | 检查雷达安装、`input_min_range_m` |
| Traversability 全是 unknown | TF 断裂 | 检查 `target_frame` 和 `base_frame` TF 链 |
| Obstacle 点全部在 base_link 附近 | 自车点未被过滤 | 检查 `self_filter_min/max` 参数 |
| TF missing ratio > 0 | bag 内无 `/tf` 或 `/tf_static` | 确认 bag 录了 TF 话题，或用 `--clock` replay |

## 12. 产物目录

```
runtime/bag_validation/
├── raw/                          # raw bag（只录传感器）
│   └── YYYYMMDD_HHMMSS_empty_front_clear/
├── processed/                    # regenerated bag（当前源码再生输出）
│   └── YYYYMMDD_HHMMSS_empty_front_clear_processed/
└── reports/                      # 分析报告
    ├── *_summary.json
    └── *_report.md
```
