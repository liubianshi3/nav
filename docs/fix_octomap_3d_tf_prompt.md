# 修复 OctoMap 3D 地图点云重叠畸变

## 问题

OctoMap 建图生成的 `pointcloud_map_3d.pcd` z 跨度达 22.7m（正常应 ≤4m），
点云呈球形重叠无法使用。

## 根因

1. **OctoMap server 依赖的 TF (`odom → base_link`) 是被压平的 2D TF**
   - `odometry_tf_broadcaster.py` 以 `flatten_z=True, planarize_orientation=True` 发布
   - 导致 octomap_server 在将 `jt128_front_link` 帧的点云变换到 `odom` 帧时，
     丢失了 pitch/roll/z 信息
   - A2 是四足机器人，行走时 pitch/roll 摆动大，12m 量程 × 无 pitch 校正 ≈ ±11m z 偏移

2. **`octomap_mapping.launch.py` 中 `lidar_to_base_rotation` 第三行符号错误**
   - 当前值: `[0.0, 1.0, 0.0]`（z_base = +lidar_y = 向下）
   - 正确值: `[0.0, -1.0, 0.0]`（z_base = -lidar_y = 向上）
   - 参考来源: `jt128_extrinsics.yaml` 和 `dlio_jt128.yaml` 均为 `[0.0, -1.0, 0.0]`

## 修改范围（仅 1 个文件）

**文件**: `src/a2_bringup/launch/octomap_mapping.launch.py`

### 改动 1：修正旋转矩阵符号

```python
# 第 42 行
# 旧:
"0.0, 1.0, 0.0]"
# 新:
"0.0, -1.0, 0.0]"
```

### 改动 2：启用 octomap_mapping_node 的 3D TF 发布

```python
# 第 70 行
# 旧:
"publish_tf": False,
# 新:
"publish_tf": True,
```

**效果**: `octomap_mapping_node` 会使用 DLIO 完整 3D 里程计（含 z、pitch、roll）
发布 `odom → base_link` 和 `base_link → jt128_front_link` TF。

### 改动 3：OctoMap server 使用独立 frame_id 避免与导航 2D TF 冲突

```python
# 第 11 行
# 旧:
DeclareLaunchArgument("frame_id", default_value="odom"),
# 新:
DeclareLaunchArgument("frame_id", default_value="odom_3d"),
```

同时在 `octomap_mapping_node.py` 的 `_publish_tf` 方法中，将 `odom_to_base` 的
parent frame 硬编码为 `odom_3d`（而非使用 odom msg 中的 frame_id），这样：
- 导航栈继续使用压平的 `odom → base_link`
- OctoMap server 使用完整 3D 的 `odom_3d → base_link_3d → jt128_front_link`

**但这会增加复杂度**。更简单的替代方案是：

### 替代方案（推荐，最小改动）

不引入 `odom_3d`，而是让 `octomap_mapping_node` 不通过 TF 而是**直接在
`_on_cloud` 中用 DLIO odom 位姿将点云变换到 odom 帧后再发布**。
这样 octomap_server 收到的点云已经在 `odom` 帧中，不需要查 TF。

具体改法：

**文件**: `src/a2_system/scripts/octomap_mapping_node.py`

在 `_on_cloud` 方法中，找到匹配的 odom 位姿后，将每个点从 `jt128_front_link`
坐标系变换到 `odom` 坐标系，并将发布的点云 `header.frame_id` 改为 `odom`。

这样 octomap_server 的 `frame_id=odom` 收到已经在 odom 帧的点云，无需 TF 查询。

---

## 推荐执行顺序

**最小风险修复（只改 launch 文件即可验证）：**

1. 修正旋转矩阵: `0.0, 1.0, 0.0` → `0.0, -1.0, 0.0`
2. 开启 3D TF: `"publish_tf": False` → `"publish_tf": True`
3. 验证：重新建图，检查 PCD 的 z 跨度是否回到 3-4m

如果改动 2 导致 TF 冲突（两个节点同时发布 `odom → base_link`），
则需要在 `dlio_mapping.launch.py` 中给 OctoMap 场景下的 odometry_tf_broadcaster
加一个条件开关，或者采用上面的"替代方案"。

## 不应改动的文件

- `odometry_tf_broadcaster.py` — 导航栈依赖压平的 TF，不能动
- `dlio_jt128.yaml` — DLIO 参数正确，不能动
- `jt128_extrinsics.yaml` — 已正确，不能动
