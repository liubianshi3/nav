# 机器狗导航与任务系统（ROS 2 Humble）

该仓库是一个面向 Unitree 系列机器狗的主机侧 ROS 2 Humble 工作区，提供“传感器接入 → 定位/建图 → Nav2 导航 → 任务编排/安全门控 → Web Console”的一体化链路。

当前工程历史上以 A2 为基线，但核心接口已按“标准 ROS 话题（/cmd_vel、/odom、/imu、/tf、点云）+ 可替换的桥接/传感器 profile”组织，支持在不改代码的情况下切换不同机器狗与不同型号雷达，并便于继续扩展更多型号。

## 核心模块

- Bringup：统一启动入口与编排 [a2_bringup](file:///Users/rick/Workspace/feishu/device-navigation/src/a2_bringup)
- 机器人桥接（状态/控制）
  - 状态采集：`a2_sdk_bridge` → `/a2/raw_state`
  - 状态规范化：`a2_state_publisher` → `/robot_state`、`/odom`、`/imu/data`、TF
  - 速度控制：`a2_control_bridge` 订阅 `/cmd_vel` 并做限速/超时停车/安全门控
- 传感器接入与适配：`sensor_sync`（点云守护、点云转发、点云→scan）
- 定位/重定位：`localization_manager`（AMCL / 手动定位 / 3D 重定位门控）
- 地图/SLAM：`map_manager`、`slam_manager`
- Nav2 集成：`nav2_integration`（goal bridge、3D 目标控制）
- 安全门控：`safety_manager`（雷达/状态/地图/定位就绪判断 → `/a2/allow_motion`、`/a2/estop`）
- 系统级任务编排：`a2_system`（task_manager、运行脚本、工具与文档）
- Web Console：`web_console`（前后端、ROS 桥接）

工程内部“唯一权威”的接口约定文档在：[interface_contracts.md](file:///Users/rick/Workspace/feishu/device-navigation/src/a2_system/docs/interface_contracts.md)

## 支持矩阵（可扩展）

- 机器狗
  - `a2`：现有默认基线
  - `go2_air`：已增加 profile（控制/状态/限速/TF 基高）
  - `b2`：已增加 profile（控制/状态/限速/TF 基高）
- 雷达
  - `hesai_jt128_front`：现有默认基线（专用 Hesai ROS driver）
  - `unitree_go2_air_native`：Go2 Air 内置点云输入（external_pointcloud + relay）
  - `robosense_rs_helios_32`：RoboSense RS-Helios-32 外部点云输入（external_pointcloud + relay）
- 深度相机
  - `realsense_d435i`：Intel RealSense D435i（external_pointcloud，默认使用 `/camera/depth/color/points`）

扩展方式见本文“新增型号”章节。

## 快速开始

### 构建

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 启动（示例）

默认（A2 + JT128）：

```bash
ros2 launch a2_bringup bringup.launch.py \
  runtime_mode:=real \
  network_interface:=<wired_iface>
```

Go2 Air + 内置雷达（点云 topic 默认 `/utlidar/cloud`，按实际设备调整）：

```bash
ros2 launch a2_bringup bringup.launch.py \
  runtime_mode:=real \
  network_interface:=<wired_iface> \
  robot:=go2_air \
  lidar:=unitree_go2_air_native

B2 + RoboSense RS-Helios-32（点云 topic 默认 `/rslidar_points`，按实际设备调整）：

```bash
ros2 launch a2_bringup bringup.launch.py \
  runtime_mode:=real \
  network_interface:=<wired_iface> \
  robot:=b2 \
  lidar:=robosense_rs_helios_32
```

D435i 深度相机加入导航避障（Nav2 voxel_layer 额外 observation source）：

```bash
ros2 launch a2_bringup bringup.launch.py \
  runtime_mode:=real \
  network_interface:=<wired_iface> \
  robot:=go2_air \
  lidar:=unitree_go2_air_native \
  camera:=realsense_d435i
```
```

Nav2 2D 导航（需要已保存的 2D map.yaml）：

```bash
ros2 launch a2_bringup bringup.launch.py \
  runtime_mode:=real \
  network_interface:=<wired_iface> \
  enable_nav2_bringup:=true \
  real_localization_mode:=amcl \
  map:=/abs/path/to/map.yaml
```

## Robot/LiDAR Profile 机制

本仓库通过“可选配置文件 + launch 参数注入”的方式实现多机型/多传感器复用：

- 选择机器狗 profile：`robot:=<name>`，对应配置文件目录 `src/a2_system/config/robots/<name>.yaml`
- 选择雷达 profile：`lidar:=<name>`，对应配置文件目录 `src/a2_system/config/lidars/<name>.yaml`
- 高级用法（直接指定路径）：
  - `robot_config:=/abs/path/to/robot.yaml`
  - `real_lidar_config:=/abs/path/to/lidar.yaml`

当 profile 被选择时：

- `a2_sdk_bridge`、`a2_control_bridge`、`a2_state_publisher` 会加载 profile 中对应节点的参数（覆盖默认值）
- `sensors.launch.py` 会根据 `real_lidar_config` 决定点云接入方式，并从 `robot_config` 读取 `static_tf_manager.base_height`
- `nav2.launch.py` 会按当前雷达的“实际消费 topic”动态改写 `nav2_stack.yaml` 中 costmap voxel_layer 的点云 topic

## 新增型号（扩展指南）

### 新增一款机器狗

1. 新建 `src/a2_system/config/robots/<robot>.yaml`
2. 按需覆盖以下节点参数（只写你要改的字段即可）
   - `a2_sdk_bridge.ros__parameters.*`：例如 `sport_state_topic`、`timer_hz`
   - `a2_control_bridge.ros__parameters.*`：例如 `max_linear_x/max_linear_y/max_yaw_rate`
   - `static_tf_manager.ros__parameters.base_height`：base_footprint → base_link 的高度
3. 启动时指定：`robot:=<robot>`

### 新增一款雷达/点云来源

1. 新建 `src/a2_system/config/lidars/<lidar>.yaml`
2. 关键字段（都在 `real_lidar.ros__parameters` 下）
   - `driver_mode`
     - `dedicated_hesai_ros_driver`：由工程启动外部 Hesai driver（现有 JT128 模式）
     - `external_pointcloud`：外部已产出 `PointCloud2`，工程只做 relay/守护/scan 投影
   - `input_topic`：外部点云输入（如 Go2 内置点云话题）
   - `output_topic`：工程内部统一消费点云（建议与现有栈保持一致，或配合 Nav2 动态改写机制）
   - `output_frame_id`：需要强制写入的 `header.frame_id`（配合静态 TF）
   - `restamp_on_receive`：必要时重打时间戳（优先用于修复“点云时间戳异常导致 TF/代价地图不可用”）
3. 启动时指定：`lidar:=<lidar>`

## 关键话题（最常用）

- 控制：`/cmd_vel`（输入），`/a2/allow_motion`（门控），`/a2/estop`（急停态）
- 状态：`/robot_state`、`/odom`、`/imu/data`
- 点云：由当前 `real_lidar_config` 决定（默认栈为 `/jt128/front/points`）
- 导航：`/navigate_to_pose`（Nav2 action），`/map`（2D），TF（`map→odom→base_link`）

## 相关文档

- 架构：[architecture.md](file:///Users/rick/Workspace/feishu/device-navigation/src/a2_system/docs/architecture.md)
- 接口约定：[interface_contracts.md](file:///Users/rick/Workspace/feishu/device-navigation/src/a2_system/docs/interface_contracts.md)
- 运维手册：[operations_runbook.md](file:///Users/rick/Workspace/feishu/device-navigation/src/a2_system/docs/operations_runbook.md)
