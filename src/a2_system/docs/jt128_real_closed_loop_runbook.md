# A2 JT128 Real Closed-Loop Runbook

本文档固化 2026-05-16 已在 A2 宿主机源码链路跑通的 JT128 3D navigation 路径。标准路径不使用 Docker，不使用 dry-run；导航入口默认启动真实 `a2_control_bridge`。真实行走前仍必须满足 LiDAR、地图、NDT、safety、real readiness 全部 ready。

## 0. 本次已跑通链路复盘

### 0.1 清理运行环境

输入：

```bash
ssh unitree@192.168.31.49
docker stop a2_system_ws.real || true
cd /home/unitree/a2_system_ws
A2_WORKSPACE=/home/unitree/a2_system_ws src/a2_system/tools/stop_jt128_stack.sh
```

依赖条件：

- A2 可 SSH 登录。
- 本次闭环使用 `/home/unitree/a2_system_ws`，不是 `/opt/a2_system_ws` Docker 内源码。

输出：

- Docker 中的 A2 stack 不再运行。
- 旧 `dlio_mapping.launch.py`、`jt128_3d_navigation.launch.py`、Hesai、DLIO、Nav2 等进程被清掉。

成功判断：

```bash
docker ps --format '{{.Names}} {{.Status}}' | grep a2_system || true
ps -ef | grep -E 'dlio_mapping.launch.py|jt128_3d_navigation.launch.py|hesai_ros_driver_node|dlio_odom_node|bt_navigator|controller_server' | grep -v grep
```

第一条应无输出；第二条在启动前应无旧进程。

### 0.2 构建宿主机源码

输入：

```bash
cd /home/unitree/a2_system_ws
source /opt/ros/humble/setup.bash
source /home/unitree/graph_pid_ws/install/setup.bash
source install/setup.bash 2>/dev/null || true
colcon build --symlink-install --packages-select a2_system a2_bringup a2_ndt_adapter
source install/setup.bash
```

依赖条件：

- `graph_pid_ws` 中可见 `hesai_ros_driver`。
- 工作区中可见 `direct_lidar_inertial_odometry`、`a2_system`、`a2_bringup`、`a2_ndt_adapter`。

输出：

- `install/a2_system`、`install/a2_bringup`、`install/a2_ndt_adapter` 更新。

成功判断：

```bash
ros2 pkg prefix hesai_ros_driver
ros2 pkg prefix direct_lidar_inertial_odometry
ros2 pkg prefix a2_system
ros2 pkg prefix a2_bringup
ros2 pkg prefix a2_ndt_adapter
```

全部应返回有效路径。

### 0.3 确认 JT128 网络和配置

输入：

```bash
ip link show net1
ip -4 -o addr show dev net1
ip route get 192.168.124.20
ping -I net1 -c 2 -W 1 192.168.124.20
```

依赖条件：

- JT128 LiDAR IP 为 `192.168.124.20`。
- LiDAR 网口为 `net1`。
- SDK/control 网口为 `eth0`。

输出：

- A2 宿主机能从 `net1` 到达 JT128。

成功判断：

- `ip route get 192.168.124.20` 显示 `dev net1`。
- `ping -I net1` 有回复。

关键配置文件：

- `/home/unitree/a2_system_ws/src/a2_system/config/jt128_front_hesai.yaml`
- `/home/unitree/a2_system_ws/src/a2_system/config/hesai_correction/JT128_Angle Correction File.csv`
- `/home/unitree/a2_system_ws/src/a2_system/config/hesai_correction/JT128_Firetime Correction File.csv`

必须保持：

```yaml
device_ip_address: 192.168.124.20
udp_port: 2368
is_use_ptc: false
ros_frame_id: jt128_front_link
ros_send_point_cloud_topic: /jt128/front/points
ros_send_imu_topic: /jt128/front/imu
correction_file_path: config_files/hs_lidar_jt128/JT128_Angle Correction File.csv
firetimes_path: config_files/hs_lidar_jt128/JT128_Firetime Correction File.csv
```

### 0.4 启动宿主机真实导航栈

输入：

```bash
cd /home/unitree/a2_system_ws
A2_WORKSPACE=/home/unitree/a2_system_ws \
src/a2_system/tools/start_jt128_3d_stack.sh \
  --mode navigation \
  --map-id closed_loop_regression_20260515_121029 \
  --lidar-iface net1 \
  --sdk-iface eth0 \
  --control-iface eth0 \
  --localization-mode ndt \
  --collision-profile strict
```

依赖条件：

- 地图目录存在：

```text
/home/unitree/a2_system_ws/runtime/maps/closed_loop_regression_20260515_121029/
```

- 该目录至少包含：

```text
pointcloud_map_3d.pcd
map.yaml
map.pgm
metadata.yaml
media_index.yaml
```

输出：

- 启动 JT128 Hesai driver。
- 启动 DLIO。
- 启动 NDT adapter。
- 启动 Nav2 3D。
- 启动 collision monitor、safety supervisor、real readiness monitor。
- 启动真实 `a2_control_bridge`，但不会自动运动；运动需要后续 goal。

成功判断：

```bash
cat /home/unitree/a2_system_ws/runtime/jt128_3d_navigation_state.yaml
```

应看到：

```yaml
mode: jt128_3d_navigation
map_id: closed_loop_regression_20260515_121029
lidar_interface: net1
sdk_interface: eth0
control_interface: eth0
enable_motion: true
dry_run: false
enable_nav2_3d: true
nav2_3d_map: /home/unitree/a2_system_ws/runtime/maps/closed_loop_regression_20260515_121029/map.yaml
```

### 0.5 验证 LiDAR、DLIO、地图、NDT

输入：

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/graph_pid_ws/install/setup.bash
source /home/unitree/a2_system_ws/install/setup.bash

timeout 5 ros2 topic hz /jt128/front/points
timeout 5 ros2 topic hz /jt128/front/imu
timeout 5 ros2 topic hz /jt128/dlio/odom
timeout 3 ros2 topic echo /a2/map_ready --once
```

输出和成功判断：

- `/jt128/front/points` 约 `10 Hz`。
- `/jt128/front/imu` 约 `200 Hz`。
- `/jt128/dlio/odom` 约 `100 Hz`。
- `/a2/map_ready` 为 `data: true`。

Hesai 日志中必须看到：

```text
Parser correction file success
```

不应看到：

```text
Open correction file failed
No available angle calibration files
```

### 0.6 发布初始位姿并刷新 NDT readiness

输入：

```bash
now=$(date +%s%N); sec=${now%?????????}; nsec=${now: -9}
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {stamp: {sec: ${sec}, nanosec: ${nsec}}, frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}"
sleep 4
```

依赖条件：

- 地图已加载。
- `/jt128/dlio/odom` 正在更新。
- `/jt128/front/points` 正在更新。

输出：

- NDT adapter latch 当前 pose。
- 发布 `/a2/relocalization/pose`。
- 建立或刷新 `map -> odom`。

成功判断：

```bash
timeout 3 ros2 topic echo /a2/localization/status --once
timeout 3 ros2 topic echo /a2/ndt/healthy --once
timeout 3 ros2 topic echo /a2/safety/status --once
timeout 3 ros2 topic echo /a2/real/report --once
```

应满足：

```text
/a2/localization/status: state=ready;ready=true
/a2/ndt/healthy: data: true
/a2/safety/status: state=allow_motion;ready=true;reason=ok
/a2/real/report: state=ready;ready=true;reason=ok;sdk=true;jt128=true;map=true;localization=true
```

重要时序：

- A2 长时间静止时，NDT health 可能因没有新匹配分数而 stale。
- 如果看到 `localization_not_ready`、`ndt_unhealthy`、`localization_down`，先重新发布 `/initialpose`，再等 3 到 5 秒复查。
- 真实 goal 应在 `/a2/safety/status` 和 `/a2/real/report` 均 ready 后发送。

### 0.7 发送真实 Nav2 goal

手动操作时机：

- 操作员确认机器人周围清空。
- 急停可用。
- `/a2/safety/status` 为 `allow_motion`。
- `/a2/real/report` 为 `ready=true`。
- collision profile 使用 `strict`。

输入示例，小距离验证：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
--feedback
```

输出：

- Nav2 action server 接受 goal。
- `/cmd_vel` 经 collision monitor 过滤到 `/cmd_vel_safe`。
- `a2_control_bridge` 发送真实 Unitree 控制。

成功判断：

```bash
timeout 5 ros2 topic echo /cmd_vel_safe --once
timeout 3 ros2 topic echo /a2/safety/status --once
timeout 3 ros2 topic echo /a2/real/report --once
```

- `/cmd_vel_safe` 有速度指令。
- safety 不进入 blocked。
- 机器人按小距离目标移动且可随时急停。

停止：

```bash
ros2 action cancel /navigate_to_pose nav2_msgs/action/NavigateToPose
A2_WORKSPACE=/home/unitree/a2_system_ws src/a2_system/tools/stop_jt128_stack.sh
```

## 1. 从零开始一次性跑通标准流程

这里的“从零开始”指从干净进程状态启动到真实行走就绪；地图资产必须已经存在。若目标地图不存在，先完成建图和 2D 投影产物生成，再进入本流程。

1. 登录 A2：

```bash
ssh unitree@192.168.31.49
```

2. 停 Docker 和旧进程：

```bash
docker stop a2_system_ws.real || true
cd /home/unitree/a2_system_ws
A2_WORKSPACE=/home/unitree/a2_system_ws src/a2_system/tools/stop_jt128_stack.sh
```

3. 构建宿主机源码：

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/graph_pid_ws/install/setup.bash
source install/setup.bash 2>/dev/null || true
colcon build --symlink-install --packages-select a2_system a2_bringup a2_ndt_adapter
source install/setup.bash
```

4. 检查地图：

```bash
MAP_ID=closed_loop_regression_20260515_121029
ls -lh runtime/maps/${MAP_ID}/pointcloud_map_3d.pcd runtime/maps/${MAP_ID}/map.yaml runtime/maps/${MAP_ID}/map.pgm
```

5. 检查 LiDAR 网络：

```bash
ip route get 192.168.124.20
ping -I net1 -c 2 -W 1 192.168.124.20
```

6. 启动真实导航栈：

```bash
A2_WORKSPACE=/home/unitree/a2_system_ws \
src/a2_system/tools/start_jt128_3d_stack.sh \
  --mode navigation \
  --map-id ${MAP_ID} \
  --lidar-iface net1 \
  --sdk-iface eth0 \
  --control-iface eth0 \
  --localization-mode ndt \
  --collision-profile strict
```

7. 观察日志路径：

```bash
cat runtime/jt128_dlio_stack_state.yaml
cat runtime/jt128_3d_navigation_state.yaml
tail -f runtime/logs/jt128_dlio_mapping_*.log
tail -f runtime/logs/jt128_3d_navigation_*.log
```

8. 验证传感器和里程计：

```bash
timeout 5 ros2 topic hz /jt128/front/points
timeout 5 ros2 topic hz /jt128/front/imu
timeout 5 ros2 topic hz /jt128/dlio/odom
timeout 3 ros2 topic echo /a2/map_ready --once
```

9. 发布 `/initialpose`：

```bash
now=$(date +%s%N); sec=${now%?????????}; nsec=${now: -9}
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{header: {stamp: {sec: ${sec}, nanosec: ${nsec}}, frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}"
sleep 4
```

10. 验证真实行走就绪：

```bash
timeout 3 ros2 topic echo /a2/localization/status --once
timeout 3 ros2 topic echo /a2/ndt/healthy --once
timeout 3 ros2 topic echo /a2/safety/status --once
timeout 3 ros2 topic echo /a2/real/report --once
```

必须同时满足：

```text
localization ready=true
ndt healthy=true
safety state=allow_motion ready=true
real report state=ready ready=true
```

11. 发送小距离真实 goal：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
--feedback
```

12. 停止或恢复：

```bash
ros2 action cancel /navigate_to_pose nav2_msgs/action/NavigateToPose
A2_WORKSPACE=/home/unitree/a2_system_ws src/a2_system/tools/stop_jt128_stack.sh
```

## 2. 跑通前检查清单

### 必须满足

- 使用 A2 宿主机源码：`/home/unitree/a2_system_ws`。
- Docker A2 stack 已停止。
- `a2_system`、`a2_bringup`、`a2_ndt_adapter` 已在宿主机 build。
- `hesai_ros_driver` 来自 `/home/unitree/graph_pid_ws/install/setup.bash`。
- `direct_lidar_inertial_odometry` 可见。
- JT128 IP 为 `192.168.124.20`，路由走 `net1`。
- SDK/control 网口为 `eth0`。
- `jt128_front_hesai.yaml` 使用 JT128 correction/firetime CSV。
- `is_use_ptc: false`。
- 地图目录包含 `pointcloud_map_3d.pcd`、`map.yaml`、`map.pgm`。
- `jt128_3d_navigation.launch.py` 的 safety map topic 使用 `/map`，不是 `/a2/map/pointcloud_3d`。
- collision profile 使用 `strict`。
- `/initialpose` 必须在发送 goal 前发布。
- 发送真实 goal 前必须看到 `/a2/safety/status` 为 `allow_motion`，`/a2/real/report` 为 `ready=true`。

### 推荐满足

- 启动前运行 `stop_jt128_stack.sh`，避免旧进程叠跑。
- Web backend 可启动，但闭环不依赖 Web。
- 先发 `0.2 m` 级别小目标验证。
- 目标点附近无遮挡，操作员手边有急停。
- 每次启动记录 `runtime/jt128_*_state.yaml` 和对应 log 文件。

## 3. 常见失败点与排查表

| 现象 | 快速判断 | 常见原因 | 处理 |
| --- | --- | --- | --- |
| `/jt128/front/points` 无数据 | `ros2 topic hz /jt128/front/points` 无输出 | LiDAR 网络不通、驱动没启动、UDP 2368 未绑定 | 查 `ip route get 192.168.124.20`、`ping -I net1`、`ss -lunp | grep 2368` |
| Hesai 日志提示 correction failed | `tail runtime/logs/jt128_dlio_mapping_*.log` | correction/firetime CSV 没复制到 driver cwd | 检查 `src/a2_system/config/hesai_correction/` 和 `jt128_driver.launch.py` |
| `/jt128/dlio/odom` 无数据 | `ros2 topic hz /jt128/dlio/odom` 无输出 | DLIO 包不可见、LiDAR/IMU 无输入 | 查 `ros2 pkg prefix direct_lidar_inertial_odometry`、`/jt128/front/imu` |
| `map_ready=false` | `/a2/map_ready` false | 地图目录缺 `pointcloud_map_3d.pcd` 或 `map.yaml` | 查 `runtime/maps/<map_id>/` |
| safety 为 `map_not_ready` | `/a2/safety/status` reason | safety 订错地图 topic 或 map server 未发 `/map` | 确认 `map_topic: /map`、`map_representation: occupancy_grid_2d` |
| safety 为 `localization_not_ready,ndt_unhealthy` | `/a2/ndt/healthy` false | 没发 `/initialpose`，或静止太久 NDT stale | 重新发布 `/initialpose`，等 3 到 5 秒 |
| `/a2/real/report` 为 degraded | report 中 `localization=false` 或 `jt128=false` | readiness 依赖未满足 | 逐项查 `/a2/lidar/connected`、`/a2/map_ready`、`/a2/localization/status` |
| Nav2 goal accepted 但不动 | `/cmd_vel_safe` 无输出 | planner/controller/recovery 阶段未产生命令，或 safety blocked | 查 `/a2/safety/status`、Nav2 log、目标是否在可达区域 |
| 多个同名进程 | `ps -ef | grep ...` 出现多份 | 旧 launch 未清理 | 运行 `stop_jt128_stack.sh` 后重启 |
| Docker 与源码结果不一致 | `docker ps` 仍有 A2 容器 | 启错运行环境 | 停 Docker，只用 `/home/unitree/a2_system_ws` |

## 4. 不应随意改动的关键配置清单

| 文件或参数 | 当前成功值 | 原因 |
| --- | --- | --- |
| `src/a2_system/tools/start_jt128_3d_stack.sh` | navigation 默认 `ENABLE_MOTION=true`、`DRY_RUN=false` | 标准路径是真实行走 |
| `src/a2_bringup/launch/jt128_3d_navigation.launch.py` | `enable_motion` 默认 `true`、`dry_run` 默认 `false` | 直接 launch 也应是真实行走默认 |
| `src/a2_system/config/jt128_front_hesai.yaml` | `device_ip_address: 192.168.124.20` | JT128 实际 IP |
| `src/a2_system/config/jt128_front_hesai.yaml` | `udp_port: 2368` | Hesai 点云 UDP 端口 |
| `src/a2_system/config/jt128_front_hesai.yaml` | `is_use_ptc: false` | 本次成功链路不依赖 PTC 握手 |
| `src/a2_system/config/jt128_front_hesai.yaml` | correction/firetime 指向 `config_files/hs_lidar_jt128/*.csv` | Hesai driver runtime cwd 需要相对路径 |
| `src/a2_system/config/hesai_correction/` | 两个 JT128 CSV 文件必须存在 | 无 correction 时驱动会禁止解析点云 |
| `src/a2_bringup/launch/jt128_driver.launch.py` | 启动时复制 `hesai_correction` 到 runtime driver config dir | 保证 driver 能找到 CSV |
| `src/a2_bringup/launch/dlio_mapping.launch.py` | DLIO 输入 `/jt128/front/points`、`/jt128/front/imu` | 本次实际传感器 topic |
| `src/a2_bringup/launch/dlio_mapping.launch.py` | 默认发布压平版 `odom -> base_link`（`start_flattened_odom_tf:=true`）；`start_jt128_dlio_mapping.sh --start-octomap` 路径会设 `start_flattened_odom_tf:=false` | 避免与 `octomap_mapping.launch.py` 内 `octomap_mapping_node` 发布的完整 3D TF 冲突；非 OctoMap / 直接 DLIO mapping 路径仍有压平 TF |
| `src/a2_bringup/launch/octomap_mapping.launch.py` | `octomap_mapping_node` 发布完整 3D `odom -> base_link` TF（`publish_tf: true`） | 3D 建图时由 OctoMap 节点持有 TF，`dlio_mapping.launch.py` 压平 TF 同时被禁用 |
| `src/a2_bringup/launch/jt128_3d_navigation.launch.py` | `ndt_odom_topic: /jt128/dlio/odom` | NDT 初值来自 DLIO |
| `src/a2_bringup/launch/jt128_3d_navigation.launch.py` | safety `map_topic: /map` | safety 使用 projected OccupancyGrid |
| `src/a2_bringup/launch/jt128_3d_navigation.launch.py` | safety `map_representation: occupancy_grid_2d` | 避免把 pointcloud topic 当 2D map |
| `src/a2_bringup/launch/jt128_3d_navigation.launch.py` | `latch_map_ready: True`、`map_transient_local: True` | 避免 safety 启动时错过 latched map |
| 启动参数 | `--lidar-iface net1` | LiDAR 物理网口 |
| 启动参数 | `--sdk-iface eth0 --control-iface eth0` | Unitree SDK/control 网口 |
| 启动参数 | `--localization-mode ndt` | 本次成功闭环使用 NDT，不是 odom_only |
| 启动参数 | `--collision-profile strict` | 真实行走默认使用严格避障 |

## 5. 本次仍需后续清理的问题

- `traversability_to_obstacle_cloud.py` 日志调用存在 rclpy logger 参数错误，日志中可能出现 `TypeError: RcutilsLogger.info() takes 2 positional arguments but 7 were given`。当前不阻塞 readiness，但应单独修复。
- 长时间静止时 NDT health 可能 stale。真实行走前固定执行 `/initialpose` 刷新，并在 ready 窗口内发送 goal。
