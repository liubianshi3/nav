# A2 System Workspace

Host-side ROS 2 Humble workspace for Unitree A2 + MID360, with mock/real mode parity and a Nav2-facing integration boundary.

## A2 real1 启动

如果你现在只关心那台 A2 上怎么先清干扰、再启动真实栈，先看这份：

- [README_A2_Quickstart.md](./README_A2_Quickstart.md)

## Build

```bash
cd /home/dell/a2_system_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Mock Bringup

```bash
cd /home/dell/a2_system_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch a2_bringup bringup.launch.py runtime_mode:=mock

ros2 launch a2_bringup bringup.launch.py runtime_mode:=mock auto_start_explore:=true
```

## Gazebo Bringup

```bash
cd /home/dell/a2_system_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
install/a2_system/share/a2_system/start_gazebo_stack.sh

# 带 GUI
A2_GAZEBO_GUI=true install/a2_system/share/a2_system/start_gazebo_stack.sh

# 启用真实 Nav2 bringup
A2_ENABLE_NAV2=true install/a2_system/share/a2_system/start_gazebo_stack.sh

# 指定地图
A2_ENABLE_NAV2=true \
A2_MAP_YAML=/home/dell/a2_system_ws/install/gazebo_bridge/share/gazebo_bridge/maps/office_house_map.yaml \
install/a2_system/share/a2_system/start_gazebo_stack.sh
```

当前默认 Gazebo world 已切到更复杂的室外场景：

- `src/gazebo_bridge/worlds/outdoor_research_park.world`

Gazebo + Nav2 模式当前已经验证到：

- `map_server / amcl / controller_server / planner_server / bt_navigator` 可完成生命周期激活
- `/a2/real/report` 可进入 `mode=gazebo;state=ready;ready=true`
- 发布 `/a2/exploration/goal` 后，Nav2 会输出路径与速度命令，Gazebo `/odom` 可观测到小车运动

## Gazebo Outdoor Mapping Pipeline

一键验证复杂室外环境下的：

- 扫图
- 建图保存
- 载入刚保存的地图并导航

```bash
cd /home/dell/a2_system_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
install/a2_system/share/a2_system/run_gazebo_outdoor_pipeline.sh outdoor_demo_map_v4
```

本机已实际验证通过，脚本最终输出：

```text
Outdoor Gazebo pipeline PASS
```

产物位置：

- 地图：`runtime/maps/outdoor_demo_map_v4/map.yaml`
- 建图日志：`runtime/outdoor_demo_logs/mapping_outdoor_demo_map_v4.log`
- 用图日志：`runtime/outdoor_demo_logs/navigation_outdoor_demo_map_v4.log`

## Delivery Docs

- `src/a2_system/docs/delivery_v3.md`
- `src/a2_system/docs/architecture.md`
- `src/a2_system/docs/interface_contracts.md`

## Real Bringup Preparation

1. Edit `src/a2_system/config/a2_sdk.yaml`
2. Edit `src/a2_system/config/motion_limits.yaml`
3. Edit `src/a2_system/config/network.yaml`
4. Set `use_mock: false`
5. Fill `network_interface` or replace `interface_candidates`

## Useful Commands

```bash
python3 src/a2_system/scripts/preflight_check.py --config-dir src/a2_system/config
python3 src/mid360_wrapper/mid360_wrapper/mid360_link_check.py --target-ip 192.168.124.20
ros2 service call /map_manager/set_mode a2_interfaces/srv/SetMode "{mode: navigation}"
ros2 service call /map_manager/manage_map a2_interfaces/srv/ManageMap "{command: list, map_id: ''}"
install/a2_system/share/a2_system/start_gazebo_stack.sh
A2_ENABLE_NAV2=true install/a2_system/share/a2_system/start_gazebo_stack.sh
install/a2_system/share/a2_system/configure_real_network.sh enx00e099003cd7
source install/a2_system/share/a2_system/setup_unitree_dds.sh enx00e099003cd7
install/a2_system/share/a2_system/start_real_stack.sh enx00e099003cd7
install/a2_system/share/a2_system/record_bag.sh
install/a2_system/share/a2_system/collect_logs.sh
```
