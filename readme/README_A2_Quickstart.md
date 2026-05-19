# A2 real1 启动指南

这份文档只解决一件事：

- 在那台 A2 真机上，先清干净旧干扰，再启动 `real1` 这一套真实导航栈

这里不讲 Docker、不讲网页按钮切换、不讲建图细节，只讲宿主机命令。

---

## 1. 适用范围

默认你操作的是这台 A2：

```text
ssh unitree@192.168.31.49
```

默认工作区路径：

```text
/home/unitree/a2_system_ws
```

默认网卡：

```text
eth0
```

`real1` 在这里指的就是这条真实启动命令最终拉起的整套真实栈：

```bash
/home/unitree/a2_system_ws/install/a2_system/share/a2_system/start_real_stack.sh eth0
```

---

## 2. 启动前先清干净

先登录 A2：

```bash
ssh unitree@192.168.31.49
```

进入工作区：

```bash
cd /home/unitree/a2_system_ws
```

### 2.1 停掉 Docker 版网页容器

```bash
docker compose -f docker-compose.a2.yml down || true
```

### 2.2 停掉宿主机旧 Web 服务

```bash
sudo systemctl stop a2-web-console.service || true
```

### 2.3 source ROS 环境

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/a2_system_ws/install/setup.bash
```

### 2.4 用官方 stop 脚本停旧栈

```bash
/home/unitree/a2_system_ws/install/a2_system/share/a2_system/stop_stack.sh || true
```

### 2.5 再扫一遍常见残留

```bash
pkill -f "bringup.launch.py" || true
pkill -f "a2_sdk_bridge_node" || true
pkill -f "a2_control_bridge_node" || true
pkill -f "manual_localization_publisher" || true
pkill -f "goal_bridge" || true
pkill -f "occupancy_mapper" || true
pkill -f "map_manager_node" || true
pkill -f "map_server" || true
pkill -f "controller_server" || true
pkill -f "planner_server" || true
pkill -f "bt_navigator" || true
pkill -f "velocity_smoother" || true
pkill -f "lifecycle_manager" || true
```

### 2.6 确认已经清干净

```bash
pgrep -af "bringup.launch.py|a2_sdk_bridge|a2_control_bridge|manual_localization_publisher|goal_bridge|occupancy_mapper|map_manager|map_server|controller_server|planner_server|bt_navigator|velocity_smoother|lifecycle_manager"
```

正常结果：

- 没有输出

如果这里还有输出，不要启动 `real1`，先把残留处理干净。

---

## 3. 启动 real1

推荐入口有两种。

### 3.1 本机一键启动

在开发机 `/home/dell/a2_system_ws` 下直接运行：

```bash
./scripts/start_a2_real1.sh --host a2
```

这条命令会自动：

1. 同步当前工作区到 A2
2. 远端增量构建 ROS 包和 web 前端
3. SSH 进入 A2
4. 关闭已知干扰进程
5. 拉起原生前雷达链
6. 拉起 `real1` 导航栈
7. 拉起 web 前端

如果你已经刚部署过，不想重复同步，可以：

```bash
./scripts/start_a2_real1.sh --host a2 --no-deploy
```

如果你知道初始位姿，可以直接一起发：

```bash
./scripts/start_a2_real1.sh --host a2 --initial-pose 0.0 0.0 0.0
```

### 3.2 A2 本机一键启动

如果你已经登录到 A2，本机入口是：

```bash
/home/unitree/a2_system_ws/src/a2_system/tools/start_real1_suite.sh \
  --iface eth0 \
  --map-yaml /home/unitree/a2_system_ws/runtime/maps/test_map_20260423_1059/map.yaml
```

### 3.3 底层 bringup 入口

如果你只想直接起真实导航栈，不带 web/清理流程，也可以用下面这条：

```bash
A2_ENABLE_NAV2=true \
A2_MAP_YAML=/home/unitree/a2_system_ws/runtime/maps/test_map_20260423_1059/map.yaml \
/home/unitree/a2_system_ws/install/a2_system/share/a2_system/start_real_stack.sh eth0
```

如果你要换地图，只改 `A2_MAP_YAML`：

```bash
A2_ENABLE_NAV2=true \
A2_MAP_YAML=/home/unitree/a2_system_ws/runtime/maps/<你的地图目录>/map.yaml \
/home/unitree/a2_system_ws/install/a2_system/share/a2_system/start_real_stack.sh eth0
```

这条命令会做三件事：

1. 检查真实网络接口
2. 配置真实 DDS/网络环境
3. 后台拉起 `bringup.launch.py runtime_mode:=real`

默认真实定位模式：

- `amcl`
- 只有明确排障或临时兼容时，才应手动覆盖 `A2_REAL_LOCALIZATION_MODE`

启动成功后，终端通常会打印：

```text
Started real bringup pid=...
Log file: /home/unitree/a2_system_ws/runtime/logs/bringup_real_*.log
network_interface=eth0
```

---

## 4. 启动后怎么确认成功

### 4.1 先看进程

```bash
pgrep -af "bringup.launch.py|a2_sdk_bridge|a2_control_bridge|goal_bridge|map_server|amcl|controller_server|planner_server|bt_navigator|velocity_smoother|lifecycle_manager"
```

正常情况应该能看到：

- `bringup.launch.py`
- `a2_sdk_bridge`
- `a2_control_bridge`
- `goal_bridge`
- `map_server`
- `amcl`
- `controller_server`
- `planner_server`
- `bt_navigator`
- `velocity_smoother`
- `lifecycle_manager`

如果这里看到的是 `manual_localization_publisher` 而没有 `amcl`，说明当前不是默认实机复现路径，应先检查 `A2_REAL_LOCALIZATION_MODE` 是否被手动覆盖。

### 4.2 再看系统状态

```bash
ros2 topic echo --once /a2/real/report
ros2 topic echo --once /a2/localization/status
ros2 topic echo --once /a2/control/status
```

理想状态是：

- `/a2/real/report` 里有 `ready=true`
- `/a2/localization/status` 里有 `ready=true`
- `/a2/control/status` 至少是节点在线，不是彻底缺失

### 4.3 如果要看日志

最新日志在这里：

```bash
ls -lt /home/unitree/a2_system_ws/runtime/logs/bringup_real_*.log | head
```

直接跟日志：

```bash
tail -f $(ls -t /home/unitree/a2_system_ws/runtime/logs/bringup_real_*.log | head -1)
```

---

## 5. 停止 real1

停栈只用这条：

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/a2_system_ws/install/setup.bash
/home/unitree/a2_system_ws/install/a2_system/share/a2_system/stop_stack.sh
```

停完后再确认一次：

```bash
pgrep -af "bringup.launch.py|a2_sdk_bridge|a2_control_bridge|manual_localization_publisher|goal_bridge|occupancy_mapper|map_manager|map_server|controller_server|planner_server|bt_navigator|velocity_smoother|lifecycle_manager"
```

正常结果：

- 没有输出

---

## 6. 一条最短路径

如果你只想照抄：

```bash
ssh unitree@192.168.31.49
cd /home/unitree/a2_system_ws
docker compose -f docker-compose.a2.yml down || true
sudo systemctl stop a2-web-console.service || true
source /opt/ros/humble/setup.bash
source /home/unitree/a2_system_ws/install/setup.bash
/home/unitree/a2_system_ws/install/a2_system/share/a2_system/stop_stack.sh || true
pkill -f "bringup.launch.py|a2_sdk_bridge_node|a2_control_bridge_node|manual_localization_publisher|goal_bridge|occupancy_mapper|map_manager_node|map_server|controller_server|planner_server|bt_navigator|velocity_smoother|lifecycle_manager" || true
pgrep -af "bringup.launch.py|a2_sdk_bridge|a2_control_bridge|manual_localization_publisher|goal_bridge|occupancy_mapper|map_manager|map_server|controller_server|planner_server|bt_navigator|velocity_smoother|lifecycle_manager"
```

确认上面最后一条没有输出后，再执行：

```bash
A2_ENABLE_NAV2=true \
A2_MAP_YAML=/home/unitree/a2_system_ws/runtime/maps/test_map_20260423_1059/map.yaml \
/home/unitree/a2_system_ws/install/a2_system/share/a2_system/start_real_stack.sh eth0
```
