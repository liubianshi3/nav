# A2 Web Console

FastAPI + React + ROS2 Humble 的轻量网页控制台，用于在局域网内查看机器人地图、状态、当前位置，并在地图上点选发送 `NavigateToPose` 目标。

## 目录结构

```text
web_console/
├── backend/
│   ├── config.example.yaml
│   ├── main.py
│   ├── models.py
│   ├── requirements.txt
│   ├── ros_bridge.py
│   ├── static/
│   ├── utils.py
│   └── ws.py
├── frontend/
│   ├── index.html
│   ├── package.json
│   ├── src/
│   ├── tsconfig.json
│   └── vite.config.ts
├── scripts/
│   ├── bootstrap_backend.sh
│   ├── build_frontend.sh
│   ├── install_systemd.sh
│   └── run_backend.sh
├── systemd/
│   ├── a2-web-console-build.service
│   └── a2-web-console.service
└── README.md
```

## 功能范围

- 显示 `/map` 对应的投影 2D OccupancyGrid (projected from 3D PCD)
- 显示 `/a2/relocalization/pose` 机器人位置和朝向 (3D NDT relocalization)
- 在地图上点击设置目标点，调用 `/navigate_to_pose`
- 一键取消当前导航任务
- 显示 `/a2/real/report`、`/a2/lidar/status`、`/a2/localization_ok`、`/a2/sdk/status`、`/odom` 等状态
- 默认按当前单雷达已跑通的现状设计，不依赖 `a2_control_bridge`

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- Node.js 18+ 和 npm
- 已经可访问 `a2_system_ws/install/setup.bash`
- 后端进程启动前必须 source:
  - `/opt/ros/humble/setup.bash`
  - `/home/unitree/a2_system_ws/install/setup.bash`

## ROS2 依赖说明

后端直接订阅这些接口：

- `/map`
- `/jt128/front/points`
- `/a2/relocalization/pose`
- `/odom`
- `/tf`
- `/a2/real/report`
- `/a2/lidar/status`
- `/a2/localization_ok`
- `/a2/localization/status`
- `/a2/map_manager/status`
- `/a2/map_manager/active_map`
- `/a2/sdk/status`
- `/a2/raw_state`
- `/camera/image_raw/compressed`
- `/camera/image_raw`
- action: `/navigate_to_pose`

`/mid360/points` 当前只作为后续扩展入口，第一版前端不绘制 3D 点云。
相机优先使用压缩图像 topic，raw 图像会由后端转 JPEG 后再推送到浏览器。

## 配置

配置文件位于 [backend/config.example.yaml](./backend/config.example.yaml)。

可配置项包括：

- 服务监听地址和端口
- 是否允许局域网外访问
- 所有 topic 名
- `NavigateToPose` action 名
- 是否允许发送导航目标
- 健康检查参数
- 相机 topic 和推送频率

建议复制一份为 `backend/config.yaml`，再按机器人实际环境修改：

```bash
cp backend/config.example.yaml backend/config.yaml
```

## 后端启动

首次安装 Python 依赖：

```bash
cd /home/unitree/a2_system_ws/web_console
./scripts/bootstrap_backend.sh
```

手动运行后端：

```bash
cd /home/unitree/a2_system_ws/web_console
./scripts/run_backend.sh
```

如果要覆盖监听地址或端口：

```bash
HOST=0.0.0.0 PORT=8080 ./scripts/run_backend.sh
```

后端接口：

- `GET /api/health`
- `GET /api/snapshot`
- `POST /api/navigation/goal`
- `POST /api/navigation/cancel`
- `GET /`
- `WebSocket /ws`

## 前端启动

开发模式：

```bash
cd /home/unitree/a2_system_ws/web_console/frontend
npm install
npm run dev
```

Vite 默认代理：

- `/api` -> `http://127.0.0.1:8080`
- `/ws` -> `ws://127.0.0.1:8080`

## 生产部署方式

推荐方式是先构建前端，再由 FastAPI 统一托管静态页面和 API。

构建前端：

```bash
cd /home/unitree/a2_system_ws/web_console
./scripts/build_frontend.sh
```

构建产物会写入：

- `/home/unitree/a2_system_ws/web_console/backend/static`

然后直接运行后端即可同时提供：

- 网页静态资源
- REST API
- WebSocket

## systemd 部署

安装 service 文件：

```bash
cd /home/unitree/a2_system_ws/web_console
sudo ./scripts/install_systemd.sh
```

启用后端服务：

```bash
sudo systemctl enable --now a2-web-console.service
```

可选：单独执行一次前端构建 service：

```bash
sudo systemctl start a2-web-console-build.service
```

查看日志：

```bash
sudo journalctl -u a2-web-console.service -f
```

## One-click Standby

If you want the robot to stop old stacks, start the native front-LiDAR source,
restart the web service, and then wait for the operator to click Mapping or
Navigation from the page, use:

```bash
/home/unitree/a2_system_ws/install/a2_system/share/a2_system/start_web_console_suite.sh --iface eth0
```

This script prints failure context for:

- residual ROS bringup processes
- native `unitree_slam.service`
- `a2-web-console.service`
- latest `runtime/logs/bringup_real_*.log`

## 浏览器访问方式

服务启动后，在同局域网的其它电脑浏览器访问：

```text
http://<机器人IP>:8080
```

建议桌面端 Chrome 优先。

## 页面说明

- 左侧：状态面板
  - ready
  - localization
  - lidar
  - SDK
  - active map
  - odom 速度
  - ROS / WebSocket 连接状态
- 中央：2D 地图
  - 显示 OccupancyGrid
  - 显示机器人位置和朝向
  - 支持平移、缩放、点击选点
  - 显示 A2 前向相机图像
- 右侧：任务与控制
  - 当前任务状态
  - 目标点坐标
  - 发送导航按钮
  - 停止导航按钮
  - 最近一次成功和错误提示

## 常见故障排查

可靠性验证清单见 [RELIABILITY_CHECKLIST.md](./RELIABILITY_CHECKLIST.md)。

### 1. 页面能打开，但地图不显示

- 检查 `/map` 是否真的在发布
- 检查 `/api/snapshot` 返回的 `map.loaded` 是否为 `true`
- 检查 `journalctl -u a2-web-console.service -f`

### 2. 页面显示定位丢失，不能发导航

- 检查 `/a2/localization_ok`
- 检查 `/a2/relocalization/pose` (3D NDT)
- 检查 `/a2/localization/status`

### 3. 点击发送导航报“导航服务不可用”

- 检查 Nav2 是否启动
- 检查 `ros2 action list | grep navigate_to_pose`
- 检查 action 名与 `backend/config.yaml` 是否一致

### 4. 页面静态资源 404

- 说明前端还没有 build，先执行：

```bash
./scripts/build_frontend.sh
```

### 5. 后端启动报找不到 `a2_interfaces` 或 `nav2_msgs`

- 说明 ROS 环境没有 source 完整
- 确认已经 source：

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/a2_system_ws/install/setup.bash
```

### 6. Node.js 不存在

- 只影响前端构建
- 后端仍然可以运行
- 但 `backend/static` 没有构建产物时，根路径只会返回提示 JSON

## 后续扩展预留

- 3D 点云查看
- waypoint 列表导航
- 导航历史和任务队列
- 权限控制
- 局域网访问白名单
- 更多健康诊断项
