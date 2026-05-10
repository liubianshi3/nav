# Docker 镜像开发记录

## 目标

将 A2 机器人系统（ROS2 Humble + a2_system_ws）打包成自包含 Docker 镜像，在任何 Linux 机器上可直接运行。

## 旧文件位置（A2 真机）

| 内容 | 路径 |
|------|------|
| 工作区源码 | `/home/unitree/a2_system_ws/src/` |
| 编译产物 | `/home/unitree/a2_system_ws/install/` |
| Unitree SDK | `/opt/unitree_robotics/` |
| 自定义 A2 SDK（含 sport_client） | `/home/unitree/ZJ/unitree_sdk2-main/` |
| 雷达驱动（Hesai） | `/home/unitree/graph_pid_ws/install/hesai_ros_driver/` |
| 建图算法 DLIO 源码 | `/home/unitree/a2_system_ws/src/third_party/direct_lidar_inertial_odometry/` |
| 系统服务 | `sudo systemctl status a2-web-console.service` |
| Web 控制台 | 端口 8080，访问 `http://<A2_IP>:8080` |

## 新仓库位置

- **代码仓库**: `codeup.aliyun.com/.../feishuyz/device-navigation.git`
- **分支**: `impr/rick`
- **本地路径**: `/home/dell/a2_system_ws/`（开发机）
- **A2 路径**: `~/device-navigation/`（通过 git clone）

## 仓库文件结构

```
device-navigation/
├── Dockerfile                    # 自包含 Docker 构建文件
├── .dockerignore                 # 排除未使用的 autoware 包
├── docker/
│   ├── entrypoint.sh             # 容器入口（启动 Web + stack）
│   ├── unitree_sdk/              # Unitree SDK 预编译库（59MB）
│   └── a2_sdk_headers/           # A2 自定义头文件（6个）
├── src/
│   ├── a2_*                      # A2 ROS2 功能包（23个）
│   ├── hesai_ros_driver/         # Hesai 雷达驱动（开源自带）
│   └── third_party/
│       └── direct_lidar_inertial_odometry/  # DLIO 建图算法
└── web_console/                  # Web 前后端
```

## 开发过程

### 阶段一：基础 Docker 化
从 `Dockerfile.real` 改造，新建根目录 `Dockerfile`。

**改动：**
- 基础镜像改用阿里云仓库（`registry.cn-hangzhou.aliyuncs.com/linuxsuren/...`）
- 添加 `ros-humble-autoware-*` 依赖
- 添加 `ros-humble-slam-toolbox`
- 只编译自己的 23 个功能包（跳过 third_party 里的 265 个 autoware 包）

### 阶段二：SDK 打包
Unitree SDK（59MB）的三种处理方式：

1. ~~用 `--build-context` 传入~~ → 别人构建还需要手动指定路径，麻烦
2. ✅ **直接放 git 仓库里** → `git clone` 完就能 `docker build`

**坑：** `.gitignore` 里的 `log/` 规则会屏蔽 SDK 的 `unitree/common/log/` 头文件。改为 `/log/` 只匹配根目录。

### 阶段三：A2 自定义头文件
A2 机器人有专属的 `sport_client.hpp` 等 6 个头文件，不在标准 SDK 里。放在 `docker/a2_sdk_headers/` 中随仓库管理。

### 阶段四：雷达驱动
Hesai JT128 激光雷达需要 `hesai_ros_driver` 驱动。

**踩坑：**
- 尝试 `git clone` → A2 上 SSL 握手失败
- 改为把驱动源码放进仓库 `src/hesai_ros_driver/`（284KB）
- 还需配套的 HesaiLidar_SDK_2.0（2.7MB）

### 阶段五：建图算法（DLIO）
DLIO（Direct LiDAR Inertial Odometry）是建图核心。

**踩坑：**
- 本地和 A2 上的 DLIO 版本不一致（一个 ROS2 版、一个 ROS1 版）
- **解决：** 统一用 ROS2 版（`ament_cmake` + `rclcpp`），本地验证编译通过

### 阶段六：启动方式

**原来：** entrypoint 自动启动 `bringup.launch.py` + Web
**问题：** 与 Web 后端的 `start_script` 冲突（会启动两个 bringup）

**改成：** entrypoint **只启动 Web**，用户打开浏览器点击"开始建图"或"开始导航"，由 stack_controller 统一管理生命周期。

### 阶段七：脚本兼容性
`start_real_stack.sh` 中引用了 `A2_GRAPH_PID_WS`（ROS1 workspace），在 Docker 里不存在。添加了 Docker 检测：

- 检测到 Docker → 跳过网络配置、SDK DDS 设置、sudo 操作
- 非 Docker → 保持原样

## 运行方式

```bash
# A2 真机上
git clone git@codeup.aliyun.com:.../device-navigation.git
cd device-navigation
git checkout impr/rick
docker build . -t robot
docker run --rm -it --privileged --network host robot

# 浏览器打开 http://<A2_IP>:8080
# 在 Web 界面点击"开始建图"或"开始导航"
```

## 网络接口

| 接口 | 用途 | 子网 |
|------|------|------|
| `net1` | 激光雷达 | 192.168.124.x |
| `eth0` | 机器人主板（SDK） | 视网络而定 |

通过环境变量 `A2_NETWORK_INTERFACE` 指定（默认 `net1`）。
