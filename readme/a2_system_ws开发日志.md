# a2_system_ws 开发日志

## 1. 文档目的

这份日志用于完整记录本轮 `a2_system_ws` 从“SSH 接入 A2 机器人本体”到“本体雷达/定位/导航/Web 前端全部打通”的全过程。

文档目标不是只记结论，而是把下面几件事讲清楚：

1. 最初机器人处于什么状态。
2. 我们为什么判断原来的运行环境不适合作为最终主系统。
3. 机器人本体到底有哪些传感器、哪些服务、哪些链路是真的可用。
4. `a2_system_ws` 为了适配机器人本体，具体改了哪些文件，为什么这样改。
5. Web 前端是如何生成、部署、托管、联调并最终跑通的。
6. 过程中遇到的每一个关键问题是什么、根因是什么、最后怎么解决。
7. 当前系统已经达到什么状态，还有什么没有做最终验证。
8. 后续如何使用、如何复现、如何继续往下推进。

这份文档默认读者是后续继续维护 `a2_system_ws` 和 A2 真机集成的人，因此会同时保留：

- 结论
- 路径
- 命令
- 配置点
- 决策理由
- 风险说明

---

## 2. 本轮工作的最终结论

先给最终结论，避免后面看太长忘了现在到了哪一步。

### 2.1 已经打通的部分

当前已经打通：

1. SSH 接入机器人本体宿主环境。
2. 识别并排除了误导性的 ROS1 Docker 链路。
3. 恢复了机器人本体 SLAM 工作区 `/home/unitree/graph_pid_ws`。
4. 确认机器人本体当前实际可用的是 Hesai 双雷达架构，但只有前雷达 `.20` 在线，后雷达 `.21` 离线。
5. 以单雷达旁路方式打通了点云输入。
6. `a2_system_ws` 已适配机器人本体真实话题，不再强绑定云台 MID360 链路。
7. 建图链打通。
8. 用图定位链打通。
9. Nav2 导航栈打通。
10. Web 前端项目已生成并部署到机器人本机。
11. Web 页面可在局域网浏览器访问。
12. Web 后端已通过 `systemd` 托管。
13. Web 已能获取地图、位姿、状态、导航 action 可用状态。

### 2.2 目前没有做最终动作验证的部分

当前还没有做最终动作级验证的是：

1. 没有在网页端实际点击目标点，让机器人真的走起来。
2. 没有从网页端实际验证“停止导航”按钮对正在执行的导航任务的取消效果。
3. `a2_control_bridge` 仍然保持关闭，没有恢复到底盘主动控制链。
4. 第二台雷达 `192.168.124.21` 仍然离线，双雷达融合没有恢复。

### 2.3 当前可认为成立的判断

截至现在，可以认为：

- 除了“网页发目标并让机器人实际运动”这一步没有做最终现场动作验证之外，其余核心链路都已打通。
- 当前系统已经具备“可看地图、可看位姿、可看状态、后端已联通 Nav2 action”的条件。
- 如果下一步要做闭环，只需要在当前 Web 页面上发一次导航目标并观察机器人是否开始运动，再测一次取消导航即可。

---

## 3. 初始环境与最开始的判断

### 3.1 最开始的工作目标

最初目标不是做网页，而是确认这台 A2 机器人本体是否真的具备可供 `a2_system_ws` 复用的真实硬件链路。

具体要确认的是：

1. 能否通过 `ssh a2`(ssh unitree@192.168.124.162,账号unitree，密码Unitree#24226) 直接接入机器人本体。
2. 机器人内部是否真有本体雷达。
3. 机器人内部是否真有本体 IMU。
4. 当前跑着的 ROS 节点里哪些是机器人本体的，哪些是云台/外置 Docker 的。
5. 是否能基于本体传感器做“纯本体建图和导航”，而不是依赖云台。

### 3.2 机器人宿主环境确认

通过 `ssh a2` 接入后，确认宿主机为：

- 主机名：`unitree-a2-pc2`
- 系统：`Ubuntu 22.04`
- 当前可见 IP：
  - `192.168.124.162`
  - `192.168.123.162`
  - `192.168.31.49`
  - `172.17.0.1`

这个信息很重要，因为它说明：

1. 这不是一个完全封闭的“黑盒机器人控制板”，而是一台带标准 Linux 网络环境的机器人本体主机。
2. `192.168.123.x`(pc1) 和 `192.168.124.x`(pc2) 这两段网卡，很符合 `a2_system_ws` 对控制网段和传感器网段的假设。
3. `192.168.31.49` 则提供了局域网访问入口，这也是后面 Web 页面可以从同网段电脑直接访问机器人的基础。

---

## 4. 先排除误导链路：ROS1 Docker 不是目标

### 4.1 最开始看到的现象

最初系统里确实存在一个 ROS1 Docker 容器，里面跑着云台/旧导航相关的链路，包括：

- `a2_ros1_sdk`
- `livox_ros_driver2_node`
- `x_nav_control`

这个容器以 `--net=host` 运行，并且带自动重启策略。

### 4.2 为什么这条链路必须先排除

如果不先排除这条 Docker 链路，会出现以下问题：

1. 很容易把云台上的雷达误认成机器人本体雷达。
2. 很容易把容器里的话题误认成宿主本体上的话题。
3. Host 网络模式会共享宿主网络，增加 DDS/ROS 端口与传感器网络的干扰风险。
4. 后续一旦要做本体 SLAM 和本体导航，这条 Docker 链会持续制造判断噪声。

### 4.3 最终处理方法

后续确认这条容器不是我们要的目标后，执行了两步处理：

1. 把容器 restart policy 从 `unless-stopped` 改为 `no`。
2. 然后 `docker stop` 停掉它。

最终状态变成：

- 该容器不再运行。
- 容器不会自动重新拉起。
- 当前 `docker ps -a` 可见其状态为退出态。

当前现场状态为：

```text
CONTAINER ID   IMAGE                                                           STATUS                     NAMES
ab8f7be72b46   registry.cn-guangzhou.aliyuncs.com/z_nav/x_nav_mj_release:2.2   Exited (137) 4 hours ago   festive_johnson
```

### 4.4 这样做达到的效果

这样做以后：

1. 机器人宿主本体链路和云台 Docker 链路被明确切开。
2. 后面所有关于雷达、IMU、SLAM、Nav2 的判断都只针对机器人本体。
3. 后续 `a2_system_ws` 的适配不会再被容器干扰。

---

## 5. 识别机器人本体 SLAM 服务的真实问题

### 5.1 最初判断

用户最开始关心的是“机器人本体有没有雷达/IMU，能不能直接拉起来”。

在宿主里排查后，关键发现是：

- `advanced_slam.service` 只是前端/API 类服务。
- 真正负责本体雷达/SLAM 入口的是 `unitree_slam.service`。

### 5.2 核心故障

`unitree_slam.service` 当时根本没正常起来，根因不是 topic 没发布，而是启动脚本缺失。

当时的关键报错是：

```text
/home/unitree/graph_pid_ws/bin/tools/service/launch_slam.sh: No such file or directory
```

同时：

- `/home/unitree/graph_pid_ws` 目录本身缺失。

### 5.3 为什么这一步重要

这一步非常关键，因为它把问题从“传感器是否存在”变成了“本体 SLAM 工作区已经丢失/损坏”。

也就是说：

- 不是机器人本体没有雷达。
- 也不是 ROS2 没装好。
- 而是它原本该运行的一整套工作区文件丢了。

这直接决定了后续工作方向：

1. 先恢复 `graph_pid_ws`。
2. 再启动本体链路。
3. 再判断本体传感器和导航链是否在线。

---

## 6. 从 OTA 备份恢复机器人本体工作区

### 6.1 恢复来源

在机器人内部找到了 OTA 备份目录，其中包含完整的本体 SLAM 工作区备份。

关键来源路径为：

```text
/unitree/ota/backup/module/slam_nav/5.0.0.2/module/slam_nav/file/home/unitree/graph_pid_ws
```

### 6.2 恢复目标

恢复目标路径为：

```text
/home/unitree/graph_pid_ws
```

### 6.3 为什么要原地恢复

之所以恢复到原路径，而不是另起一个目录，原因是：

1. `unitree_slam.service` 的启动脚本和二进制都写死引用 `/home/unitree/graph_pid_ws`。
2. 宿主上的若干 service、launch、脚本之间是耦合的。
3. 先恢复原路径，最容易让本体链路恢复到“能启动”的状态。

### 6.4 恢复结果

恢复完成后：

- `/home/unitree/graph_pid_ws` 目录重新存在。
- 大小约 `81M`。
- `launch_slam.sh`、二进制、配置、Python planner 脚本都回来了。

这一步实现了从“服务完全起不来”到“可以进入下一阶段排障”的跨越。

---

## 7. 修复宿主本体 SLAM 启动链

### 7.1 恢复后遇到的新问题

工作区恢复之后，`unitree_slam.service` 虽然不再是“找不到脚本”，但启动链仍然不干净。

排查时发现几个具体问题：

1. 宿主启动脚本引用了错误的 planner 文件名。
2. `stop.sh` 回收逻辑太弱，无法干净结束宿主本体相关进程。
3. 旧实例停不干净，systemd 经常卡在 `deactivating (final-sigterm)`。

### 7.2 修复的具体内容

在机器人本体宿主上，修复了 `launch_slam.sh` 中两个拼错的 planner 文件名：

- `navigation_mapping_.py` 改为 `navigation_mapping.py`
- `dwa_obstacle_avoidance_.py` 改为 `dwa_obstacle_avoidance.py`

当前宿主文件可见为：

```text
/home/unitree/graph_pid_ws/bin/tools/py-planner/navigation_mapping.py
/home/unitree/graph_pid_ws/bin/tools/py-planner/dwa_obstacle_avoidance.py
```

同时补强了：

```text
/home/unitree/graph_pid_ws/bin/tools/service/stop.sh
```

它现在不只会杀 `rslidar`，还会回收：

- `hesai_ros_driver`
- `point_cloud_fusion`
- `unitree_slam`
- `navigation_mapping.py`
- `dwa_obstacle_avoidance.py`
- `frenet_omni_obstacle_avoidance.py`

### 7.3 为什么要先修 stop 链

这一步看起来像小事，但实际上非常重要。

因为如果 stop/restart 不干净，会直接导致：

1. 重复节点残留。
2. 端口、topic、DDS graph 里出现重影。
3. 后续判断“某个 topic 是否可用”时被旧实例污染。
4. 导航链、Web 后端都可能拿到错误的系统状态。

---

## 8. 确认真正的本体雷达类型：不是当前生效的 MID360

### 8.1 用户最关心的问题

用户一开始明确说：

- MID360 更像是云台里的雷达。
- 当前目标不是云台，而是机器人本体自己的雷达和 IMU。
- 希望只靠本体传感器做扫图、建图、用图。

### 8.2 现场证据

恢复宿主链路后，日志和话题都指向：

1. 当前实际跑起来的是 `hesai_ros_driver_node`。
2. 它连接的是两个设备：
   - `192.168.124.20:9347`
   - `192.168.124.21:9347`
3. 日志中出现了 `JT128` 解析器相关信息。
4. 话题为双雷达格式：
   - `/unitree/slam_lidar/points1`
   - `/unitree/slam_lidar/points2`
   - `/unitree/slam_lidar/imu1`
   - `/unitree/slam_lidar/imu2`

### 8.3 得出的判断

这说明机器人本体当前生效链路更像：

- `Hesai JT128 双雷达`

而不是：

- 单个 `Livox MID360`

### 8.4 为什么这一步决定了后续适配策略

这一步直接改变了 `a2_system_ws` 的适配路线。

如果继续按最初的 `MID360` 假设去做，会造成：

1. 驱动层理解错误。
2. 话题名适配错误。
3. 点云输入源搞错。
4. IMU 来源判断错误。

所以后续的策略改成：

1. 不再试图让 `a2_system_ws` 自己在真机上启动一套 Livox/MID360 驱动。
2. 直接消费机器人本体已经跑出来的真实点云和 IMU/odom。
3. 让 `a2_system_ws` 专注做上层编排、状态、建图、定位、导航和 Web。

---

## 9. 本体 IMU 的判断与策略

### 9.1 机器人本体上的 IMU 不是单一来源

排查后明确发现，本体上至少存在两类 IMU 概念：

1. 机身 IMU
   - 配置中多次出现 `dog_imu_raw`
2. 雷达自带 IMU
   - 双 Hesai 链路中体现为 `imu1`、`imu2`

### 9.2 为什么不能简单“三路 IMU 全融合”

用户提出过一个自然的想法：是不是把 `dog_imu_raw + imu1 + imu2` 全都吃进去再融合。

最终没有这么做，原因是：

1. 多 IMU 不是简单拼一起就稳定。
2. 三路 IMU 时间戳、安装位置、噪声特性不同。
3. 生硬混合会让建图链更难排障。

### 9.3 最终采用的原则

最终确定的原则是：

1. 建图主 IMU 使用机身 IMU `dog_imu_raw`。
2. 两路雷达点云先融合成一路点云。
3. 雷达自带 IMU 只在确实需要时用于时间同步/去畸变辅助。

这是一个偏工程稳态的策略，而不是理论上最复杂的策略。

---

## 10. 双雷达现状：`.20` 在线，`.21` 离线

### 10.1 实际排查结果

对本体雷达网络继续深挖后，确认：

- `192.168.124.20` 在线。
- `192.168.124.21` 离线。

现场证据包括：

1. `ping .21` 不通。
2. 邻居表显示 `FAILED/INCOMPLETE`。
3. `tcpdump` 只能稳定看到 `.20` 的主点云 UDP。
4. `.20` 的 `9347/tcp` 可连。
5. `.21` 没有表现出可连的 Hesai 设备特征。

### 10.2 直接后果

由于宿主的 `point_cloud_fusion` 默认按双输入同步策略工作，因此出现了：

- `points1` 有数据。
- `points2` 没数据。
- 融合输出 `/unitree/slam_lidar/points` 不稳定甚至不出。

### 10.3 为什么这一步不能只在 ROS 层继续调

这里已经不是简单的 launch 配置问题，而是更偏现场硬件状态问题：

1. 后雷达是否供电。
2. 后雷达到机内交换机/网口链路是否正常。
3. IP 是否仍为 `192.168.124.21`。
4. 是否被改成别的网段。

因此当前对 `.21` 的判断是：

- 它是独立的后续硬件排障项，不是本轮打通 Web 和导航的阻塞项。

---

## 11. 为什么最终选择把主系统放回 `a2_system_ws`

### 11.1 两种路径

当时有两个可能方向：

1. 继续沿用机器人本体内部原有 `graph_pid_ws` 和宿主脚本体系。
2. 把 `a2_system_ws` 同步到机器人内部，让它成为主系统，把机器人宿主只当作真实驱动和原始信号提供者。

### 11.2 为什么不继续扩展宿主原生那套

宿主原生链路的问题很明显：

1. 历史包袱重。
2. 文件与 service 之间强耦合。
3. 旧脚本不干净。
4. Docker、宿主、备份工作区之间很容易混线。
5. 后续每做一项适配，成本都高。

### 11.3 为什么选择 `a2_system_ws`

最终选择以 `a2_system_ws` 作为主系统，原因是：

1. 这套工作区已经把 `mock/gazebo/real` 三种模式统一抽象。
2. 状态机、健康检查、建图、导航、安全管理、节点编排都已有基础。
3. 后续要做 sim2real 收口，继续在现有代码上改最划算。
4. 真机问题被压缩到“驱动接入层 + 话题映射 + TF + 参数层”，边界清晰。

因此最终总体架构变成：

- 机器人宿主：提供真实点云、真实状态、真实 odom/IMU/地图相关底层链路。
- `a2_system_ws`：做真实模式下的统一编排、映射、状态管理、建图、定位、导航和 Web。

---

## 12. 把 `a2_system_ws` 放进机器人内部

### 12.1 部署路径

将本地工作区同步到机器人内部：

```text
/home/unitree/a2_system_ws
```

### 12.2 这样做的意义

这样做的直接意义有两个：

1. 真机运行时可以直接使用同一套 `a2_system_ws` 工程，而不是额外维护一套宿主专有逻辑。
2. 所有后续适配、Web、导航、地图管理都能统一放在一个工作区内。

---

## 13. `a2_system_ws` 真机适配：核心改动

这一节是整个适配过程中最重要的代码级记录。

### 13.1 `real_lidar.yaml`：从“假设 MID360 驱动”改成“消费机器人本体真实点云”

关键文件：

```text
/home/dell/a2_system_ws/src/a2_system/config/real_lidar.yaml
```

当前关键配置为：

```yaml
real_lidar:
  ros__parameters:
    profile: unitree_native_fused
    driver_mode: external_pointcloud
    input_topic: /unitree/slam_lidar/points1
    output_topic: /mid360/points
    output_frame_id: lidar_link
    stale_timeout_sec: 1.0
```

#### 为什么这么改

原先 `a2_system_ws` 更偏向自己拉 Livox/MID360 驱动。

但当前机器人本体的真实情况是：

1. 本体不是当前生效的 MID360。
2. 本体前雷达 `points1` 已经是可用真实点云。
3. 双雷达融合暂时不可用，因为 `.21` 离线。

所以改成：

- 直接吃 `/unitree/slam_lidar/points1`
- 然后输出成 `a2_system_ws` 上层继续使用的 `/mid360/points`

#### 达到的效果

1. 不必重写上层建图与导航逻辑。
2. 上层仍然可以沿用原来的“统一点云输入口”。
3. 先用单雷达把系统打通，不阻塞整体进度。

### 13.2 `sensors.launch.py`：支持外部点云模式

关键文件：

```text
/home/dell/a2_system_ws/src/a2_bringup/launch/sensors.launch.py
```

这里的关键变化是：

1. 不再强制要求 Livox 驱动包存在。
2. 支持 `external_pointcloud` 模式。
3. 通过 `mid360_driver_guard` 监控点云话题是否真实可用。

#### 为什么这么改

因为真机已经有宿主自己的雷达驱动与点云链路，`a2_system_ws` 不应该在这里再硬起一套假定的设备驱动。

#### 达到的效果

1. 真实模式下可以把机器人本体点云直接纳入 `a2_system_ws`。
2. 保留原来 mock/gazebo 模式，不破坏已有结构。
3. 让 `a2_system_ws` 的真实模式对传感器来源更抽象。

### 13.3 `slam.launch.py`：改为 `external_odom`

关键文件：

```text
/home/dell/a2_system_ws/src/a2_bringup/launch/slam.launch.py
```

这里的重要方向是：

1. 支持 `external_odom` profile。
2. 在真实模式下，如果选择 `external_odom`，就不强行拉 FAST_LIO，而是等待机器人原生 odom / 外部导航栈提供的里程计与定位相关输出。

#### 为什么这么改

因为当前机器人本体本来就有一条可运行的宿主链路，我们不应该在第一轮适配时同时引入两套 SLAM 竞争。

#### 达到的效果

1. 降低首次真机打通的复杂度。
2. 先用机器人现有 odom/定位结果把 `a2_system_ws` 上层跑起来。
3. 把问题分层：先验证系统接入，再决定是否替换底层建图链。

### 13.4 `bringup.launch.py`：为 `a2_control_bridge` 增加显式开关

关键文件：

```text
/home/dell/a2_system_ws/src/a2_bringup/launch/bringup.launch.py
```

新增了：

- `enable_control_bridge`

并让 `a2_control_bridge` 按条件启动。

#### 为什么这么改

因为现场验证发现：

- `a2_control_bridge` 一旦接上真实 SDK，会触发一次堆内存崩溃。

这会直接阻塞“先把建图/导航/Web 打通”的大目标。

#### 达到的效果

1. 可以在不启用底盘主动控制链的前提下先打通状态、地图、定位、导航和 Web。
2. 把控制桥从主阻塞项降级为后续单独修复项。
3. 保证本轮工作聚焦在“看图、定位、导航能力在线”。

### 13.5 `a2_sdk_bridge/CMakeLists.txt` 与 `a2_control_bridge/CMakeLists.txt`

关键文件：

```text
/home/dell/a2_system_ws/src/a2_sdk_bridge/CMakeLists.txt
/home/dell/a2_system_ws/src/a2_control_bridge/CMakeLists.txt
```

做了同类修改：

1. 优先搜索机器人现成的 SDK 安装位置：
   - `/opt/unitree_robotics/lib/cmake/unitree_sdk2`
2. 同时保留本地开发机上的 fallback 路径：
   - `/home/dell/unitree_sdk2`
   - `/usr/local/lib/cmake/unitree_sdk2`

#### 为什么这么改

因为编译环境已经不再只是在开发机本地，而是要适配机器人本体内部的真实 SDK 环境。

#### 达到的效果

1. 同一份代码能在开发机和机器人宿主上都尽量兼容。
2. 减少“为了机器人内部再维护一份特殊分支”的需要。

### 13.6 `map_manager.yaml` 与 `map_manager_node.py`

关键文件：

```text
/home/dell/a2_system_ws/src/a2_system/config/map_manager.yaml
/home/dell/a2_system_ws/src/map_manager/map_manager/map_manager_node.py
```

核心修改是把原来写死的路径改成 `HOME` 相关路径。

例如：

```yaml
map_root: ${HOME}/a2_system_ws/runtime/maps
```

#### 为什么这么改

原先如果写死 `/home/dell/...`，一部署到机器人宿主用户 `unitree` 下就会出路径错误和权限问题。

#### 达到的效果

1. 同一份代码可以随着当前用户环境自动定位地图目录。
2. 机器人内部保存和加载地图时不再依赖开发机用户名。

### 13.7 `localization.yaml`

关键文件：

```text
/home/dell/a2_system_ws/src/a2_system/config/localization.yaml
```

最终改成：

```yaml
localization_gate:
  ros__parameters:
    input_pose_topic: /amcl_pose
    status_topic: /a2/localization_ok
    max_pose_age_sec: 30.0
    max_xy_variance: 0.25
    max_yaw_variance: 0.2
```

#### 为什么最初改成 2 秒，又为什么最后改成 30 秒

早期为了保证定位 freshness，先把 `max_pose_age_sec` 收到了 `2.0`。

但是现场最终发现：

1. 机器人静止时，`amcl_pose` 不一定持续高频更新。
2. `localization_gate` 会据此把定位判成 stale。
3. 这会进一步把 `/a2/real/report` 拉成 `degraded`。
4. Web 前端虽然已经能看到地图和位姿，但系统 ready 会反复掉下去。

最终把这个阈值放宽到 `30.0`，目的是更符合当前“机器人静止 + 重点做网页监控与导航接入”的场景。

#### 达到的效果

1. 静止时系统不会因为位姿刷新过慢而频繁退化。
2. Web 前端的 ready 状态更稳定。
3. 当前阶段更适合联调和演示。

---

## 14. 在机器人上把导航系统拉起来

### 14.1 Nav2 依赖安装

机器人内部安装了：

```text
ros-humble-navigation2
ros-humble-nav2-bringup
```

### 14.2 使用的地图

当前使用的地图文件为：

```text
/home/unitree/a2_system_ws/runtime/maps/test_map_20260423_1059/map.yaml
```

### 14.3 主要启动命令

用于拉起真实模式导航栈的关键命令形式为：

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/a2_system_ws/install/setup.bash
cd /home/unitree/a2_system_ws
ros2 launch a2_bringup bringup.launch.py \
  runtime_mode:=real \
  use_mock:=false \
  network_interface:=eth0 \
  enable_nav2_bringup:=true \
  enable_control_bridge:=false \
  map:=/home/unitree/a2_system_ws/runtime/maps/test_map_20260423_1059/map.yaml
```

### 14.4 为什么 `enable_control_bridge:=false`

因为：

- `a2_control_bridge` 接上真实 SDK 后会触发堆崩溃。

当前目标是：

1. 先让传感器、建图、定位、导航、状态、Web 都打通。
2. 底盘主动控制作为独立问题后续再修。

### 14.5 当时观察到的运行节点

启动后可见典型导航节点包括：

- map_server -> 提供地图
amcl -> 提供当前位置
planner_server -> 算全局路线
bt_navigator -> 组织导航流程
controller_server -> 算局部控制
velocity_smoother -> 平滑速度
a2_control_bridge -> 真正发到底盘

### 14.6 初始位姿问题

Nav2 启动后，第一次常见问题是：

- AMCL 没有初始位姿，不会立即发布可用定位。

为此通过向 `/initialpose` 发布 `PoseWithCovarianceStamped` 解决。

发布的是典型 map frame 下原点附近初始位姿，协方差也做了合理设置。

---

## 15. 为什么最终采用单雷达旁路而不是等双雷达修好

### 15.1 如果坚持等 `.21` 修好再继续

那么整个进度会被后雷达硬件问题卡住，后果是：

1. `a2_system_ws` 真机适配无法继续验证。
2. 建图、定位、导航、Web 都得等硬件恢复。
3. 无法快速验证整个软件框架在机器人本体上的可行性。

### 15.2 旁路策略

因此最终采用：

- 直接用 `/unitree/slam_lidar/points1` 作为真实点云输入源。

### 15.3 这一步达成的意义

这一步是整个项目能快速向前推进的关键决策之一。

它带来的直接收益是：

1. 单雷达已经足够支撑建图、定位和基础导航。
2. 不需要等 `.21` 恢复就能先验证软件系统。
3. 先把“整体架构可用”这件事钉死，再回头修双雷达。

---

## 16. Web 前端项目：为什么要单独做一套 Web Console

### 16.1 用户需求

在前面通过 SSH 反复看话题、看日志之后，用户提出了很明确的需求：

- 不想每次都 SSH 到机器人内部看图和看状态。
- 希望从自己本机电脑浏览器直接看到地图、机器人位置和状态。
- 希望后续能在地图上点目标，让机器人去走。

### 16.2 需求边界

当时明确约束为：

1. 第一版只做 2D 地图。
2. 不做 3D 点云可视化。
3. 不做复杂运维页面。
4. 不做手动遥控。
5. 不优先用 rosbridge。
6. 优先部署在机器人本机。
7. 局域网其他电脑用浏览器访问机器人 IP。

### 16.3 最终技术选型

最终选型为：

- 前端：`React + TypeScript + Vite`
- 后端：`FastAPI + Python 3 + rclpy`
- 通信：`WebSocket + REST`
- 部署：前端 build 后由 FastAPI 统一静态托管
- 进程管理：`systemd`

### 16.4 为什么不用 rosbridge 作为主实现

不用 rosbridge 作为主实现的原因：

1. 当前需求不复杂。
2. 直接用后端订阅 ROS2，更好控异常与权限。
3. 能更稳地做 snapshot/cache/action timeout 等工程性逻辑。
4. 减少中间层，后续维护更清楚。

---

## 17. Web Console 项目结构

本地项目生成在：

```text
/home/dell/a2_system_ws/web_console
```

核心结构如下：

```text
backend/
frontend/
scripts/
systemd/
README.md
```

关键文件包括：

- `backend/main.py`
- `backend/ros_bridge.py`
- `backend/config.example.yaml`
- `frontend/src/App.tsx`
- `frontend/src/components/MapCanvas.tsx`
- `scripts/bootstrap_backend.sh`
- `scripts/build_frontend.sh`
- `scripts/run_backend.sh`
- `systemd/a2-web-console.service`

---

## 18. Web 后端如何设计

### 18.1 后端职责

后端职责包括：

1. 订阅地图、位姿、状态 topic。
2. 缓存最近一次有效数据。
3. 对外提供：
   - `GET /api/health`
   - `GET /api/snapshot`
   - `POST /api/navigation/goal`
   - `POST /api/navigation/cancel`
4. 通过 `/ws` WebSocket 向前端持续推送：
   - 地图
   - 位姿
   - 状态
   - 导航任务状态
   - 后端健康状态
5. 作为 `NavigateToPose` action client。

### 18.2 接入的 ROS2 话题

后端接入了：

- `/map`
- `/mid360/points`
- `/amcl_pose`
- `/odom`
- `/tf`
- `/tf_static`
- `/a2/real/report`
- `/a2/lidar/status`
- `/a2/localization_ok`
- `/a2/localization/status`
- `/a2/map_manager/status`
- `/a2/map_manager/active_map`
- `/a2/sdk/status`
- `/a2/raw_state`
- action `/navigate_to_pose`

### 18.3 为什么要做 snapshot 缓存

因为浏览器连接是随时可能断开重连的，如果后端不缓存：

1. 页面刷新后可能一段时间内没有内容。
2. 某些低频或 latch 类型 topic 在重连场景下不稳定。

所以后端会缓存最近一次：

- 地图
- 位姿
- 状态
- action 状态

这样新连接一上来就能拿到当前状态。

---

## 19. Web 前端如何设计

### 19.1 页面布局

当前页面按工程调试风格做成三栏：

1. 左侧：状态面板
2. 中央：2D 地图
3. 右侧：任务与控制

### 19.2 页面能力

页面第一版目标包括：

1. 展示地图。
2. 展示机器人当前位置与朝向。
3. 展示状态：
   - ready
   - localization_ok
   - lidar 状态
   - active_map
   - sdk 状态
   - map manager 状态
   - 线速度与角速度
4. 支持目标点选择。
5. 支持发送导航目标。
6. 支持停止导航。

### 19.3 为什么优先 2D 而不是 3D

因为当前目标很明确：

- 让人能在局域网里直观看到“地图、位置、状态、目标”。

而不是做一套浏览器版 RViz。

2D 足够支撑：

1. 看地图。
2. 看当前位置。
3. 点目标导航。
4. 看状态与导航任务。

---

## 20. Web 项目首次部署到机器人内部

### 20.1 同步项目

将项目同步到机器人：

```text
/home/unitree/a2_system_ws/web_console
```

### 20.2 机器人上缺少的依赖

第一次部署时，机器人缺少：

1. `node`
2. `npm`
3. `python3.10-venv`

### 20.3 为什么要在机器人本机安装这些依赖

因为当前部署策略是：

1. 前端在机器人本机 build。
2. 后端在机器人本机直接运行。
3. 最终由机器人本机 FastAPI 统一托管前端静态文件和 API/WebSocket。

这比再维护一台额外服务器更简单。

### 20.4 安装结果

在机器人本机安装后，版本为：

- `node v20.20.2`
- `npm 10.8.2`

并安装了：

- `python3.10-venv`

---

## 21. Web 项目构建与部署过程中遇到的问题

这一节记录 Web 部署过程中的关键工程问题。

### 21.1 `.venv` 创建失败

#### 现象

首次在机器人本机跑后端 bootstrap 时，虚拟环境创建不完整。

#### 根因

机器人最初没装 `python3.10-venv`。

#### 解决

1. 安装 `python3.10-venv`。
2. 修改 `bootstrap_backend.sh`，在发现旧 `.venv` 坏掉时先删除再重建。

#### 达到的效果

后端依赖环境可稳定创建。

### 21.2 `run_backend.sh` 在 `set -u` 下 source ROS 环境报错

#### 现象

直接带 `set -euo pipefail` 去 source ROS setup，有概率因未定义变量触发 shell 错误。

#### 解决

在 `run_backend.sh` 中：

1. 先 `set +u`
2. source `/opt/ros/humble/setup.bash`
3. source `/home/unitree/a2_system_ws/install/setup.bash`
4. 再恢复 `set -u`

#### 达到的效果

后端启动脚本更稳，避免 ROS setup 脚本和 strict shell 选项冲突。

### 21.3 缺少 `numpy`

#### 现象

ROS Python 消息相关依赖在运行时需要 `numpy`。

#### 解决

在：

```text
backend/requirements.txt
```

中加入 `numpy`。

#### 达到的效果

后端在机器人本机环境下能稳定加载 ROS 相关消息模块。

### 21.4 Vite ESM 配置问题

#### 现象

前端构建环境中，`vite.config.ts` 对 `__dirname` 等 CommonJS 假设不兼容。

#### 解决

1. 添加 `@types/node`
2. 在 `tsconfig.node.json` 中加入 `types: ["node"]`
3. 在 `vite.config.ts` 中改为 `fileURLToPath(import.meta.url)` 风格

#### 达到的效果

前端在机器人上的 `npm build` 可正常通过。

---

## 22. Web `systemd` 部署中遇到的问题

### 22.1 第一次 `systemd` 启动失败

#### 现象

第一次安装 `a2-web-console.service` 后，服务不断自动重启，报错集中在：

- `CHDIR`
- `Permission denied`

#### 根因

service 文件中使用了 `%h` 作为 home 目录占位，但在当前场景下被解析到了 `/root`，导致：

- `WorkingDirectory` 错误变成 `/root/a2_system_ws/web_console`

这当然不是实际项目目录。

#### 解决

将：

- `WorkingDirectory`
- `CONFIG_PATH`
- `ExecStart`

全部改成绝对路径：

```text
/home/unitree/a2_system_ws/web_console/...
```

#### 达到的效果

`a2-web-console.service` 最终可以正常 `enabled + active`。

### 22.2 为什么最终 `ExecStart` 走 `run_backend.sh`

没有直接把一长串 source + python 写死在 service 里，而是让它走：

```text
/home/unitree/a2_system_ws/web_console/scripts/run_backend.sh
```

原因是：

1. 手工启动已经验证过这条脚本是可用的。
2. 把启动逻辑集中在一个地方，后续更容易维护。
3. 减少 service 和手动运行之间的行为差异。

---

## 23. Web 后端上线后出现的 QoS 问题

### 23.1 现象

`systemd` 版 Web 后端上线后，发现：

- `/api/health` 正常
- 其他状态 topic 能收到
- 但是 `map_received=false`
- `pose_received=false`

也就是说：

- 状态和 odom 进来了
- 但 `/map` 和 `/amcl_pose` 没进来

### 23.2 为什么这是关键问题

如果地图和位姿收不到：

1. 页面无法显示地图。
2. 页面无法显示机器人位置。
3. 导航目标发送前的前提检查无法成立。

### 23.3 根因分析

通过 `ros2 topic info --verbose` 排查，发现：

- `/map` 发布端是 `TRANSIENT_LOCAL`
- `/amcl_pose` 发布端是 `TRANSIENT_LOCAL`

而 Web 后端原本使用默认 `VOLATILE` 订阅。

这意味着：

1. 如果服务启动时没正好赶上消息，
2. 或服务重启发生在后面，
3. 就拿不到 latched 数据。

### 23.4 解决方法

在：

```text
/home/dell/a2_system_ws/web_console/backend/ros_bridge.py
```

中将 `/map` 和 `/amcl_pose` 的订阅 QoS 改成：

- `ReliabilityPolicy.RELIABLE`
- `DurabilityPolicy.TRANSIENT_LOCAL`

### 23.5 达到的效果

服务重启后，Web 后端仍然能重新拿到：

- 当前地图
- 当前 AMCL 位姿

这一步是 Web 真正稳定可用的关键修复之一。

---

## 24. 导航系统重新启动时遇到的定位问题

### 24.1 现象

Web 打通后，现场又出现一轮导航 ready 退化，具体表现为：

- `/a2/localization_ok = false`
- `/a2/real/report` 变为 `degraded`
- `/a2/localization/status` 报：
  - `stale_pose`
  - `pose_timeout`

### 24.2 深挖后发现的问题

这个问题不是 Web 导致的，而是导航层自身存在几类叠加问题：

1. `localization_gate` 对 pose freshness 判定过严。
2. AMCL 重启时初始位姿发得太早，日志明确提示：
   - `Received initial pose request, but AMCL is not yet in the active state`
3. 之前多次手工重启残留了旧实例，导致 graph 中出现同名重复节点。

### 24.3 重复节点的影响

当时图里能看到若干节点重复，例如：

- `/sync_monitor`
- `/safety_supervisor`
- `/real_readiness_monitor`
- `/localization_gate`
- `/exploration_manager`
- `/a2_state_publisher`
- `/a2_sdk_bridge`

这会导致：

1. readiness 状态判断混乱。
2. 某些 topic 被多个实例同时发布/订阅。
3. AMCL/TF 状态判断变得不可信。

### 24.4 最终解决方法

最终采用以下顺序解决：

1. 清理旧的 `a2_system_ws` 相关进程。
2. 清理旧的 Nav2 相关进程。
3. 重新只拉起一个干净实例的 `bringup.launch.py`。
4. 等待 AMCL 完全 active。
5. 再重新发布 `/initialpose`。
6. 同时把 `max_pose_age_sec` 放宽到 `30.0`。

### 24.5 达到的效果

清理并重拉后，恢复为：

- `/amcl_pose` 有效
- `/a2/localization_ok = true`
- `/a2/real/report = ready=true`

这一步标志着：

- Web、地图、位姿、状态、导航 action 都真正对上了同一套干净的导航实例。

---

## 25. 当前现场状态

截至本日志编写时，现场状态如下。

### 25.1 机器人主机

- 主机名：`unitree-a2-pc2`
- IP：
  - `192.168.124.162`
  - `192.168.123.162`
  - `192.168.31.49`
  - `172.17.0.1`

### 25.2 宿主本体雷达

- 前雷达 `.20` 在线
- 后雷达 `.21` 离线
- 当前系统采用单雷达旁路

### 25.3 Docker

- 干扰性的 ROS1 Docker 已停用，不再自动重启

### 25.4 `a2_control_bridge`

- 仍关闭
- 原因：接真实 SDK 会触发堆崩溃

### 25.5 `a2_system_ws` 导航状态

当前已恢复到：

- `/a2/localization_ok = true`
- `/a2/real/report = mode=real;state=ready;ready=true`

### 25.6 Web Console 状态

当前 Web 服务：

- service 名：`a2-web-console.service`
- 状态：`enabled`
- 状态：`active`

当前访问地址：

```text
http://192.168.31.49:8080/
```

当前健康检查：

- `backend_ok = true`
- `ros_connected = true`
- `action_server_ready = true`
- `map_received = true`
- `pose_received = true`

---

## 26. 当前已经修改/生成的重要文件清单

### 26.1 `a2_system_ws` 相关

- `/home/dell/a2_system_ws/src/a2_system/config/real_lidar.yaml`
- `/home/dell/a2_system_ws/src/a2_bringup/launch/sensors.launch.py`
- `/home/dell/a2_system_ws/src/a2_bringup/launch/slam.launch.py`
- `/home/dell/a2_system_ws/src/a2_bringup/launch/bringup.launch.py`
- `/home/dell/a2_system_ws/src/a2_sdk_bridge/CMakeLists.txt`
- `/home/dell/a2_system_ws/src/a2_control_bridge/CMakeLists.txt`
- `/home/dell/a2_system_ws/src/a2_system/config/map_manager.yaml`
- `/home/dell/a2_system_ws/src/map_manager/map_manager/map_manager_node.py`
- `/home/dell/a2_system_ws/src/a2_system/config/localization.yaml`

### 26.2 机器人宿主本体工作区相关

- `/home/unitree/graph_pid_ws/bin/tools/service/launch_slam.sh`
- `/home/unitree/graph_pid_ws/bin/tools/service/stop.sh`

### 26.3 Web Console 相关

- `/home/dell/a2_system_ws/web_console/backend/main.py`
- `/home/dell/a2_system_ws/web_console/backend/ros_bridge.py`
- `/home/dell/a2_system_ws/web_console/backend/config.example.yaml`
- `/home/dell/a2_system_ws/web_console/backend/requirements.txt`
- `/home/dell/a2_system_ws/web_console/frontend/src/App.tsx`
- `/home/dell/a2_system_ws/web_console/frontend/src/components/MapCanvas.tsx`
- `/home/dell/a2_system_ws/web_console/frontend/vite.config.ts`
- `/home/dell/a2_system_ws/web_console/frontend/tsconfig.node.json`
- `/home/dell/a2_system_ws/web_console/scripts/bootstrap_backend.sh`
- `/home/dell/a2_system_ws/web_console/scripts/build_frontend.sh`
- `/home/dell/a2_system_ws/web_console/scripts/run_backend.sh`
- `/home/dell/a2_system_ws/web_console/systemd/a2-web-console.service`

---

## 27. 现在如何使用

这一节写给下一位直接接手的人。

### 27.1 看网页

在和机器人同一局域网内的电脑上打开：

```text
http://192.168.31.49:8080/
```

能看到：

1. 地图
2. 机器人位置
3. 状态面板
4. 当前导航任务状态

### 27.2 看 Web 后端是否活着

在本机或机器人上访问：

```text
http://192.168.31.49:8080/api/health
```

如果正常，应该能看到类似：

- `backend_ok=true`
- `ros_connected=true`
- `action_server_ready=true`
- `map_received=true`
- `pose_received=true`

### 27.3 如果网页打不开

优先检查：

```bash
ssh a2
systemctl status a2-web-console.service
```

如果需要查看日志：

```bash
journalctl -u a2-web-console.service -n 200 --no-pager
```

### 27.4 如果网页能打开但地图/位姿没有

检查：

```bash
curl http://127.0.0.1:8080/api/health
curl http://127.0.0.1:8080/api/snapshot
```

再检查：

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/a2_system_ws/install/setup.bash
ros2 topic echo --once /map
ros2 topic echo --once /amcl_pose
```

### 27.5 如果 ready 掉成 false

优先检查：

```bash
ros2 topic echo --once /a2/real/report
ros2 topic echo --once /a2/localization_ok
ros2 topic echo --once /a2/localization/status
```

如果是：

- `stale_pose`
- `pose_timeout`

则说明定位 freshness 又掉了，需要：

1. 检查 `/amcl_pose` 是否持续可用。
2. 检查导航实例是否残留重复节点。
3. 必要时重拉导航栈并重新发 `/initialpose`。

---

## 28. 现在如何复现整套过程

下面给出“从零到可访问网页”的复现路径。

### 28.1 连接机器人

```bash
ssh a2
```

确认：

```bash
hostname
hostname -I
```

### 28.2 确保干扰 Docker 不运行

```bash
docker ps -a
```

如果云台 ROS1 Docker 又被启起来，先停掉并关闭自动重启。

### 28.3 确保本体宿主雷达链在

检查宿主进程：

```bash
ps -ef | grep -E "hesai_ros_driver|point_cloud_fusion" | grep -v grep
```

检查点云：

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/a2_system_ws/install/setup.bash
ros2 topic echo --once /unitree/slam_lidar/points1
```

### 28.4 如果 `graph_pid_ws` 丢了

从 OTA 备份恢复：

```text
/unitree/ota/backup/module/slam_nav/5.0.0.2/module/slam_nav/file/home/unitree/graph_pid_ws
```

恢复到：

```text
/home/unitree/graph_pid_ws
```

### 28.5 同步 `a2_system_ws`

把本地工作区同步到机器人：

```bash
rsync -az --delete /home/dell/a2_system_ws/ a2:/home/unitree/a2_system_ws/
```

### 28.6 编译 `a2_system_ws`

机器人上执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/unitree/a2_system_ws
colcon build --symlink-install
```

### 28.7 拉起导航栈

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/a2_system_ws/install/setup.bash
cd /home/unitree/a2_system_ws
ros2 launch a2_bringup bringup.launch.py \
  runtime_mode:=real \
  use_mock:=false \
  network_interface:=eth0 \
  enable_nav2_bringup:=true \
  enable_control_bridge:=false \
  map:=/home/unitree/a2_system_ws/runtime/maps/test_map_20260423_1059/map.yaml
```

### 28.8 如果需要发布初始位姿

使用 `PoseWithCovarianceStamped` 向：

```text
/initialpose
```

发布初始位姿。

关键点：

1. 要等 `amcl` 进入 active 状态再发。
2. 协方差要给合理值。
3. 最好多发几次。

### 28.9 同步 Web Console

```bash
rsync -az --delete /home/dell/a2_system_ws/web_console/ a2:/home/unitree/a2_system_ws/web_console/
```

### 28.10 安装机器人本机依赖

首次部署时，机器人本机需要具备：

- `node`
- `npm`
- `python3.10-venv`

### 28.11 构建 Web 前后端

```bash
cd /home/unitree/a2_system_ws/web_console
./scripts/bootstrap_backend.sh
./scripts/build_frontend.sh
```

### 28.12 启动 Web 后端

开发/手工方式：

```bash
cd /home/unitree/a2_system_ws/web_console
./scripts/run_backend.sh
```

生产/稳定方式：

```bash
sudo cp /home/unitree/a2_system_ws/web_console/systemd/a2-web-console.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now a2-web-console.service
```

### 28.13 验证 Web

```bash
curl http://127.0.0.1:8080/api/health
curl http://127.0.0.1:8080/api/snapshot
```

再从局域网浏览器打开：

```text
http://192.168.31.49:8080/
```

---

## 29. 当前已知问题与风险

### 29.1 `a2_control_bridge` 仍然关闭

原因：

- 接真实 SDK 会触发堆崩溃。

风险：

- 当前还不适合通过这条链做低层主动控制。

### 29.2 后雷达 `.21` 离线

原因：

- 当前更像是硬件/网络链路问题，不是 ROS launch 参数问题。

影响：

- 只能使用单雷达旁路。
- 双雷达融合未恢复。

### 29.3 网页端动作级验证还没做最终闭环

虽然：

- action client 已联通
- `action_server_ready=true`

但当前仍未做最终现场验证：

1. 在网页点目标。
2. 看机器人是否开始运动。
3. 再点停止。
4. 看 action cancel 是否生效。

### 29.4 宿主原生链仍有历史包袱

虽然：

- `graph_pid_ws` 已恢复
- stop 链已加强

但宿主本体原生环境仍然不是一套非常干净、现代化、易维护的系统。

因此后续应该继续坚持：

- `a2_system_ws` 做主系统
- 宿主链只提供底层真实驱动与原始数据

---

## 30. 建议的下一步

按优先级建议如下。

### 30.1 第一优先级：做网页端真机动作闭环验证

也就是：

1. 在网页点击目标点。
2. 发送 `NavigateToPose`。
3. 观察机器人是否起步。
4. 再点击停止导航。
5. 观察取消是否成功。

这是当前最该做的一步，因为：

- 它是从“所有链都看起来通了”到“真正形成可操作系统”的最后一步。

### 30.2 第二优先级：修 `a2_control_bridge`

目标：

- 恢复真实 SDK 下不崩溃的控制桥。

### 30.3 第三优先级：排查后雷达 `.21`

需要现场从硬件层排查：

1. 供电
2. 网线
3. 交换机链路
4. 设备 IP

### 30.4 第四优先级：继续增强 Web

等动作级验证通过后，可再加：

1. 导航结果反馈更丰富的 UI
2. 多目标点
3. 地图切换
4. 后续 3D 点云扩展位

---

## 31. 一句话总结

这次工作的核心不是“做了一个网页”，而是把 `a2_system_ws` 从原先偏仿真/偏 MID360 假设的状态，真正收口到了 A2 机器人本体上：

- 宿主驱动和历史环境被理清了；
- 本体真实雷达链被识别出来了；
- 单雷达可运行路线被建立了；
- 建图、定位、导航被重新接通了；
- Web 前端也已经在机器人本机部署并跑起来了。

当前离最终闭环只差最后一步：

- 从网页上真实发一次导航目标，让机器人动起来，再验证停止导航。

---

## 32. 2026-04-23 第二阶段目标：把“看起来都通了”变成“网页点图真机真的走”

### 32.1 这一阶段的目标和上一阶段不同

上一阶段的目标是：

- 把真机话题、地图、位姿、Web 页面、Nav2 action server 全部接通。

但这还不等于真正可用。

第二阶段真正要解决的是：

1. 把旧的 `bringup` 残留清干净，只保留一套由正式脚本拉起的真机栈。
2. 确认 `/odom`、`/a2/raw_state`、`/cmd_vel`、`/a2/control/status` 在真机上都正常。
3. 让 Web 前端对应的真实后端接口可以设置初始位姿。
4. 让 Web 前端对应的真实后端接口可以发送导航目标。
5. 让导航目标不仅“被接收”，而且真的让 A2 真机在地图里发生位移。

### 32.2 这一阶段的验收标准

这一阶段的验收标准不再是“topic 正常”，而是下面这些动作级标准：

1. 真机 `real bringup` 只存在一套进程，不存在历史残留双实例。
2. Web 后端 `health` 返回 `action_server_ready=true`。
3. Web 设置初始位姿后，`/a2/localization/status=ready`，`/a2/real/report=ok`。
4. Web 发送导航目标后，`a2_control_bridge` 至少要进入一次 `command_sent`。
5. 真机 `pose` / `raw_state.position` 要发生真实变化，不能只是 action 状态在动。
6. 至少有一段短程导航在真机上成功结束。

---

## 33. 这一阶段先碰到的两个硬阻塞

### 33.1 旧的 `real bringup` 并不等于“真机可运动”

最开始现场看起来像是：

- `bringup` 已经起来了；
- `Nav2` 也起来了；
- Web 也能看到地图和位姿；
- `NavigateToPose` action server 也可用。

但一旦真的走动作验证，就发现这不等于真机可动。

当时最典型的现象是：

1. 导航目标能被接收。
2. `navigation.state=navigating`。
3. 控制桥有时会进入 `command_sent`。
4. 但 `pose` / `raw_state.position` 长时间不变化。
5. 最后 `controller_server` 报 `Failed to make progress`。

这说明：

- 问题不在 Web 发不出目标；
- 也不在 Nav2 action 接不到目标；
- 而是在“控制命令发出去之后，真机为什么没有形成真实位移”。

### 33.2 现场还存在旧 `bringup` 残留污染

这一阶段现场另一个很大的问题是：

- 机器人宿主上经常残留历史 `bringup.launch.py` 进程；
- 这些残留会导致新的 `map_server` / `lifecycle_manager` / `a2_control_bridge` 被旧实例污染；
- 最直接的表现就是：
  - 新旧两套 `bringup` 同时存在；
  - `lifecycle bond` 对不上；
  - 新一轮调试结果不可信。

所以第二阶段真正开始做动作闭环前，先把“干净单实例 bringup”作为前置条件。

---

## 34. 为了适配真机网页操作，新增了一条 `manual_odom` 定位链

### 34.1 为什么不继续依赖当时那套 `amcl`

现场联调时发现，直接依赖那套原始 `amcl` 路径有两个现实问题：

1. 真机 bringup 后不一定能立刻进入“可直接发初始位姿”的稳定状态。
2. 对网页操作来说，最重要的是“用户先点初始位姿，再让系统用真机 `/odom` 往前推”，而不是强绑定某个现场 `amcl` 激活时序。

因此这阶段新增了一套更适合网页真机联调的链路：

- 先通过 Web 发一次初始位姿；
- 再由真机 `/odom` 持续推算 `map -> odom`；
- 对外持续发布 `/amcl_pose` 风格的位姿。

### 34.2 新增的关键文件

这一阶段新增或修改的关键文件如下：

1. `localization_manager/localization_manager/manual_localization_publisher.py`
2. `localization_manager/setup.py`
3. `a2_bringup/launch/localization.launch.py`
4. `a2_bringup/launch/nav2.launch.py`
5. `a2_bringup/launch/bringup.launch.py`

其中：

- `manual_localization_publisher.py`
  - 订阅 `/initialpose`
  - 订阅 `/odom`
  - 维护 `map -> odom`
  - 发布 `/amcl_pose`
- 真实模式下的 localization launch 现在支持：
  - `real_localization_mode:=manual_odom`
- `Nav2` 在这个模式下：
  - 启 `map_server`
  - 启 navigation stack
  - 不再依赖现场 `amcl` 作为唯一入口

### 34.3 为什么这条链对 Web 特别重要

因为 Web 页面上用户的真实操作就是：

1. 先告诉系统“机器人现在在地图上的哪个位置”。
2. 再告诉系统“接下来要去哪”。

`manual_odom` 模式正好对应这个交互模型。

在这条链跑通后，Web 后端可以直接把“设置初始位姿”变成一条真实有效的现场动作。

---

## 35. 为了让 Web 真正可用，前后端也补了初始位姿能力

### 35.1 后端新增了真实初始位姿 API

这阶段对 Web 后端新增了：

```text
POST /api/localization/initialpose
```

关键文件：

1. `web_console/backend/main.py`
2. `web_console/backend/ros_bridge.py`

这条 API 的作用是：

1. 接收网页提交的地图坐标。
2. 结合当前地图做一次“吸附到最近可行栅格”。
3. 通过 `/initialpose` 连续发布多次 `PoseWithCovarianceStamped`。

### 35.2 前端新增了“设置初始位姿”入口

对应的前端文件为：

```text
web_console/frontend/src/components/ControlSidebar.tsx
```

这一步非常重要，因为没有这一步，Web 页面虽然能看地图，但现场无法真正完成“先定位再导航”的闭环。

### 35.3 这一步和“网页点击地图让机器人走”是什么关系

关系非常直接：

- Web 前端点击地图，最终并不是直接发 ROS topic；
- 前端真实走的是后端 API；
- 后端再去发 `/initialpose` 和 `NavigateToPose`。

所以这一阶段用 `curl` 实测这些 API，本质上就是在验证“网页点击之后真正会走的那条后端链”。

---

## 36. 先把真机 `bringup` 本身变成稳定可复现

### 36.1 这一阶段还补了 `start_real_stack.sh`

为了避免每次人工手敲 launch 参数时不一致，这阶段把正式脚本也补强了。

关键文件：

```text
src/a2_system/tools/start_real_stack.sh
```

它现在会在 `A2_ENABLE_NAV2=true` 时自动补上：

1. `enable_nav2_bringup:=true`
2. `enable_control_bridge:=true`
3. `real_localization_mode:=manual_odom`

### 36.2 为什么还要处理 DDS 环境变量

现场另一类问题是：

- `configure_real_network.sh` 会准备 `RMW_IMPLEMENTATION` 和 `CYCLONEDDS_URI`
- 但 Unitree 真实 SDK 那边并不总适合继承这套 ROS 侧 DDS 绑定
- 直接带着这些环境去起真机控制链时，曾出现：

```text
Failed to create domain explicitly
```

因此正式脚本在真正执行 `ros2 launch` 前，还专门做了两件事：

1. `unset RMW_IMPLEMENTATION`
2. `unset CYCLONEDDS_URI`

这样做的目的不是否定 ROS 的 CycloneDDS 配置，而是让：

- ROS2 导航栈一侧保持正常；
- Unitree SDK 控制侧不要再被错误 DDS 环境拖崩。

### 36.3 正式拉起命令

这一阶段正式使用的拉起方式是：

```bash
A2_ENABLE_NAV2=true \
A2_MAP_YAML=/home/unitree/a2_system_ws/runtime/maps/test_map_20260423_1059/map.yaml \
/home/unitree/a2_system_ws/install/a2_system/share/a2_system/start_real_stack.sh eth0
```

这样拉起后，能在机器人上稳定得到一套完整的真机栈。

---

## 37. 现场真正导致“导航接单但真机不走”的根因

### 37.1 当时看到的关键现象

在第一轮动作验收里，已经看到了这些事实同时成立：

1. `POST /api/navigation/goal` 能成功返回。
2. `navigation.state=navigating`。
3. `controller_server` 持续 `Passing new path to controller`。
4. `a2_control_bridge` 进入过：

```text
reason=command_sent
```

但与此同时：

1. `pose` 几乎不变。
2. `raw_state.position` 几乎不变。
3. 最终 `controller_server` 报：

```text
Failed to make progress
```

### 37.2 这说明问题不在 Web，也不在 Nav2 接单

因为如果问题在更前面，现场不会出现：

- `command_sent`
- `Passing new path to controller`

所以真正的问题只能在更后面：

- `a2_control_bridge -> SportClient -> 真机运动状态`

### 37.3 用最小直接 SDK 实验把根因坐实

为了确认“SDK 到底能不能让真机动”，这阶段专门做了最小直接实验，不经过 Web，也不经过 Nav2，只直接打 Unitree SDK：

```python
BalanceStand()
Move(0.1, 0.0, 0.0)
StopMove()
```

现场返回码是：

```text
BalanceStand 0
Move 0
StopMove 0
```

同时，真机状态也真的变化了：

- `pose`
  - 从大约 `(-0.35, 0.25)` 变化到 `(-0.37, 0.30)`
- `raw_state.position`
  - 从大约 `(-0.005, 0.003, 0.059)` 变化到 `(-0.026, 0.054, 0.440)`

这一步的意义非常大，因为它把问题一下子切清楚了：

1. 不是 Unitree SDK 完全不能让真机运动。
2. 不是 `Move()` 接口本身无效。
3. 真正的问题是：控制桥没有给 A2 一个正确的“进入可运动平衡态”的前置准备时序。

---

## 38. 最终修掉的不是 Nav2，而是 `a2_control_bridge` 的启动时序

### 38.1 原来的实现为什么不够

原来的 `a2_control_bridge` 代码虽然已经支持：

```text
prepare_balance_stand
```

但原逻辑有一个关键问题：

1. 第一次收到真实速度命令时，
2. 在同一个 control tick 里立刻：
   - `BalanceStand()`
   - 然后马上 `Move()`

这对于真机来说太快了。

直接 SDK 实验已经证明：

- A2 在 `BalanceStand()` 之后需要一个很短但真实存在的缓冲时间，
- 否则虽然命令看起来发出去了，但真机不一定进入稳定运动状态。

### 38.2 最终修改的文件

这一步最终修改的是：

```text
src/a2_control_bridge/src/a2_control_bridge_node.cpp
```

### 38.3 这一步做了哪些具体修改

具体做了这些修改：

1. 真实模式下默认启用 `prepare_balance_stand`。
2. 新增 `prepare_balance_wait_sec`，真实模式默认 `2.0s`。
3. 第一次活跃命令到来时，不再直接 `Move()`，而是：
   - 先 `BalanceStand()`
   - 发布：

```text
state=preparing;reason=balance_stand
```

4. 在等待窗口内持续保持：

```text
state=preparing;reason=balance_stand_wait
```

5. 等待时间到后，再真正发 `Move()`。
6. 同时把 `Move()` / `StopMove()` 返回码也补了日志，方便继续排查。

### 38.4 为什么这一步是关键

因为这一步修掉之后，真机第一次收到 Web 发来的导航控制命令时，不再是“动作链看起来通了但真机不动”，而是：

1. 先进入 `BalanceStand`
2. 等 2 秒
3. 再真正进入 `command_sent`
4. 然后真机开始位移

这就是从“逻辑链闭环”变成“真机动作闭环”的关键分界线。

---

## 39. 真机现场：先强制清空所有旧 bringup，再只起一套新的

### 39.1 为什么要强制清场

因为这阶段反复遇到的问题是：

- 旧 `bringup.launch.py` 进程不一定会干净退出；
- 新 bringup 启动时，旧 `map_server` / `lifecycle_manager` 还在；
- 最后出现两套系统互相污染。

所以现场最终用了“强制清场”的方式，把下面这些残留统一杀掉：

1. `bringup.launch.py`
2. `a2_sdk_bridge`
3. `a2_control_bridge`
4. `manual_localization_publisher`
5. `goal_bridge`
6. `map_server`
7. `controller_server`
8. `planner_server`
9. `bt_navigator`
10. `velocity_smoother`
11. `lifecycle_manager`
12. 以及若干相关监控节点

### 39.2 清场后如何确认真的干净

清场后执行：

```bash
pgrep -af "bringup.launch.py|a2_sdk_bridge|a2_control_bridge|manual_localization_publisher|goal_bridge|map_server|bt_navigator|planner_server|controller_server|velocity_smoother|lifecycle_manager"
```

确认没有残留进程，再重新拉起。

### 39.3 最终单实例 fresh bringup

最终 fresh bringup 成功后，现场只保留了一套新的真实栈，例如：

- `bringup.launch.py`
- `a2_sdk_bridge`
- `a2_control_bridge`
- `manual_localization_publisher`
- `goal_bridge`
- `map_server`
- `controller_server`
- `planner_server`
- `bt_navigator`
- `velocity_smoother`

对应的现场日志文件为：

```text
/home/unitree/a2_system_ws/runtime/logs/bringup_real_20260423_173109.log
```

---

## 40. Web 真实闭环验收：先设初始位姿，再发导航目标

### 40.1 先用 Web 后端接口设置初始位姿

现场实际使用的是：

```bash
curl -X POST http://127.0.0.1:8080/api/localization/initialpose \
  -H "Content-Type: application/json" \
  -d '{"pose":{"x":-0.37,"y":0.30,"yaw":0.0,"frame_id":"map"}}'
```

返回成功后，立刻确认到了：

1. `action_server_ready=true`
2. `pose_received=true`
3. `/a2/localization/status`

```text
mode=real;state=ready;ready=true;reason=pose_ok
```

4. `/a2/real/report`

```text
mode=real;state=ready;ready=true;reason=ok
```

5. `/a2/control/status`

```text
mode=real;state=idle;ready=true;reason=cmd_timeout
```

这一步说明：

- 真机已经从“没有定位，不允许动”进入“定位 ready，可以接导航命令”的状态。

### 40.2 为什么这一步可以视为网页点击链的真实验证

因为 Web 页面上“设置初始位姿”最终走的就是这条 API。

所以这里虽然是用 `curl` 发的，但它验证的是网页点按钮后真实会走的后端逻辑。

---

## 41. 第一段真正跑通的真机导航：短程目标成功，机器人实际位移

### 41.1 验收目标点

第一段真正成功的目标点是：

```text
(-0.60, 0.50)
```

现场调用方式：

```bash
curl -X POST http://127.0.0.1:8080/api/navigation/goal \
  -H "Content-Type: application/json" \
  -d '{"goal":{"x":-0.60,"y":0.50,"yaw":0.0,"frame_id":"map"}}'
```

### 41.2 这次成功为什么可信

这次不是只有 action 状态变化，而是现场看到了完整的真机动作链：

1. `bt_navigator`

```text
Begin navigating from current location (-0.37, 0.30) to (-0.60, 0.50)
```

2. `a2_control_bridge`

```text
state=preparing;reason=balance_stand
```

3. 接着：

```text
BalanceStand triggered on interface 'eth0'; waiting 2.00s before Move().
```

4. 两秒后：

```text
Balance stand preparation completed after 2.05s on interface 'eth0'.
```

5. 然后控制桥进入：

```text
state=ready;reason=command_sent
```

6. `controller_server`

```text
Reached the goal!
```

7. `bt_navigator`

```text
Goal succeeded
```

### 41.3 真机位姿和原始状态都发生了真实变化

这一步最关键的证据不是日志，而是数据本身变化了。

导航开始前大致是：

- `pose`
  - `(-0.370, 0.300)`
- `raw_state.position`
  - `(-0.027, 0.053, 0.440)`

导航结束后大致变成：

- `pose`
  - `(-0.439, 0.335)`
- `raw_state.position`
  - `(-0.095, 0.088, 0.442)`

这说明：

1. 真机不是只在 action 状态机里“看起来成功”。
2. 真机确实发生了真实位移。
3. Web 后端发出的导航目标已经能驱动 A2 真机动作。

### 41.4 这一步和“网页点击地图能不能让机器人走”之间的关系

这一步已经足够证明：

- Web 页面上真正会调用的导航 API
- 已经可以驱动真机走一段短程导航

也就是说：

- “网页点图 -> 后端 API -> Nav2 -> a2_control_bridge -> Unitree SDK -> 真机运动”

这条链已经在真机上实测跑通。

---

## 42. 第二轮和第三轮复验说明了什么

### 42.1 第二轮短目标为什么不算有效动作样本

后面又补了几轮目标验证。

其中有一些目标虽然在 `navigation` 里显示：

```text
Goal succeeded
```

但它们并不都能算有效动作样本，原因是：

1. 当前 `Nav2` 的 `xy_goal_tolerance` 配置是：

```text
0.25m
```

2. 某些目标距离虽然在地图上看起来不近，但经过当前配置与局部状态后，很快就被判成“已经到达”。
3. 这类样本不能证明真机再次发生了可靠位移。

所以这阶段真正可信的动作样本，只把“发生明显位移的那一轮”作为主验收依据。

### 42.2 更远目标暴露出当前系统的真实短板

这阶段还故意发过一个更远目标：

```text
(-0.75, 0.55)
```

这一次现场出现了很典型的“系统已经能动，但长距离还不稳”的现象：

1. 控制桥多次进入 `command_sent`
2. 真机 `pose` / `raw_state.position` 的确继续变化
3. 但随后进入 recovery
4. `planner_server` 多次报：

```text
GridBased failed to create plan with tolerance 0.50
```

5. `controller_server` 还出现：

```text
No valid trajectories out of 419
```

6. recovery 中的 `spin` / `backup` 因碰撞判断失败
7. 最终这轮导航没有自然成功，需要手工取消

### 42.3 这说明现在到了什么程度

这说明当前系统已经不再是“不能动”，而是进入了一个更真实的阶段：

1. 短程目标已经能成功。
2. 更远目标会暴露：
   - 规划可行域
   - 恢复行为
   - 当前 `manual_odom` 定位漂移
   - 局部代价地图与碰撞判断

也就是说，系统已经从“链路问题”过渡到了“性能与稳定性问题”。

---

## 43. 对前文第 29-31 节的状态更正

### 43.1 `a2_control_bridge` 不再是“仍然关闭”

前文第 `29.1` 节写的是：

- `a2_control_bridge` 仍然关闭

这在第二阶段结束后已经过期。

最新状态是：

1. `a2_control_bridge` 已重新启用。
2. 真实 SDK 崩溃问题已不再是当前主阻塞。
3. 控制桥现在已经能在真机导航过程中进入：

```text
reason=command_sent
```

4. 并已经在至少一轮短程导航里驱动真机真实位移并成功到达。

### 43.2 “网页端动作级验证还没做”也已经过期

前文第 `29.3` 节写的是：

- 网页端动作级验证还没做最终闭环

这在第二阶段结束后也已经过期。

最新状态是：

1. Web 对应的初始位姿 API 已实测可用。
2. Web 对应的导航目标 API 已实测可用。
3. 通过这条链，真机已经完成过一段成功短程导航。

更准确的说法应该更新为：

- 网页端短程真机动作闭环已经做完；
- 但更远距离的稳定性和 recovery 表现还没有完全收敛。

### 43.3 “只差最后一步让机器人动起来”也已经过期

前文第 `31` 节最后写的是：

- 从网页上真实发一次导航目标，让机器人动起来，再验证停止导航。

这句话在第二阶段结束后也已经过期。

因为现在已经完成了：

1. 真实发导航目标
2. 真机实际运动
3. 短程目标成功
4. 长程失败目标的手动取消

所以当前更准确的一句话总结应该变成：

- 网页驱动真机短程导航已经打通，当前剩余问题是长距离导航稳定性而不是“机器人能不能被网页驱动起来”。

---

## 44. 当前这套系统现在到底到了什么状态

### 44.1 已经可以认为成立的结论

截至 `2026-04-23` 这轮现场联调结束，可以认为下面这些判断已经成立：

1. 正式 `start_real_stack.sh` 已能稳定拉起单实例真机导航栈。
2. `manual_odom` 真机定位链已经能支撑网页“先设初始位姿，再导航”的交互模式。
3. Web 后端对应接口已经能：
   - 设置初始位姿
   - 发送导航目标
   - 取消导航
4. `a2_control_bridge` 已经不再卡在“命令发出但真机不动”的状态。
5. 在补上 `BalanceStand` 等待时序后，A2 真机已经能被网页导航链驱动产生真实位移。
6. 至少一段短程目标已经在真机上成功到达。

### 44.2 当前还不能过度宣称的部分

当前还不能过度宣称的，是：

1. 不能说“任意地图点击都已经长距离稳定”。
2. 不能说“当前真机定位已经达到长期全局稳定”。
3. 不能说“recovery 行为已经完全适配 A2 这台真机”。

因为现场已经看到：

1. 更远目标会进入 recovery。
2. `GridBased` 会在某些情况下重规划失败。
3. `manual_odom` 更适合当前阶段的短程真机闭环，不等于长期全局导航最终形态。

---

## 45. 当前建议的真机复现方式

### 45.1 正式启动

```bash
ssh unitree@192.168.31.49

A2_ENABLE_NAV2=true \
A2_MAP_YAML=/home/unitree/a2_system_ws/runtime/maps/test_map_20260423_1059/map.yaml \
/home/unitree/a2_system_ws/install/a2_system/share/a2_system/start_real_stack.sh eth0
```

### 45.2 如果怀疑有旧残留，先强制清场

至少确认下面这些旧节点不存在：

1. `bringup.launch.py`
2. `a2_sdk_bridge`
3. `a2_control_bridge`
4. `map_server`
5. `controller_server`
6. `planner_server`
7. `bt_navigator`

### 45.3 先设置初始位姿

可以通过 Web 页面，也可以通过后端 API：

```bash
curl -X POST http://127.0.0.1:8080/api/localization/initialpose \
  -H "Content-Type: application/json" \
  -d '{"pose":{"x":-0.37,"y":0.30,"yaw":0.0,"frame_id":"map"}}'
```

然后确认：

1. `action_server_ready=true`
2. `pose_received=true`
3. `/a2/localization/status=ready`
4. `/a2/real/report=ok`

### 45.4 再发一个短程目标做闭环

当前最推荐的不是一开始就发远距离目标，而是先发一段短程目标验证动作链，例如：

```bash
curl -X POST http://127.0.0.1:8080/api/navigation/goal \
  -H "Content-Type: application/json" \
  -d '{"goal":{"x":-0.60,"y":0.50,"yaw":0.0,"frame_id":"map"}}'
```

### 45.5 当前最适合网页现场操作的方式

如果直接用网页做现场操作，当前建议流程是：

1. 打开：

```text
http://192.168.31.49:8080/
```

2. 先点“设置初始位姿”。
3. 在地图上点击当前机器人附近的正确位置。
4. 再点击一个短距离、明确在自由区里的目标点。
5. 先验证真机起步、转向、停住、成功到达。
6. 再逐渐尝试更远目标。

---

## 46. 第二阶段的一句话总结

这一次真正补上的，不是某个单独的 launch 参数，而是“网页导航能不能在 A2 真机上形成真实动作”这件事。

最终现场结论是：

1. 这条链已经能在真机上跑通短程闭环；
2. Web 点击背后的真实后端接口已经能驱动真机运动；
3. 当前剩余问题已经从“能不能动”转成了“长距离稳不稳定、恢复行为强不强”。
# 历史说明

本文件是历史开发日志，不是当前运行说明。当前真机入口、topic 合约、TF 合约、扫描任务和运维流程，请以 `readme/README.md` 与 `src/a2_system/docs/` 中的现行文档为准。
