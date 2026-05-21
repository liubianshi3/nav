# A2 Unitree DDS Isolation Architecture

## 改造前问题

旧架构把 ROS bridge 和 Unitree SDK2 放在同一类 ROS 进程里运行：

- `a2_control_bridge_ros` / `a2_sdk_bridge_ros` 可能直接 include/link Unitree SDK2。
- ROS bridge 启动路径可能注入 `LD_PRELOAD=libddsc.so.0`。
- 部分启动路径把 bridge 切到 `rmw_fastrtps_cpp`，导致 ROS Domain 0 内混入非 CycloneDDS ROS participant。
- `ros2 node list` 依赖的 DDS graph 可能被 FastDDS、CycloneDDS、Unitree CycloneDDS 组合污染，表现为空 node name、陈旧 daemon graph、bridge 崩溃后 graph 异常。

## 改造后架构

改造后拆成三个同级边界角色：

- `a2_control_bridge_ros`: ROS node，只订阅 `/cmd_vel_safe`，通过 UDS 发送控制/stop 到 `unitree_agent`。
- `a2_sdk_bridge_ros`: ROS node，只从 UDS 接收状态流，发布 `/a2/raw_state`、`/a2/battery`、`/a2/status`。
- `unitree_agent`: 非 ROS 常驻进程，唯一加载 Unitree SDK2 / `libddsc.so.0`，负责 Unitree DDS/API、SDK lifecycle、异常处理和 stop 兜底。

`a2_control_bridge_ros` 和 `a2_sdk_bridge_ros` 是同级业务适配层，不直接依赖彼此。

## ROS Domain 0 约束

ROS 侧必须满足：

- `ROS_DOMAIN_ID=0`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- 不允许 `rmw_fastrtps_cpp`
- 不允许 ROS bridge 使用 `LD_PRELOAD=libddsc.so.0`
- 不允许 ROS bridge link/load Unitree SDK2 或 `libddsc.so.0`

`ROS_DOMAIN_ID` 只约束 ROS graph，不用于控制 Unitree SDK DDS。Unitree SDK 侧使用独立参数 `A2_UNITREE_DDS_DOMAIN_ID` / `--dds-domain-id`，默认 `0`，因为真实机器人通常期望 SDK domain 0。隔离依赖进程边界和 UDS，而不是把 `ROS_DOMAIN_ID` 传给 SDK。

## Unitree SDK Boundary

`unitree_agent` 是唯一 SDK owner：

- 不创建 ROS node。
- 不 source `/opt/ros`。
- 不进入 `ros2 node list`。
- 可以配置 `LD_LIBRARY_PATH` / `A2_UNITREE_AGENT_LD_PRELOAD`。
- 加载 Unitree SDK2 / `libddsc.so.0`。
- 负责 `ChannelFactory::Init(A2_UNITREE_DDS_DOMAIN_ID, A2_SDK_INTERFACE)`。

## UDS IPC Boundary

IPC socket:

```text
/run/a2/unitree_agent.sock
```

这是 Unix Domain Socket，不是普通文件，不参与 DDS discovery，不跨网络暴露。Docker 模式通过共享 volume 暴露给两个容器：

```yaml
- /run/a2:/run/a2
```

协议为 length-prefixed protobuf 二进制帧，schema 固定在 `proto/a2/unitree_agent.proto`。UDS 只负责传输本机字节流，`.sock` 本身不会增长；每个消息帧格式为：

```text
uint32_be payload_length
protobuf Envelope payload
```

这里仅使用 protobuf 序列化，不引入 gRPC server、HTTP/2 或网络监听端口。

当前 `Envelope` 定义：

- `CONTROL`: 速度控制、gait、speed level、body height、command timeout。
- `STOP`: 显式 stop 及原因。
- `MOTION`: balance stand、stand up/down、switch gait 等动作命令。
- `LIGHT`: 灯光控制兼容路径。
- `STATE`: agent 上报机器人状态流。
- `HEALTH_STATUS`: agent 健康状态。
- `ACK`: 成功/失败确认。

## 控制链路

```text
/cmd_vel_safe
  -> a2_control_bridge_ros
  -> /run/a2/unitree_agent.sock
  -> unitree_agent
  -> Unitree SDK2
  -> A2 Robot
```

控制 bridge 只做 ROS 侧安全门、限速、命令超时、IPC client，不拥有 SDK。

## 状态链路

```text
A2 Robot
  -> Unitree SDK2
  -> unitree_agent
  -> /run/a2/unitree_agent.sock
  -> a2_sdk_bridge_ros
  -> /a2/raw_state / /a2/battery / /a2/status
```

真实电池状态由 `a2_sdk_bridge_ros` 从 `unitree_agent` 状态流转换发布。旧 `a2_battery_publisher.py` 保留为 mock/兼容发布器，不再 import Unitree SDK。

## 安全兜底逻辑

ROS bridge 层：

- `/cmd_vel_safe` 超时后发送 `STOP`。
- IPC 不可用时发布 bridge 安全状态，不继续复用旧命令。
- 安全门关闭、estop、localization/map 不满足时发送 stop。
- `/a2/control/status` 和 `/a2/control/state` 暴露原因。

`unitree_agent` 层：

- IPC 控制 client 断连后 stop。
- SDK init / command / stop 异常后 stop。
- 超过命令 timeout 未收到新 `CONTROL` 后 stop。
- 进程退出前尽量 stop。
- 日志使用 `[unitree_agent][safety_stop] reason=...` 标记触发原因。

## 启动方式

### 源码启动

源码启动仍然走同一条链路。`start_jt128_3d_stack.sh` 会在非外部 agent 模式下启动 `unitree_agent`：

源码构建需要 C++ protobuf 工具链，Dockerfile 已安装 `protobuf-compiler` 和 `libprotobuf-dev`；裸机源码构建时也需要同等依赖。

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export A2_UNITREE_AGENT_SOCKET=/run/a2/unitree_agent.sock
export A2_UNITREE_DDS_DOMAIN_ID=0
./src/a2_system/tools/start_jt128_3d_stack.sh
```

也可以手动分开启动：

```bash
src/a2_unitree_agent/scripts/start_unitree_agent.sh
ros2 run a2_control_bridge a2_control_bridge_node --ros-args -p use_mock:=false -p runtime_mode:=real -p ipc_socket_path:=/run/a2/unitree_agent.sock
ros2 run a2_sdk_bridge a2_sdk_bridge_node --ros-args -p use_mock:=false -p ipc_socket_path:=/run/a2/unitree_agent.sock
```

### Docker compose 启动

生产部署必须在 A2 主机 `192.168.31.49` 本机构建：

```bash
git pull
docker compose -f docker-compose.a2.yml build
docker compose -f docker-compose.a2.yml up -d
```

不要在 Mac 构建生产镜像，不使用 buildx，不做多平台镜像复制。compose 保留现有 `a2-system-ws` 服务，并新增 `unitree-agent` 服务：

- `unitree-agent`: 使用 `Dockerfile.unitree_agent`，包含 Unitree SDK2，`network_mode: host`，挂载 `/run/a2:/run/a2`。
- `a2-system-ws`: ROS bridge 容器，不包含 Unitree SDK2，`network_mode: host`，挂载同一个 `/run/a2:/run/a2`。

## 验收方式

非破坏性边界检查：

```bash
scripts/verify_a2_dds_isolation.sh
```

真实容错检查会 kill `unitree_agent` 并发布测试 `/cmd_vel_safe`，只在机器狗安全条件满足时执行：

```bash
scripts/verify_a2_dds_isolation.sh --destructive
```

脚本检查：

- `ros2 node list` 不包含 `unitree_agent`。
- `a2_control_bridge_ros` / `a2_sdk_bridge_ros` 进程环境为 CycloneDDS Domain 0。
- ROS bridge 进程 maps 中没有 `libddsc.so.0`。
- `unitree_agent` 进程 maps 中有 `libddsc.so.0`。
- `/run/a2/unitree_agent.sock` 是 socket。
- runtime 配置和进程不含 `rmw_fastrtps_cpp`。
- destructive 模式验证 kill agent 和 `/cmd_vel_safe` timeout 后进入安全状态。

## 回滚方案

此改造在分支 `feature/a2-unitree-dds-isolation` 上完成，不直接修改 `master`。回滚方式：

1. 停止新服务：

```bash
docker compose -f docker-compose.a2.yml down
```

2. 切回旧分支或旧 tag：

```bash
git checkout master
git pull
```

3. 重新按旧流程启动。

如果只需要临时绕过 agent，可停止 `unitree-agent` 服务并切回旧镜像/tag；不要在新架构下把 `LD_PRELOAD=libddsc.so.0` 加回 ROS bridge。
