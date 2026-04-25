# A2 原生 SLAM 接入改造 Prompt

## 目标

把当前 `a2_system_ws` 的真实模式从：

- 使用机器人原生点云
- 使用外部 odom
- 由我们自己的 `occupancy_mapper` 做 2D 栅格建图

改造成：

- 地图和定位尽量直接复用 A2 机器人自带的原生 SLAM 能力
- `a2_system_ws` 不再承担低质量的自研建图主链
- `a2_system_ws` 只做读取、适配、状态汇总、Web/UI、任务编排、控制桥接

目标方向可以参考当前机器上运行的 `festive_johnson` 容器思路：

- 机器人自己维护地图资产
- 上层系统主要消费地图和定位结果
- 不再重复从点云硬生成一套简化地图

## 背景事实

当前现场已经确认：

- 宿主机 `/home/unitree/graph_pid_ws` 在跑机器人原生 SLAM / 感知链
- ROS1 Docker 容器 `festive_johnson` 在消费 `/nav_map/...` 下的地图资产做 localization
- 当前 `a2_system_ws` 真实模式并没有真正复用机器人原生建图结果
- 当前 `a2_system_ws` 主要是：
  - 读取 `/unitree/slam_lidar/points1`
  - 转成 `/mid360/points`
  - 再由 `occupancy_mapper` 用 `pointcloud + odom` 生成 `/map`

这条链路的问题是：

- 建图质量差
- 只是轻量 occupancy grid 叠图，不是完整 SLAM
- 没有 scan matching / loop closure / 全局优化
- 性能和稳定性都不够好

## 改造原则

1. 不再把 `occupancy_mapper` 作为真实模式主建图方案。
2. 尽量复用机器人原生地图、定位、里程计、点云结果。
3. `a2_system_ws` 保留自己的上层系统价值：
   - Web Console
   - 状态汇总
   - 控制桥
   - 任务流
   - 地图管理
   - Nav2 接口适配
4. 不强依赖 ROS1 容器本身，但可以借鉴它“只消费地图与定位”的架构思路。
5. 优先接宿主机原生 SLAM 输出，不优先继续强化自研简化建图算法。

## 期望架构

真实模式下应尽量变成：

- 机器人原生 SLAM 负责：
  - 建图
  - 定位
  - 地图资产维护
  - 原生 odom / pose / map 输出
- `a2_system_ws` 负责：
  - 读取原生 SLAM 输出
  - 转换为本系统统一接口
  - 提供 Web 可视化
  - 提供地图选择 / 激活 / 状态展示
  - 必要时把原生地图转换为 Nav2 可消费格式

## 实现要求

### 1. 先做现状盘点

先在 A2 上确认并形成清单：

- 宿主机原生 SLAM 当前实际发布了哪些话题
- 哪些是稳定可用的：
  - 地图话题
  - odom 话题
  - pose / localization 话题
  - 点云话题
  - tf
- 哪些地图文件目录是机器人原生维护的
- `festive_johnson` 当前究竟消费了哪些地图文件和定位输入

输出一份明确对照表：

- 原生接口名
- 消息类型
- 发布频率
- 是否稳定
- 与 `a2_system_ws` 现有接口的映射关系

### 2. 去掉“真实模式自己建图”的默认路径

真实模式下，原则上不再默认起：

- `occupancy_mapper`

如果还保留，也只能作为：

- fallback
- debug
- 实验模式

不能再作为主路径。

### 3. 新增“原生 SLAM 接入模式”

在 `a2_system_ws` 中新增明确的真实模式策略，例如：

- `real_native_slam`
- 或在现有 `runtime_mode=real` 下增加 `slam_source=native`

要求这条模式下：

- 不自行做主建图
- 读取机器人原生 map / odom / pose
- 必要时做 topic rename / relay / frame bridge / QoS 适配

### 4. 地图接入策略

优先支持两条路线：

1. 直接消费机器人原生发布的地图话题
2. 如果导航链必须吃 `map.yaml + pgm`，则提供转换/导出流程

要求明确区分：

- “可视化地图”
- “导航用地图”
- “原生 PCD 地图”
- “Nav2 二维占据栅格地图”

不要把这些概念混在一起。

### 5. 定位接入策略

如果机器人原生已经提供稳定定位结果，则：

- 不再默认使用 `manual_localization_publisher`
- 优先接入原生 pose / odom / localization 输出

只有在原生定位接口不可用时，才允许退回手动定位方案。

### 6. 对上层接口保持兼容

尽量保证以下上层接口继续可用：

- `/a2/slam/status`
- `/a2/real/report`
- `/a2/map_manager/status`
- `/a2/map_manager/active_map`
- Web Console 当前地图显示接口
- 上层导航/任务流状态接口

如果底层换成原生 SLAM，上层尽量无感。

## 不允许的方向

- 不要继续围绕 `occupancy_mapper` 做小修小补，试图把它打磨成正式 SLAM。
- 不要把“机器人原生点云 + 我们自己的简化二维栅格投影”继续当成长期正式方案。
- 不要默认把 ROS1 容器直接当唯一真相；优先判定宿主机原生 SLAM 才是主地图源。

## 推荐输出

最终至少给出以下结果：

1. 一份现状分析
   - 当前机器人原生 SLAM 输出清单
   - `a2_system_ws` 现有真实模式链路清单
   - 二者差异

2. 一份改造方案
   - 哪些节点保留
   - 哪些节点旁路
   - 哪些节点删除或降级为 fallback
   - 哪些适配层需要新增

3. 一份最小可落地改造路径
   - 第一阶段先接 odom / map
   - 第二阶段再接定位与地图管理
   - 第三阶段再决定是否保留 Nav2 或部分替换

4. 明确代码改动点
   - 启动文件
   - 配置文件
   - map manager
   - localization manager
   - Web backend / ROS bridge

## 验收标准

- 真实模式下不再依赖 `occupancy_mapper` 作为主建图链
- 能读到机器人原生 SLAM 的地图或其等价产物
- 能读到机器人原生定位/里程计结果
- `a2_system_ws` 上层状态、Web、地图管理仍可工作
- 给出清晰的“原生地图 -> 本系统消费接口”的映射与验证命令

## 交付倾向

优先目标不是“保留现有自研建图实现”，而是：

**让 `a2_system_ws` 像 `festive_johnson` 一样，站在机器人原生 SLAM 之上消费地图与定位结果，而不是重复做一套低质量建图。**
