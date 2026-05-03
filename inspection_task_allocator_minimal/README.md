# inspection_task_allocator_minimal

## 项目说明

这是论文《融合优先级与路径代价的四足机器人动态巡检任务分配方法》的最小 Python 仿真版本。当前程序实现的是任务分配算法本身，用于验证机器人在二维栅格地图中面对多个巡检任务点时，如何依据任务优先级、区域风险、异常权重、A* 路径距离、路径复杂度和能耗估计动态选择下一个巡检点。

本版本不是完整机器人导航系统，也不包含 ROS2、Nav2 或真实机器人控制。

## 文件说明

- `task_model.py`：定义巡检任务数据结构 `InspectionTask`。
- `astar_planner.py`：实现二维栅格地图上的 A* 路径规划与路径代价统计。
- `task_allocator.py`：实现本文 Proposed 方法，即融合优先级与路径代价的动态任务分配算法。
- `demo_simulation.py`：构造地图和任务点，运行最小仿真并打印结果。

## 运行方式

```bash
python3 inspection_task_allocator_minimal/demo_simulation.py
```

## 当前实现内容

- 巡检任务数据结构
- A* 路径代价计算
- 路径转弯次数统计
- 路径邻域障碍物统计
- 任务评分函数
- 动态任务选择
- 总路径长度统计
- 总巡检时间统计
- 高优先级任务平均响应时间统计

## 当前不包含内容

- 不包含 ROS2
- 不包含 Nav2
- 不包含 Gazebo
- 不包含真实四足机器人控制
- 不包含真实导航系统
- 不包含异常反馈动态触发
- 不包含 FS/NNF/AStarOnly 对比实验
- 不包含消融实验

## 对比算法 demo

新增 `baseline_methods.py` 与 `compare_methods_demo.py`，用于在相同地图和相同任务点上比较四种方法：

- `FS`：固定顺序巡检
- `NNF`：每轮选择 A* 路径长度最短的任务
- `AStarOnly`：每轮选择 A* 综合路径代价最低的任务
- `Proposed`：融合优先级、风险、异常权重、路径距离、路径复杂度和能耗估计的任务分配方法

运行命令：

```bash
python3 inspection_task_allocator_minimal/compare_methods_demo.py
```

该 demo 用于在同一张地图和同一组任务点上比较四种方法。

## 小车运动学仿真实验

新增 `vehicle_model.py`、`vehicle_path_follower.py`、`vehicle_simulator.py`、`vehicle_sim_experiment.py` 与 `export_vehicle_sim_figures.py`，用于验证任务分配算法在带速度、转向和执行时间约束的小车运动学模型下的执行表现。

说明：

1. 本实验不是 Gazebo；
2. 本实验不是 Nav2；
3. 本实验不是实车；
4. 本实验用于验证任务分配结果在带速度和转向约束的小车运动学模型下的执行表现；
5. A* 只用于生成路径点；
6. 小车模型沿路径点运动，记录真实模拟轨迹长度和执行时间。

运行命令：

```bash
python3 inspection_task_allocator_minimal/vehicle_sim_experiment.py
python3 inspection_task_allocator_minimal/export_vehicle_sim_figures.py
```

## 自适应 A-RH-PADS 实验

新增 `A-RH-PADS`（Adaptive Receding-Horizon Priority-Aware Dynamic Scheduler，自适应滚动时域优先级感知动态调度算法）。该方法引入动态 `lambda_t`，根据任务紧急度压力 `U_t`、异常压力 `A_t` 与路径压力 `D_t` 计算响应收益和运动代价之间的权衡系数。`lambda_t` 越大越偏响应优先，越小越偏运动代价控制。

`A-RH-PADS` 的目的不是路径最短，而是在异常反馈、高优先级任务响应和路径/转向/执行时间代价之间动态调整。原 `RH-PADS` 与 `RH-PADS-L` 保留为固定权重基线。小车实验不是 Gazebo、不是 Nav2、不是实车，仅为小车运动学仿真。

运行命令：

```bash
python3 inspection_task_allocator_minimal/adaptive_rh_pads_main_experiment.py
python3 inspection_task_allocator_minimal/adaptive_rh_pads_abnormal_experiment.py
python3 inspection_task_allocator_minimal/adaptive_rh_pads_ablation_experiment.py
python3 inspection_task_allocator_minimal/adaptive_vehicle_sim_experiment.py
python3 inspection_task_allocator_minimal/adaptive_rh_pads_significance.py
python3 inspection_task_allocator_minimal/export_adaptive_rh_pads_tables.py
python3 inspection_task_allocator_minimal/export_adaptive_rh_pads_figures.py
```

## 后续可扩展方向

- 增加异常反馈重规划实验
- 增加消融实验
- 增加权重敏感性实验
- 接入 ROS2/Nav2
- 接入真实四足机器人平台
