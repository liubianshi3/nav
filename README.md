# A2 System Workspace

## 论文实验复现说明

### 1. 项目说明

本仓库中的 `inspection_task_allocator` 模块用于验证四足机器人动态巡检任务分配算法，支撑论文《融合优先级与路径代价的四足机器人动态巡检任务分配方法》的仿真实验。该方法的核心思想是融合任务优先级、区域风险、异常反馈、A* 路径距离、路径复杂度和能耗代价，从而在动态巡检场景中生成更合理的任务执行顺序。

### 2. 模块结构

`inspection_task_allocator` 包中主要文件及作用如下：

- `task_model.py`：定义巡检任务数据结构。
- `astar_planner.py`：实现基于二维栅格地图的 A* 路径规划。
- `task_allocator.py`：实现 Proposed 方法，即融合多因素代价的动态任务分配策略。
- `baseline_methods.py`：实现对比算法，包括 `FS`、`NNF` 和 `AStarOnly`。
- `demo_simulation.py`：单次仿真运行入口，用于快速验证方法效果。
- `experiment_runner.py`：批量实验运行入口，自动生成多组对比实验结果。
- `analyze_results.py`：统计实验结果并生成论文表格所需的汇总数据。

### 3. 单次仿真运行方式

运行命令如下：

```bash
python3 src/inspection_task_allocator/inspection_task_allocator/demo_simulation.py
```

该脚本会在二维栅格地图上生成巡检任务并执行单次仿真，输出内容包括：

- 任务执行序列
- 总路径长度
- 总巡检时间
- 高优先级任务平均响应时间

### 4. 批量实验运行方式

运行命令如下：

```bash
python3 src/inspection_task_allocator/inspection_task_allocator/experiment_runner.py
```

批量实验设置如下：

- 地图规模：30 × 30
- 障碍物比例：0.1、0.2、0.3
- 任务数量：10、20、30
- 每组重复 20 次
- 对比方法：`FS`、`NNF`、`AStarOnly`、`Proposed`
- 同一 `seed` 下四种方法使用相同地图和任务点

### 5. 结果统计方式

运行命令如下：

```bash
python3 src/inspection_task_allocator/inspection_task_allocator/analyze_results.py
```

统计脚本会基于批量实验结果生成以下文件：

- `src/inspection_task_allocator/results/summary_by_task_num.csv`
- `src/inspection_task_allocator/results/summary_by_obstacle_ratio.csv`
- `src/inspection_task_allocator/results/summary_overall.csv`

### 6. 方法说明

四种方法定义如下：

- `FS`：固定顺序巡检，按照任务列表原始顺序依次执行。
- `NNF`：最近邻优先，每轮选择当前机器人位置到任务点 A* 路径长度最短的未完成任务。
- `AStarOnly`：仅基于 A* 路径综合代价选择任务，综合代价为路径长度、转弯数和障碍邻近数量的加权和。
- `Proposed`：融合任务优先级、区域风险、异常反馈以及路径代价的动态任务分配方法。

### 7. 注意事项

当前实验基于二维栅格仿真环境，主要用于论文方法验证与对比分析。后续可进一步接入 ROS2、Nav2 以及真实四足机器人平台，以完成实机验证与系统级联调。

### 8. 自适应 A-RH-PADS 实验

Route B 新增 `A-RH-PADS`（Adaptive Receding-Horizon Priority-Aware Dynamic Scheduler，自适应滚动时域优先级感知动态调度算法）。该方法将任务调度目标拆成响应收益 `R(Q_t)` 与运动代价 `C(Q_t)`，并根据当前任务紧急度压力 `U_t`、异常压力 `A_t` 和路径压力 `D_t` 动态计算 `lambda_t`。`lambda_t` 越大，调度越偏响应优先；`lambda_t` 越小，调度越偏运动代价控制。

该方法的目标不是追求路径最短，而是在任务响应与运动代价之间做动态权衡。原 `RH-PADS` / `RH-v2-Light` 保留为固定权重基线。小车运动学实验仅是二维栅格路径加差速小车跟踪的运动学仿真，不是 Gazebo、Nav2 或真实机器人实验。

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
