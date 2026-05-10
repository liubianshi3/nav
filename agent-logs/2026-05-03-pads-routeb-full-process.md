# PADS Route B 全流程详细记录

## 1. 这份文档是干什么的

这份文档用来完整记录本轮工作的来龙去脉，目标是让一个刚接手仓库、甚至第一次看这个项目的人，也能快速明白下面几件事：

1. 我们原本想解决什么问题；
2. 为什么要走 Route B 这条“创新增强大修”路线；
3. 代码层面具体新增了什么；
4. 实验层面具体跑了什么；
5. 论文层面具体改了什么；
6. 最后得到的结果到底说明了什么；
7. 哪些结论可以说，哪些结论不能乱说；
8. 这些成果最后是怎么整理并推到 GitHub 上的。

如果你只想先看结论，可以直接跳到：

- [第 3 节：一句话总结](#3-一句话总结)
- [第 8 节：真实实验结果告诉了我们什么](#8-真实实验结果告诉了我们什么)
- [第 12 节：最终产物清单](#12-最终产物清单)

---

## 2. 项目背景：我们到底在研究什么

本项目研究的是：

**单机器人在线动态巡检任务调度问题。**

这句话很重要，必须拆开理解：

### 2.1 什么叫“单机器人”

这里只有一个机器人，不是多机器人协同。

所以我们不研究：

- 多机器人之间如何分工；
- 多机器人之间如何避免冲突；
- 多机器人资源如何分配。

### 2.2 什么叫“在线动态”

不是一开始把所有任务排完就永远不变，而是机器人执行过程中，会根据当前状态重新决定“下一步先去哪一个任务点”。

动态的来源包括：

- 机器人当前位置在变化；
- 剩余任务集合在变化；
- 任务异常状态可能在执行中触发；
- 路径代价和响应需求之间需要重新平衡。

### 2.3 什么叫“巡检任务调度”

这里更准确的词是“调度”而不是“分配”。

原因是：

- “任务分配”更容易让人想到多机器人 MRTA；
- 我们这里的核心动作是：**决定单个机器人接下来执行哪个任务、按什么顺序执行**；
- 所以问题本质更接近：**带优先级、异常反馈和运动代价约束的在线任务序列优化问题**。

### 2.4 为什么这件事值得做

如果只按最近距离走，机器人通常会有两个问题：

1. 路近，但高优先级任务可能被拖后；
2. 异常任务出现后，如果还是只看距离，响应会不够快。

所以我们要解决的核心矛盾是：

**关键任务响应** 和 **路径/时间/运动代价控制** 之间如何平衡。

---

## 3. 一句话总结

这轮工作不是简单“修个脚本”，而是把原先的巡检算法工作，升级成了一个更完整的框架型研究叙事：

> 我们从原来的固定权重滚动时域方法出发，新增了一个自适应版本 A-RH-PADS，系统跑完主实验、异常实验、消融实验、小车运动学仿真、显著性检验、表格导出、图片导出，再把论文主稿重构成 PADS Framework 的框架叙事，同时保持所有结论基于真实结果，不伪造、不夸大。

更直白一点：

- **RH-PADS**：主打“响应优先”；
- **A-RH-PADS**：主打“响应收益与运动代价可调权衡”；
- 最终论文不再只是“一个算法”，而是变成了：
  **PADS Framework（Priority-Aware Dynamic Scheduling Framework）**。

---

## 4. 本轮工作的总目标

本轮工作的目标不是只做代码，也不是只改论文，而是把整个研究链条补完整：

### 4.1 算法目标

在现有 RH-PADS / RH-v2-Light 基础上，新增一个更能回应“人工调参过重”质疑的方法：

- 新方法名称：`A-RH-PADS`
- 全称：`Adaptive Receding-Horizon Priority-Aware Dynamic Scheduler`
- 中文：`自适应响应—代价权衡调度方法`

### 4.2 实验目标

围绕这个新方法，补齐以下实验：

1. 主实验；
2. 异常反馈实验；
3. 自适应结构消融；
4. 小车运动学仿真；
5. 显著性检验；
6. 表格导出；
7. 图片导出。

### 4.3 论文目标

把论文从“一个巡检任务分配算法”，大胆升级成：

**面向关键任务响应与路径代价权衡的单机器人在线动态巡检任务调度框架**

并明确建立以下框架口径：

- `PADS Framework`：总框架
- `RH-PADS`：响应优先模式
- `A-RH-PADS`：自适应响应—代价权衡扩展模式

### 4.4 风险控制目标

这一点也非常重要。整个过程中，我们明确坚持了这些红线：

1. 不覆盖原有正确 Word 主稿；
2. 不覆盖已有有效 CSV；
3. 不删除旧算法；
4. 不伪造实验结果；
5. 不把 A-RH-PADS 写成全面优于 RH-PADS；
6. 不声称 Gazebo / Nav2 / 真实机器人实验已经完成；
7. 不声称路径最短或全局最优。

---

## 5. 我们具体做了什么

下面按时间和工作流来展开。

### 5.1 第一步：先保护已有成果

在开始大改之前，先做备份，避免“新想法还没跑稳，老成果先丢了”。

#### 5.1.1 备份论文主稿

源文件：

- `/home/dell/下載/AROC_quadruped_inspection_manuscript_RH_v2_refs_cn_figs_submission_format.docx`

备份文件：

- `/home/dell/下載/AROC_quadruped_inspection_manuscript_before_adaptive_routeB_20260503.docx`

#### 5.1.2 备份实验结果

备份目录：

- `inspection_task_allocator_minimal/results/backup_before_adaptive_20260503/`

备份了当时的重要结果文件，包括：

- `vehicle_sim_results.csv`
- `vehicle_sim_summary.csv`
- `vehicle_sim_records.json`
- `rh_v2_compare_results.csv`
- `rh_v2_compare_summary.csv`
- `rh_v2_abnormal_results.csv`
- `rh_v2_abnormal_summary.csv`
- `drf_rh_main_results.csv`
- `drf_rh_abnormal_results.csv`
- `final_paper_tables_clean.md`

#### 5.1.3 建立独立工作分支

新建分支：

- `routeB_adaptive_rh_pads`

这样做的好处是：

- 不会直接污染旧主线；
- 方便随时比较前后差异；
- 后面推 GitHub 时也更清晰。

---

### 5.2 第二步：新增自适应调度算法 A-RH-PADS

新增核心文件：

- `inspection_task_allocator_minimal/adaptive_rh_pads_allocator.py`

这里的设计思想，是本轮最重要的算法创新点。

#### 5.2.1 为什么要新增 A-RH-PADS

原来的 RH-PADS 更像是固定权重模式：

- 它强调高优先级和异常任务响应；
- 但容易被审稿人质疑“是不是权重太人工、太依赖经验调参”。

所以新的思路是：

不要再把论文主创新写成“六个固定权重的精细调参”，而是把目标重新组织成更高层的两部分：

1. `Response Utility`：响应收益
2. `Motion Cost`：运动代价

然后根据当前任务状态，动态计算一个自适应权衡系数 `lambda_t`。

#### 5.2.2 A-RH-PADS 的核心公式

任务紧急度：

```text
u_i(t) = clip(0.45 p_i + 0.35 r_i + 0.20 a_i(t), 0, 1)
```

其中：

- `p_i`：任务优先级
- `r_i`：区域风险
- `a_i(t)`：异常反馈权重

当前任务紧急度压力：

```text
U_t = 0.6 * max_i u_i(t) + 0.4 * mean(top-k u_i(t))
```

异常压力：

```text
A_t = max_i a_i(t)
```

路径压力：

```text
D_t = mean_i d̄_i(t)
```

自适应权衡系数：

```text
lambda_t = lambda_min + (lambda_max - lambda_min) * sigma(k0 + k_u U_t + k_a A_t - k_d D_t)
```

其中默认取值范围：

- `lambda_min = 0.25`
- `lambda_max = 0.85`

含义很直观：

- `lambda_t` 越大，越偏响应优先；
- `lambda_t` 越小，越偏运动代价控制。

#### 5.2.3 A-RH-PADS 的决策目标

对于候选序列 `Q_t`：

```text
J_A(Q_t) = lambda_t * R(Q_t) - (1 - lambda_t) * C(Q_t)
```

其中：

- `R(Q_t)`：响应收益
- `C(Q_t)`：运动代价

响应收益写成：

```text
R(Q_t) = Σ discount^(k-1) * u_qk(t) / (1 + finish_time_norm_qk)
```

运动代价写成：

```text
C(Q_t) = 0.70 * C_path + 0.20 * C_turn + 0.10 * C_time
```

也就是说，新的方法不再直接把很多小权重散在最终决策层，而是把“响应”与“代价”收束到一个更清晰的目标结构里。

#### 5.2.4 搜索策略

为了在线可用，不做全空间暴力搜索，而是用：

- 候选池筛选；
- 有限预测时域；
- Beam Search；
- 每次只执行最优序列的第一个任务；
- 然后滚动更新。

这既保留了前瞻性，又控制了计算量。

---

### 5.3 第三步：补齐配套实验脚本

为了不是“只加一个算法名字”，而是真正形成可验证的新工作流，我们新增了一整套实验脚本。

#### 5.3.1 主实验

文件：

- `inspection_task_allocator_minimal/adaptive_rh_pads_main_experiment.py`

输出：

- `results/adaptive_rh_pads_main_results.csv`
- `results/adaptive_rh_pads_main_summary.csv`

作用：

- 在统一地图、统一任务规模、统一随机种子下，对比多个方法；
- 重点观察路径长度、总巡检时间、高优先级响应时间、优先级加权完成时间；
- 检查 A-RH-PADS 相对 RH-PADS、RH-PADS-L 和传统基线到底表现如何。

#### 5.3.2 异常反馈实验

文件：

- `inspection_task_allocator_minimal/adaptive_rh_pads_abnormal_experiment.py`

输出：

- `results/adaptive_rh_pads_abnormal_results.csv`
- `results/adaptive_rh_pads_abnormal_summary.csv`

作用：

- 在执行前 3 个任务后，触发新的异常任务；
- 观察调度策略是否会对异常压力做出合理反应；
- 特别记录异常前后 `lambda_t` 的变化。

#### 5.3.3 自适应消融实验

文件：

- `inspection_task_allocator_minimal/adaptive_rh_pads_ablation_experiment.py`

输出：

- `results/adaptive_rh_pads_ablation_results.csv`
- `results/adaptive_rh_pads_ablation_summary.csv`

作用：

- 验证自适应机制是不是“有用但不神化”；
- 看去掉 `U_t`、`A_t`、`D_t` 等项会发生什么；
- 看固定 `lambda`、只看响应、只看代价会发生什么。

#### 5.3.4 小车运动学仿真

文件：

- `inspection_task_allocator_minimal/adaptive_vehicle_sim_experiment.py`

输出：

- `results/adaptive_vehicle_sim_results.csv`
- `results/adaptive_vehicle_sim_summary.csv`
- `results/adaptive_vehicle_sim_records.json`

作用：

- 把任务调度层的输出，接到一个简化的小车运动执行模型上；
- 引入速度、转向、航向变化、路径跟踪等因素；
- 看策略在“不是纯静态路径长度”的情况下，是否还能体现响应与代价之间的权衡。

注意：

- 这不是 Gazebo；
- 这不是 Nav2；
- 这不是实车；
- 这是一个**简化运动执行模型**上的验证。

#### 5.3.5 显著性检验

文件：

- `inspection_task_allocator_minimal/adaptive_rh_pads_significance.py`

输出：

- `results/adaptive_rh_pads_significance.csv`

作用：

- 给“差异是否只是随机波动”这件事一个统计层面的回答；
- 优先使用 `scipy.stats.ttest_rel`；
- 输出成对比较的 `p_value`、均值差等信息。

#### 5.3.6 表格与图片导出

文件：

- `inspection_task_allocator_minimal/export_adaptive_rh_pads_tables.py`
- `inspection_task_allocator_minimal/export_adaptive_rh_pads_figures.py`

输出：

- `results/adaptive_rh_pads_paper_tables.md`
- `results/figures/adaptive_rh_pads/` 下的多张图片

作用：

- 让实验结果能直接进入论文整理；
- 减少手工抄表和手工做图的出错风险。

---

### 5.4 第四步：实际把实验跑完

这一步很关键。我们不是只写脚本不运行，而是按顺序真的执行了全部实验链路。

运行顺序是：

```bash
python3 inspection_task_allocator_minimal/adaptive_rh_pads_main_experiment.py
python3 inspection_task_allocator_minimal/adaptive_rh_pads_abnormal_experiment.py
python3 inspection_task_allocator_minimal/adaptive_rh_pads_ablation_experiment.py
python3 inspection_task_allocator_minimal/adaptive_vehicle_sim_experiment.py
python3 inspection_task_allocator_minimal/adaptive_rh_pads_significance.py
python3 inspection_task_allocator_minimal/export_adaptive_rh_pads_tables.py
python3 inspection_task_allocator_minimal/export_adaptive_rh_pads_figures.py
```

这样做的价值在于：

- 代码、实验、表格、图、结论是一条连通的链；
- 不是“先写个结论，再想办法凑数据”；
- 而是“先让脚本真实运行，再据实写论文”。

---

### 5.5 第五步：重构论文主稿

这一部分是结构性大修，而不是只改几句摘要。

#### 5.5.1 使用的正确主稿

我们明确使用的正确主稿是：

- `/home/dell/下載/AROC_quadruped_inspection_manuscript_RH_v2_refs_cn_figs_submission_format.docx`

并且明确不再修改：

- `cleaned.docx`
- `cleaned_vehicle_sim.docx`
- 其他旧版本

#### 5.5.2 论文叙事的核心升级

原来更像是：

- 一个巡检任务分配算法；

后来升级成：

- 一个面向关键任务响应与路径代价权衡的单机器人在线动态巡检任务调度框架。

这个升级的意义很大：

1. 问题定位更准确；
2. 不容易再被误解成多机器人任务分配；
3. 算法之间的关系更清晰；
4. 论文逻辑从“一个工程版本方法”变成“一个框架 + 两种模式”。

#### 5.5.3 术语和命名重构

全文做了系统统一：

- `RH-Proposed-v2` -> `RH-PADS`
- `RH-v2-Light` -> `RH-PADS-L`
- `Proposed-Balanced` -> `Greedy-PADS`
- `A-RH-PADS` 保留为自适应扩展模式

总框架命名为：

- `PADS Framework`
- `Priority-Aware Dynamic Scheduling Framework`
- 中文：`优先级感知动态调度框架`

#### 5.5.4 论文中的两个模式

我们最后形成的论文口径是：

##### RH-PADS

- 响应优先主模式；
- 更强调高优先级任务和异常任务快速响应；
- 作为论文主方法保留。

##### A-RH-PADS

- 自适应响应—代价权衡扩展模式；
- 通过 `lambda_t` 动态调节响应收益和运动代价；
- 不写成全面替代 RH-PADS；
- 而是写成“代价受限场景下的扩展模式”。

#### 5.5.5 生成的论文版本

本轮工作过程中，生成过多个阶段性的 Word 版本：

- `/home/dell/下載/AROC_quadruped_inspection_manuscript_submission_format_PADS_adaptive_bold_revised.docx`
- `/home/dell/下載/AROC_quadruped_inspection_manuscript_PADS_adaptive_final_theory_patch.docx`
- `/home/dell/下載/AROC_quadruped_inspection_manuscript_PADS_theoretical_properties_added.docx`

这些文件不在仓库里，而是在本机下载目录中。原因很简单：

- 它们是本地论文交付物；
- 不是代码仓库必须管理的源码文件；
- 仓库里主要保留实验、脚本、结果、日志。

---

### 5.6 第六步：补强理论表述

为了回应潜在审稿意见，我们不只是改实验叙事，也补了理论分析。

#### 5.6.1 `lambda_t` 的有界性与反馈调节分析

新增了这样的理论讨论：

- `lambda_t` 不是为了保证全局最优；
- 它是任务调度层的在线反馈调节变量；
- 由于经过 Sigmoid 映射并限制在 `[lambda_min, lambda_max]`，所以始终有界；
- `U_t`、`A_t` 增大时，`lambda_t` 上升，更偏响应优先；
- `D_t` 增大时，`lambda_t` 下降，更偏运动代价控制。

这里的写法是有意保守的：

- 讲“有界性”；
- 讲“可解释性”；
- 讲“反馈调节”；
- 不讲“严格收敛证明”；
- 不讲“控制理论稳定性已证明”。

#### 5.6.2 小车运动学仿真的机器人学合理性

我们也专门补了一段说明：

- 虽然本文没有做 Gazebo、Nav2、实车；
- 但 PADS 的运动代价项不只是几何路径长度；
- 它可以接收执行时间、转弯次数、航向变化量、能耗估计等信息；
- 小车运动学仿真就是为了验证这类运动代价接口在简化执行模型下的作用。

这段话的目的是：

- 不夸大；
- 但也不把小车仿真写成“纯玩具”；
- 给它一个合理、诚实、工程上说得过去的定位。

#### 5.6.3 理论性质分析

后来又新增了一个更正式的小节：

- `1.6 理论性质分析`

其中包含：

1. 问题 NP-hard 说明；
2. 有限候选空间内最优性命题；
3. Beam 宽度单调性命题；
4. `lambda_t` 有界性命题；
5. 评分函数有界性命题；
6. 复杂度总结。

这些分析的作用不是“吹成理论最优”，而是：

- 让方法更严谨；
- 让边界更清楚；
- 让审稿人知道我们知道自己方法的能力范围在哪里。

---

### 5.7 第七步：把代码和结果推到 GitHub

最后，代码成果需要真正上传到远端仓库。

目标仓库：

- `https://github.com/liubianshi3/a2_system_ws`

#### 5.7.1 推送时遇到的问题

一开始走的是 HTTPS 远端，但碰到了认证问题：

- 当前机器没有可用的 HTTPS GitHub 凭据；
- `git push` 报错：无法读取 GitHub 用户名。

#### 5.7.2 解决方法

后来检查发现：

- 这台机器上已经有 SSH key；
- GitHub SSH 认证是通的。

于是切换为 SSH 推送：

- `git@github.com:liubianshi3/a2_system_ws.git`

#### 5.7.3 实际推送的分支

最终推上去的分支包括：

- `routeB_adaptive_rh_pads`
- `master`
- `main`

其中：

- `routeB_adaptive_rh_pads`：保留完整开发分支；
- `master`：同步到当前主要工作提交；
- `main`：通过合并方式同步，而不是粗暴改写远端历史。

#### 5.7.4 这次上传时刻意没带上的文件

下面这个文件没有上传：

- `AROC_quadruped_inspection_manuscript_clean.docx`

原因是：

- 它是仓库根目录里的一个旧 `docx`；
- 看起来与本次代码与实验主线无关；
- 为了避免把历史遗留文件混进这次提交，故意排除了它。

---

## 6. 新增/修改的关键文件清单

这一节给新手一个“文件地图”。

### 6.1 算法与实验核心脚本

- `inspection_task_allocator_minimal/adaptive_rh_pads_allocator.py`
- `inspection_task_allocator_minimal/adaptive_experiment_utils.py`
- `inspection_task_allocator_minimal/adaptive_rh_pads_main_experiment.py`
- `inspection_task_allocator_minimal/adaptive_rh_pads_abnormal_experiment.py`
- `inspection_task_allocator_minimal/adaptive_rh_pads_ablation_experiment.py`
- `inspection_task_allocator_minimal/adaptive_vehicle_sim_experiment.py`
- `inspection_task_allocator_minimal/adaptive_rh_pads_significance.py`
- `inspection_task_allocator_minimal/export_adaptive_rh_pads_tables.py`
- `inspection_task_allocator_minimal/export_adaptive_rh_pads_figures.py`

### 6.2 论文自动化辅助脚本

- `inspection_task_allocator_minimal/rewrite_submission_format_pads_adaptive.py`
- `inspection_task_allocator_minimal/patch_pads_theory_discussion.py`
- `inspection_task_allocator_minimal/add_theoretical_properties_to_pads_doc.py`

### 6.3 关键结果文件

- `inspection_task_allocator_minimal/results/adaptive_rh_pads_main_summary.csv`
- `inspection_task_allocator_minimal/results/adaptive_rh_pads_abnormal_summary.csv`
- `inspection_task_allocator_minimal/results/adaptive_rh_pads_ablation_summary.csv`
- `inspection_task_allocator_minimal/results/adaptive_vehicle_sim_summary.csv`
- `inspection_task_allocator_minimal/results/adaptive_rh_pads_significance.csv`
- `inspection_task_allocator_minimal/results/adaptive_rh_pads_paper_tables.md`

### 6.4 关键图片目录

- `inspection_task_allocator_minimal/results/figures/adaptive_rh_pads/`

### 6.5 说明文件

- `README.md`
- `inspection_task_allocator_minimal/README.md`
- `agent-logs/2026-05-03-pads-routeb-full-process.md`（本文件）

---

## 7. 从新手视角理解：我们做成了什么

如果你不是算法方向的人，可以把这轮工作的成果理解成三层。

### 7.1 第一层：把“响应优先”做得更清楚了

`RH-PADS` 不是简单贪心，而是：

- 先看一小段未来；
- 评估候选任务序列；
- 用 Beam Search 限制复杂度；
- 每次只执行序列中的第一个任务。

所以它更像“有限前瞻的在线调度”，而不是“走一步看一步”。

### 7.2 第二层：把“自适应权衡”做出来了

`A-RH-PADS` 的价值，不是“无敌”，而是：

- 它能根据场景变化，自动在响应和代价之间调节；
- 不是全靠固定权重死写到底；
- 当异常多、任务更急时，它会更偏响应；
- 当路径压力大时，它会更偏代价控制。

### 7.3 第三层：把论文从“一个方法”升级成了“一个框架”

这是非常重要的研究表达升级。

现在论文主线变成：

- 一个总框架：`PADS Framework`
- 两种模式：
  - `RH-PADS`：响应优先
  - `A-RH-PADS`：自适应响应—代价权衡

这比单独堆一个新名字要更有整体性，也更容易和实验结果保持一致。

---

## 8. 真实实验结果告诉了我们什么

这一节最关键。我们不只看“好不好看”，而是看“真实数据到底说明什么”。

### 8.1 主实验：A-RH-PADS 不是全面更强，而是更偏代价控制

主实验中，几个关键方法的均值结果如下：

| 方法 | 总路径长度 | 总巡检时间/s | 高优响应时间/s | 优先级加权完成时间/s |
|---|---:|---:|---:|---:|
| RH-PADS | 220.27 | 466.78 | 130.12 | 183.10 |
| RH-PADS-L | 219.43 | 465.39 | 134.24 | 182.57 |
| A-RH-PADS | 207.07 | 444.78 | 143.00 | 178.72 |
| A-RH-PADS-L | 212.43 | 453.72 | 146.64 | 182.60 |

这组结果说明：

#### RH-PADS 的特点

- 高优先级任务响应更快；
- 更符合“响应优先模式”的定位。

#### A-RH-PADS 的特点

- 路径更短；
- 总时间更低；
- 优先级加权完成时间也更好；
- 但高优先级平均响应时间变差了。

也就是说：

**A-RH-PADS 不是把 RH-PADS 全面打败了，而是把策略推向了更偏代价控制的一侧。**

这正是我们最后在论文里采用的诚实口径。

### 8.2 主实验显著性检验

`A-RH-PADS vs RH-PADS` 的关键显著性结果：

- 路径长度改善显著：`p = 0.0005448636`
- 总巡检时间改善显著：`p = 0.0005448636`
- 高优响应变差显著：`p = 0.0056104210`
- 优先级加权完成时间改善显著：`p = 0.0025645291`

这意味着：

- 不是“看起来好像差一点”；
- 而是统计上真的出现了这种 trade-off。

### 8.3 异常实验：lambda_t 的确会动，但异常响应没有全面反超 RH-PADS

异常实验中，关键结果如下：

| 方法 | 异常优先率/% | 异常响应时间/s | 高优响应时间/s |
|---|---:|---:|---:|
| RH-PADS | 52.50 | 116.64 | 135.99 |
| A-RH-PADS | 54.17 | 118.42 | 138.71 |

`A-RH-PADS` 记录到的 `lambda_t` 变化：

- 异常前 `lambda`：`0.618`
- 异常后 `lambda`：`0.785`
- 平均变化量：`+0.167`

这说明：

1. 自适应机制不是摆设，异常触发后它确实上调了响应权重；
2. 但即使如此，它的异常平均响应时间也没有全面超过 RH-PADS。

显著性结果也支持这个更克制的结论：

- `A-RH-PADS vs RH-PADS` 的异常平均响应时间：`p = 0.7322019234`
- 也就是：**没有显著差异**

所以论文里不能写成：

- “A-RH-PADS 异常响应全面更好”

只能写成：

- “它对异常压力具有可解释的自适应反应，但异常响应优势并不稳定或并不显著”。

### 8.4 消融实验：自适应机制确实有价值，但不是没有代价

消融实验的作用，是回答这个问题：

> 你这个自适应设计，到底是真的有意义，还是换个常数也差不多？

几个典型现象如下。

#### Full vs FixedLambda

| 方法 | 总路径长度 | 总巡检时间/s | 高优响应时间/s | 异常响应时间/s |
|---|---:|---:|---:|---:|
| A-RH-PADS-Full | 221.07 | 468.11 | 138.71 | 118.42 |
| A-RH-PADS-FixedLambda | 204.93 | 441.22 | 148.39 | 127.11 |

解读：

- `FixedLambda` 路更短、时间更少；
- 但 `Full` 的高优先级响应和异常响应更好。

这说明：

- 自适应机制确实在把策略往“更关心关键任务”的方向推；
- 但这不是免费的，通常会付出额外路径和时间代价。

#### Full vs CostOnly

| 方法 | 总路径长度 | 总巡检时间/s | 高优响应时间/s | 异常响应时间/s |
|---|---:|---:|---:|---:|
| A-RH-PADS-Full | 221.07 | 468.11 | 138.71 | 118.42 |
| A-RH-PADS-CostOnly | 189.60 | 415.67 | 158.14 | 149.79 |

解读非常直白：

- 只看代价，路径和时间明显更漂亮；
- 但关键任务响应明显更差。

这组对比非常适合用来解释：

**为什么不能只看路径短不短。**

#### Full vs NoFinishTimeResponse

`NoFinishTimeResponse` 去掉了“越早完成越有额外收益”的设计。

结果是：

- 路径更长；
- 总时间更差；
- 高优响应更差；
- 异常响应也更差。

这说明在 `R(Q_t)` 里加入完成时刻归一化项，是有实际意义的。

### 8.5 小车运动学仿真：权衡关系在简化执行模型下仍然存在

小车运动学仿真中，关键对比如下：

| 方法 | 车辆轨迹长度 | 执行时间/s | 高优响应时间/s |
|---|---:|---:|---:|
| RH-PADS-L | 211.95 | 420.87 | 125.70 |
| A-RH-PADS-L | 193.74 | 382.57 | 162.10 |

这组结果很有代表性：

- `A-RH-PADS-L` 在实际车辆轨迹长度和执行时间上更优；
- 但它的高优先级响应明显更差。

显著性检验也支持这个说法：

- 轨迹长度差异显著：`p = 6.2065e-05`
- 执行时间差异显著：`p = 2.0280e-05`
- 高优响应变差显著：`p = 3.5508e-06`

这说明：

1. 自适应模式确实能把策略推向更省路径、更省时间的一侧；
2. 但响应优先模式仍然更适合“关键任务必须快”的场景；
3. 这种权衡不是二维静态图上的假象，在引入速度与转向约束的简化执行模型下依旧存在。

---

## 9. 这些结果最后支持了什么结论

把全部实验、显著性检验和论文大修放在一起，最后最稳妥的结论是：

### 9.1 可以明确说的

1. `RH-PADS` 更适合作为**响应优先主方法**；
2. `A-RH-PADS` 能提供一种**响应—代价可调**的扩展模式；
3. `A-RH-PADS` 确实能在很多情况下减少路径长度和总时间；
4. 但它通常会牺牲一部分高优先级响应性能；
5. `lambda_t` 的变化在异常实验中是可解释的，而不是完全没反应。

### 9.2 不能乱说的

1. 不能说 `A-RH-PADS` 全面优于 `RH-PADS`；
2. 不能说我们的算法获得了全局最优；
3. 不能说小车运动学仿真等价于 Gazebo / Nav2 / 实车；
4. 不能说已经完成真实机器人验证；
5. 不能说所有提升都显著，必须看具体 `p` 值。

### 9.3 所以论文主线怎么定最合适

最合适的主线不是：

- “A-RH-PADS 完全取代 RH-PADS”

而是：

- `PADS Framework` 是总框架；
- `RH-PADS` 是响应优先主模式；
- `A-RH-PADS` 是自适应扩展模式；
- 两者共同说明关键任务响应与运动代价之间存在可调权衡。

这是最符合真实结果、也最不容易被审稿人抓住硬伤的说法。

---

## 10. 本轮在论文上具体达成了什么效果

这一节从“论文质量”的角度看成果。

### 10.1 题目升级了

从偏算法工程命名，升级成更像框架研究的问题表述：

- 强调“单机器人”
- 强调“在线动态巡检任务调度”
- 强调“框架”
- 强调“关键任务响应与路径代价权衡”

### 10.2 摘要升级了

摘要不再只讲“我们做了一个算法”，而是更完整地交代：

- 问题是什么；
- 为什么距离优先不够；
- PADS 框架是什么；
- RH-PADS 和 A-RH-PADS 分别是什么角色；
- 实验结果支持什么、不支持什么；
- 本文不涉及哪些系统级验证。

### 10.3 方法结构更清楚了

方法部分被重构成更像论文而不是代码说明书的结构：

1. 问题定义
2. 任务紧急度与运动代价建模
3. RH-PADS
4. A-RH-PADS
5. Beam Search 求解
6. 复杂度分析
7. 理论性质分析

### 10.4 审稿风险更低了

因为我们主动处理了很多可能的质疑：

- “是不是只是经验调参？”
- “为什么这个自适应变量可信？”
- “二维仿真是不是太弱？”
- “为什么说这是机器人背景，而不是纯路径排序？”
- “为什么不是全局最优？”

我们没有用夸大说法去挡这些问题，而是：

- 给出有界性、复杂度、NP-hard、有限候选空间内最优等保守但有力的分析；
- 给出小车运动学仿真的合理定位；
- 明确说未做 Gazebo / Nav2 / 实车。

---

## 11. 如果你是新手，现在应该怎么接手这个仓库

下面给一个最实用的接手建议。

### 11.1 先看哪些文件

建议按这个顺序看：

1. `inspection_task_allocator_minimal/README.md`
2. `inspection_task_allocator_minimal/adaptive_rh_pads_allocator.py`
3. `inspection_task_allocator_minimal/adaptive_rh_pads_main_experiment.py`
4. `inspection_task_allocator_minimal/adaptive_rh_pads_abnormal_experiment.py`
5. `inspection_task_allocator_minimal/adaptive_rh_pads_ablation_experiment.py`
6. `inspection_task_allocator_minimal/adaptive_vehicle_sim_experiment.py`
7. `inspection_task_allocator_minimal/results/adaptive_rh_pads_paper_tables.md`
8. `inspection_task_allocator_minimal/results/adaptive_rh_pads_significance.csv`

### 11.2 如果你想重新跑实验

建议顺序：

```bash
python3 inspection_task_allocator_minimal/adaptive_rh_pads_main_experiment.py
python3 inspection_task_allocator_minimal/adaptive_rh_pads_abnormal_experiment.py
python3 inspection_task_allocator_minimal/adaptive_rh_pads_ablation_experiment.py
python3 inspection_task_allocator_minimal/adaptive_vehicle_sim_experiment.py
python3 inspection_task_allocator_minimal/adaptive_rh_pads_significance.py
python3 inspection_task_allocator_minimal/export_adaptive_rh_pads_tables.py
python3 inspection_task_allocator_minimal/export_adaptive_rh_pads_figures.py
```

### 11.3 如果你想继续改论文

先理解当前论文口径：

1. 主方法仍然是 `RH-PADS`；
2. `A-RH-PADS` 是扩展模式，不是全面替代者；
3. 小车实验只是运动学仿真，不是系统级实机验证；
4. 理论部分讲的是“复杂性、有界性、有限候选空间内最优”，不是“全局最优和严格收敛”。

### 11.4 如果你想继续做更强验证

后续最自然的方向有：

1. 更高保真仿真；
2. Gazebo / Nav2 接口接入；
3. 更丰富的运动代价建模；
4. 真机平台验证；
5. 更系统的参数鲁棒性分析。

---

## 12. 最终产物清单

这一节是整个流程的落地交付总结。

### 12.1 代码与结果产物

已经在仓库中形成并保存：

- 自适应调度核心算法
- 主实验脚本
- 异常实验脚本
- 消融实验脚本
- 小车运动学仿真脚本
- 显著性检验脚本
- 表格导出脚本
- 图片导出脚本
- 对应 CSV / JSON / Markdown / 图片结果

### 12.2 论文产物

已经在本机下载目录中形成多个阶段性的新版 Word 文件：

- `AROC_quadruped_inspection_manuscript_submission_format_PADS_adaptive_bold_revised.docx`
- `AROC_quadruped_inspection_manuscript_PADS_adaptive_final_theory_patch.docx`
- `AROC_quadruped_inspection_manuscript_PADS_theoretical_properties_added.docx`

### 12.3 Git 产物

已经建立并同步：

- `routeB_adaptive_rh_pads`
- `master`
- `main`

### 12.4 过程记录产物

过程日志已经留在仓库中：

- `agent-logs/2026-05-02-agent-session.md`
- `agent-logs/2026-05-03-pads-routeb-full-process.md`

---

## 13. 最后一句话

如果要用一句最准确的话概括这轮工作，那就是：

> 我们不是简单给旧算法换了个名字，而是把“单机器人巡检任务调度”这条研究线，完整升级成了一个有新方法、有真实实验、有统计检验、有论文框架重构、也有理论边界说明的 PADS Framework 工作流，并且始终保持结果诚实、不夸大、不伪造。

这也是为什么这份记录要写得这么细：因为真正有价值的，不只是“最后多了哪些文件”，而是**这些文件背后各自解决了什么问题，以及它们最后是如何连成一条完整研究链条的**。
