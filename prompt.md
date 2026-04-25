# A2 导航系统工业化落地执行说明

## 当前判断

当前 A2 真机链路已经处于“能跑”的状态：单雷达输入、地图、AMCL 位姿、Nav2 action、Web Console 状态面板都已经联通。距离工业级别的主要差距不在“有没有节点”，而在稳定性、可诊断性、定位闭环质量、目标执行精度、TF/odom 约束和部署前检查。

当前阶段先不依赖 SSH 进入机器人，所有代码先在本地完善。等机器人可 SSH 后，直接同步工作区、编译、启动、验证。

## 本地改造目标

1. 默认真实定位模式从 `manual_odom` 切到 `amcl`。
2. 保留 `manual_odom` 作为应急 fallback，但明确它会漂移，不作为工业默认。
3. 收紧 Nav2 goal tolerance、planner tolerance、controller 频率与速度平滑参数。
4. 让 `/odom` 对 Nav2 更友好：平面化 Z、平面化姿态、补齐协方差。
5. 强化 `goal_bridge`：校验 frame、坐标范围、四元数、action server、超时、结果状态。
6. 强化 `localization_gate`：支持 transient local AMCL pose、位姿 latch、协方差阈值、可诊断状态文本。
7. 强化 `manual_localization_publisher`：让协方差随 odom 外推距离增长，避免假装长期高精度。
8. 强化 `static_tf_manager`：跳过重复 child frame 和动态 frame 的静态发布，降低 TF 冲突风险。
9. 增加本地导航契约检查脚本，保证同步上机前能自动检查关键配置。

## 上机验证顺序

1. 同步本地工作区到 `/home/unitree/a2_system_ws`。
2. 编译：`colcon build --symlink-install`。
3. 运行本地契约检查：`ros2 run a2_system nav_contract_check.py` 或直接执行脚本。
4. 启动真实栈，并显式使用 AMCL：

```bash
ros2 launch a2_bringup bringup.launch.py \
  runtime_mode:=real \
  use_mock:=false \
  network_interface:=eth0 \
  enable_nav2_bringup:=true \
  enable_control_bridge:=false \
  real_localization_mode:=amcl \
  map:=/home/unitree/a2_system_ws/runtime/maps/test_map_20260423_1059/map.yaml
```

5. 等 AMCL active 后发布 `/initialpose`。
6. 验证 `/map`、`/scan`、`/amcl_pose`、`/tf`、`/odom`。
7. 验证 `/a2/localization_ok` 和 `/a2/real/report`。
8. 打开 Web Console：`http://192.168.31.49:8080/`。
9. 在空旷安全区域进行网页点选导航和停止导航测试。

## 工业化验收标准

1. `nav_contract_check.py` 通过。
2. ROS graph 中没有重复同名核心节点。
3. `/map -> /odom -> /base_link` TF 单一且连续。
4. `/a2/localization_ok` 稳定为 true，不靠无限长 latch 掩盖定位失效。
5. `/a2/real/report` 为 `ready=true`。
6. Web Console 能拿到地图、位姿、状态和 action server ready。
7. 点选目标后机器人能执行路径，停止按钮能取消 action。
8. 到点误差和朝向误差明显小于“跑通版”参数。

