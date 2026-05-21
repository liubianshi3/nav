# A2 Unitree DDS Isolation Audit

Date: 2026-05-21

Branch baseline: `origin/master` at `16845f7`.

## Scope

This audit searches the application code for Unitree SDK2, `libddsc.so.0`,
`LD_PRELOAD`, FastDDS/FastRTPS, and ROS RMW configuration. Vendored SDK headers
under `docker/unitree_sdk/**` and `docker/a2_sdk_headers/**` are not treated as
runtime integration sites.

## Current Findings

### ROS bridges directly load Unitree SDK2 today

- `src/a2_control_bridge/CMakeLists.txt` locates `unitree_sdk2`, `ddsc`, and
  `ddscxx`, defines `A2_ENABLE_UNITREE_SDK`, and links them into
  `a2_control_bridge_node`.
- `src/a2_control_bridge/include/a2_control_bridge/a2_control_bridge_node.hpp`
  includes `unitree/robot/channel/channel_factory.hpp` and
  `unitree/robot/a2/sport/sport_client.hpp` when `A2_ENABLE_UNITREE_SDK` is set.
  In real mode it initializes `ChannelFactory`, owns `SportClient`, calls
  `Move`, `StopMove`, `BalanceStand`, `StandUp`, `StandDown`, `RecoveryStand`,
  `Damp`, `SwitchGait`, `SpeedLevel`, `BodyHeight`, and `SetAutoRecovery`.
- `src/a2_sdk_bridge/CMakeLists.txt` locates and links `unitree_sdk2`, `ddsc`,
  and `ddscxx` into `a2_sdk_bridge_node` and `a2_light_bridge_node`.
- `src/a2_sdk_bridge/src/a2_sdk_bridge_node.cpp` includes Unitree DDS IDL and
  channel headers, initializes `ChannelFactory`, and subscribes to Unitree
  `SportModeState_`, `LowState_`, and `BmsState_` topics directly.
- `src/a2_sdk_bridge/src/a2_light_bridge_node.cpp` includes Unitree DDS IDL and
  channel headers, initializes `ChannelFactory`, and publishes Unitree `LowCmd_`
  directly.

Impact: the ROS bridge binaries are part of the ROS graph and can also load
Unitree SDK2/CycloneDDS libraries. This violates the target boundary where only
`unitree_agent` may own Unitree SDK2 and `libddsc.so.0`.

### Launch files deliberately inject FastDDS and libddsc into ROS bridge nodes

- `src/a2_bringup/launch/jt128_3d_navigation.launch.py` defines
  `_unitree_ddsc_env()` with `RMW_IMPLEMENTATION` defaulting to
  `rmw_fastrtps_cpp`, then prepends `/opt/unitree_robotics/lib/x86_64/libddsc.so.0`
  or `/unitree/opt/lib/libddsc.so.0` to `LD_PRELOAD`.
- The same launch applies that environment to both `a2_sdk_bridge_node` and
  `a2_control_bridge_node`.
- `src/a2_bringup/launch/bringup.launch.py` has the same `_unitree_ddsc_env`
  pattern and applies it to `a2_sdk_bridge_node`, `a2_light_bridge_node`, and
  `a2_control_bridge_node`.

Impact: FastDDS participants from bridge processes are intentionally mixed into
ROS Domain 0. The `libddsc.so.0` preload is also applied to ROS nodes, which is
explicitly forbidden by the isolation target.

### Docker standby startup also injects FastDDS and optional LD_PRELOAD

- `docker/entrypoint.sh` defaults container-wide `RMW_IMPLEMENTATION` to
  `rmw_fastrtps_cpp`.
- `docker/entrypoint.sh` starts standby `a2_control_bridge_node` with
  `RMW_IMPLEMENTATION=${A2_UNITREE_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}` and
  optional `LD_PRELOAD=${A2_CONTROL_BRIDGE_LD_PRELOAD}`.
- `docker/entrypoint.sh` starts standby `a2_sdk_bridge_node` with
  `RMW_IMPLEMENTATION=${A2_UNITREE_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}` and
  optional `LD_PRELOAD=${A2_SDK_BRIDGE_LD_PRELOAD:-${A2_CONTROL_BRIDGE_LD_PRELOAD:-}}`.
- `web_console/backend/stack_control.py` starts manual standby ROS bridge
  processes with `RMW_IMPLEMENTATION` defaulting to `rmw_fastrtps_cpp`; if
  `/opt/unitree_robotics/lib/x86_64/libddsc.so.0` exists, it injects it into
  `LD_PRELOAD`.

Impact: even when the main ROS graph environment is CycloneDDS, the standby
path can still start FastDDS bridge participants and preload Unitree DDS into
ROS processes.

### ROS Domain 0 defaults are already present, but not consistently enforced

- `docker/a2_ros.env` sets `ROS_DOMAIN_ID=0` and
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- `entrypoint.sh` defaults `RMW_IMPLEMENTATION` to `rmw_cyclonedds_cpp`.
- `src/a2_system/tools/setup_unitree_dds.sh` exports
  `RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}` and configures
  `CYCLONEDDS_URI`.
- Existing contract tests currently assert the old FastDDS bridge behavior:
  `web_console/backend/test/test_web_contracts.py::test_unitree_bridge_nodes_use_fastrtps_rmw`
  and
  `src/a2_system/test/test_nav2_3d_recovery_contract.py::test_real_a2_source_launch_uses_cyclonedds_with_unitree_bridge_isolation`.

Impact: the desired Domain 0 policy exists in some shared environment files, but
bridge-specific overrides bypass it.

## Required Isolation Changes

1. Move all Unitree SDK2 and `libddsc.so.0` ownership into a new non-ROS
   `unitree_agent` process.
2. Remove Unitree SDK2, `ddsc`, and `ddscxx` linking from
   `a2_control_bridge_node`, `a2_sdk_bridge_node`, and `a2_light_bridge_node`.
3. Remove Unitree headers and `ChannelFactory` usage from ROS bridge sources.
4. Replace bridge-to-SDK calls with local Unix Domain Socket IPC at
   `/run/a2/unitree_agent.sock`.
5. Force all ROS bridge startup paths to use:
   - `ROS_DOMAIN_ID=0`
   - `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
6. Remove `LD_PRELOAD=libddsc.so.0` from all ROS bridge startup paths.
7. Start `unitree_agent` separately, without creating ROS nodes, and allow
   Unitree SDK library path or preload only in that process environment.
8. Update tests and verification scripts so FastDDS and `libddsc.so.0` pollution
   are treated as failures for ROS bridge processes.

## Target Boundary Summary

- ROS Domain 0: pure ROS CycloneDDS only.
- `a2_control_bridge_ros`: ROS node, subscribes `/cmd_vel_safe`, sends control
  and stop messages over UDS, never links Unitree SDK2.
- `a2_sdk_bridge_ros`: ROS node, receives state and health over UDS, publishes
  `/a2/raw_state`, `/a2/battery`, and `/a2/status`, never links Unitree SDK2.
- `unitree_agent`: non-ROS process, only process allowed to load Unitree SDK2
  and `libddsc.so.0`, owns SDK lifecycle, owns robot-side DDS/API, and performs
  command timeout, IPC disconnect, SDK exception, and exit-time stop fallbacks.
