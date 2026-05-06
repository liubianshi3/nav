# A2 Autoware NDT Adapter Plan

This document defines the recommended integration boundary for replacing the current
host-side `pcd_relocalizer_3d` implementation with an Autoware-style NDT
scan-to-map localizer while preserving A2 runtime contracts.

## Goal

Adopt a mature pointcloud-map localization backend modeled on Autoware
`ndt_scan_matcher`, but keep the A2 stack interfaces stable:

- keep DLIO as the real 3D odometry source
- keep A2 topic names for localization outputs and readiness consumers
- keep A2 TF ownership rules
- keep Web, task, safety, and localization gate contracts stable

This is an adapter integration, not a whole-stack Autoware migration.

## Current A2 Boundary

Current 3D navigation-localization flow:

```text
/jt128/front/points
  + /jt128/dlio/odom
  + /initialpose
    -> pcd_relocalizer_3d
    -> /a2/relocalization/pose
    -> /a2/relocalization/status
    -> map -> odom
    -> localization_gate
    -> safety / Web / pose_goal_controller_3d
```

The active contracts that must remain stable are defined in:

- [architecture.md](/home/dell/a2_system_ws/src/a2_system/docs/architecture.md)
- [interface_contracts.md](/home/dell/a2_system_ws/src/a2_system/docs/interface_contracts.md)

## Recommended Integration Pattern

Use three layers:

```text
Layer 1: Sensor and odometry
  /jt128/front/points
  /jt128/dlio/odom

Layer 2: Autoware-style NDT localizer
  mature scan-to-map localization backend
  score, covariance, diagnostics, initial pose handling

Layer 3: A2 adapter and readiness surface
  /a2/relocalization/pose
  /a2/relocalization/status
  map -> odom
  A2-compatible readiness semantics for localization_gate, Web, and safety
```

This means:

- Autoware ideas and implementation quality should live inside the localization engine
- A2 owns the external contract
- DLIO stays in place

## What We Should Borrow Directly

These parts are good candidates for direct borrowing or close behavioral mimicry:

- NDT scan matcher core matching logic
- score and convergence criteria
- covariance estimation strategy
- initial pose trigger and re-initialization workflow
- diagnostics and failure classification
- dynamic map loading model
- pointcloud-aligned debug output behavior

Official references:

- Autoware localization methods:
  https://autowarefoundation.github.io/autoware-documentation/pr-480/how-to-guides/integrating-autoware/launch-autoware/localization-methods/
- Autoware Core ndt_scan_matcher:
  https://autowarefoundation.github.io/autoware_core/latest/localization/autoware_ndt_scan_matcher/

## What Must Stay A2-Specific

These interfaces should stay A2-native even if the localization engine comes from
Autoware or is heavily modeled on it:

- input topic names
- output topic names
- status string format used by current tools
- readiness semantics used by `localization_gate`
- TF ownership and `map -> odom` authority
- map manager integration
- Web-consumable status fields
- startup and shutdown scripts

Specifically, preserve:

- `/jt128/front/points`
- `/jt128/dlio/odom`
- `/initialpose`
- `/a2/relocalization/pose`
- `/a2/relocalization/status`
- `map -> odom`

## Adapter Contract

The adapter should present this stable A2 surface.

### Inputs

- `input_points_topic`: `/jt128/front/points`
- `input_odom_topic`: `/jt128/dlio/odom`
- `input_initial_pose_topic`: `/initialpose`
- `input_map_topic` or map-loader service boundary: current A2 map manager path

### Outputs

- `pose`: `/a2/relocalization/pose`
  - type: `geometry_msgs/msg/PoseWithCovarianceStamped`
  - frame: `map`
- `status`: `/a2/relocalization/status`
  - type: `std_msgs/msg/String`
  - shape: parseable key-value text
- TF: `map -> odom`
  - exactly one active owner in 3D localization mode

### Required Status Fields

The status payload should include these top-level fields:

- `state`
- `ready`
- `reason`
- `matcher=ndt`
- `score`
- `effective_correspondences`
- `iterations`
- `translation`
- `rotation_deg`
- `map_id`
- `live_cloud_topic`
- `odom_topic`

Recommended states:

- `waiting_seed`
- `waiting_map`
- `waiting_odom`
- `waiting_scan`
- `converging`
- `ready`
- `rejected`
- `error`

### Readiness Semantics

`localization_gate` should continue to reason over A2 outputs instead of learning
Autoware-specific internals. The adapter should convert NDT health into A2-ready
semantics:

- `ready=true` only when pose freshness, score, covariance, and correspondence
  thresholds are all acceptable
- `ready=false` when seed is missing, scan is stale, match quality is poor,
  covariance is inflated, or the solution is rejected

## Dynamic Map Loading Strategy

Autoware supports dynamic map loading around the vehicle. That idea is worth
adopting, but the service boundary should remain compatible with A2 map
management.

Recommended A2 direction:

1. Keep `map_manager` as the A2 map authority.
2. Extend `pointcloud_map_loader` or add a compatible differential-load service.
3. Let the NDT adapter request map tiles or surrounding map segments near the
   current pose estimate.
4. Keep the save/load/promote lifecycle in A2 tooling rather than replacing it
   with Autoware launch assumptions.

This allows us to borrow the dynamic loading model without forcing an Autoware map
service migration on day one.

## Two Practical Integration Options

### Option A: Behavioral Reimplementation

Rebuild the NDT localizer in A2 code while following Autoware behavior closely.

Pros:

- full control over interfaces
- no external package dependency
- easier packaging in current workspace

Cons:

- highest maintenance burden
- easiest way to accidentally re-create a less mature copy
- more math and diagnostics work remains on us

### Option B: Wrapped Autoware NDT

Run an Autoware NDT localizer or a locally vendored derivative and wrap it with
an A2 adapter.

Pros:

- strongest maturity path
- best leverage from proven scan matching behavior
- easier to inherit future parameter tuning ideas

Cons:

- requires bringing in the package and its dependency surface
- requires explicit adapter code for topics, status, TF, and map loading

Recommendation: prefer Option B.

## Recommended Next Step

Do not switch the whole stack at once. Move in this order:

1. Add an adapter design package or module boundary:
   - `autoware_ndt_adapter`
   - or an upgraded `pcd_relocalizer_3d` wrapper mode
2. Keep the external A2 topics exactly stable.
3. Feed Autoware-style NDT with:
   - `/jt128/front/points`
   - `/jt128/dlio/odom`
   - `/initialpose`
   - A2-managed pointcloud map
4. Convert the output into:
   - `/a2/relocalization/pose`
   - `/a2/relocalization/status`
   - `map -> odom`
5. Reuse `localization_gate`, Web, and safety layers unchanged at first.

## Minimal Acceptance Criteria

An A2 Autoware-NDT integration should not be called successful until all of the
following are true:

- DLIO remains the odometry source
- `/a2/relocalization/pose` remains stable and fresh
- bad matches do not update `map -> odom`
- `/a2/relocalization/status` is parseable and diagnostic-rich
- `localization_gate` continues working without topic contract changes
- Web continues showing localization state without backend contract churn
- dry-run robot validation passes before any motion enablement

## Current Blocker

The current development environment does not show an installed Autoware NDT
package in `ros2 pkg list`, so the immediate next work item is not runtime
integration. It is:

- decide whether to vendor the Autoware NDT package into the workspace, or
- install a compatible package on the robot/dev machine, then
- implement the A2 adapter boundary on top of it

## Bottom Line

We should borrow Autoware deeply at the localization engine layer, but we should
not surrender the A2 system boundary.

The right shape is:

```text
Autoware NDT inside
A2 interfaces outside
```
