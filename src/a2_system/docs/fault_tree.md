# A2 Fault Tree

This fault tree is used to classify failures during dry-run, navigation, scan
missions, and web-control validation.

## No Map

Symptoms:

- `/map` missing
- web map remains empty
- scan mission fails with `map_missing`

Checks:

- `ros2 topic echo --once /map`
- map server lifecycle state
- map manager active map

Likely fixes:

- start mapping mode
- load a saved map before navigation mode
- verify map server lifecycle activation

## No AMCL Pose

Symptoms:

- `/amcl_pose` missing
- `/a2/localization_ok=false`
- scan mission fails with `pose_missing`

Checks:

- `ros2 topic echo --once /amcl_pose`
- `ros2 topic echo /a2/localization/status`
- `/scan` availability
- `map->odom` TF availability

Likely fixes:

- publish initial pose
- verify AMCL lifecycle state
- verify scan frame and map frame consistency

## Localization Stale Or High Covariance

Symptoms:

- `/a2/localization/status` shows `stale_pose`
- `/a2/localization/status` shows `covariance_rejected`
- web blocks goal sending

Checks:

- AMCL pose timestamp freshness
- covariance indices `[0]`, `[7]`, `[35]`
- robot motion causing pose jumps

Likely fixes:

- improve initial pose
- slow down navigation
- inspect scan quality and map alignment

## TF Error

Symptoms:

- Nav2 cannot transform goal
- AMCL cannot publish stable pose
- warnings about duplicate static transforms

Checks:

- `ros2 run tf2_tools view_frames`
- `ros2 topic echo /tf_static`
- `ros2 topic echo /tf`

Likely fixes:

- remove duplicate static TF child
- ensure only one owner publishes `odom->base_link`
- ensure only localization owns `map->odom`

## NavigateToPose Unavailable

Symptoms:

- web action ready is false
- scan mission fails with `navigate_action_not_ready`

Checks:

- `ros2 action list | grep navigate_to_pose`
- Nav2 lifecycle nodes
- BT navigator logs

Likely fixes:

- activate Nav2 lifecycle
- load navigation map
- inspect BT navigator startup errors

## Goal Rejected Or Invalid

Symptoms:

- goal bridge reports `bad_frame`
- scan mission route validation fails
- web says target is outside map or no feasible grid

Checks:

- goal frame must be `map`
- target must be inside `/map`
- target cell must not be unknown or occupied

Likely fixes:

- choose another waypoint
- improve map coverage
- adjust route YAML after dry-run report

## Robot Does Not Move

Symptoms:

- Nav2 accepts goal but robot stays still
- `/cmd_vel` exists but body does not move

Checks:

- `a2_control_bridge` status if motion is intended
- `/a2/allow_motion`
- `/a2/control/status`
- robot physical safety mode

Likely fixes:

- for pure mapping/localization, keep control bridge disabled
- for motion tests, re-enable only after SDK crash path is fixed
- verify emergency stop and robot mode

## Web Console Stale Or Disconnected

Symptoms:

- WebSocket disconnected
- map or camera stops updating
- buttons disabled unexpectedly

Checks:

- `sudo journalctl -u a2-web-console.service -f`
- `/api/health`
- browser devtools network panel

Likely fixes:

- restart web service
- rebuild frontend static assets
- verify backend config topic names

## Camera Missing

Symptoms:

- camera panel shows no image
- health shows `camera_received=false`

Checks:

- `ros2 topic list | grep -i camera`
- `ros2 topic list | grep compressed`
- backend config `camera_image_topic`
- backend config `camera_compressed_topic`

Likely fixes:

- set the actual A2 camera topic in `web_console/backend/config.yaml`
- prefer compressed image topic when available
- install backend dependency `Pillow` if only raw `sensor_msgs/Image` is available
