# Task Manager

`task_manager.py` is the ROS 2 adaptation layer for the useful orchestration ideas
borrowed from the legacy ROS 1 container:

- a single command entrypoint instead of scattered ad-hoc scripts
- route asset storage under `runtime/routes`
- direct single-goal navigation support
- initial pose publishing
- map save/load/promote passthrough
- route mission lifecycle management by reusing `auto_scan_mission.py`

## Service

- Service: `/a2/task_manager/command`
- Type: `a2_interfaces/srv/NavCommand`

Supported commands:

- `list_maps`
- `save_map`
- `load_map`
- `promote_map`
- `set_mode`
- `send_goal`
- `cancel_goal`
- `set_initial_pose`
- `list_routes`
- `get_route`
- `save_route`
- `delete_route`
- `run_route`
- `stop_route`
- `route_status`

## Route assets

Routes are stored as YAML files under:

```text
${A2_WORKSPACE}/runtime/routes
```

The format is intentionally aligned with `auto_scan_mission.py`:

```yaml
mission_name: office_loop
waypoints:
  - id: p1
    x: 1.2
    y: 0.8
    yaw: 0.0
    dwell_sec: 0.0
    note: start
```

## Topics

- `/a2/task_manager/status`
- `/a2/task_manager/report`

The status topic uses the shared text contract:

```text
mode=...;state=...;ready=...;reason=...
```

Additional fields include:

- `current_mode`
- `active_map`
- `route_state`
- `route_id`
- `route_path`
- `report_path`

## Design choice

This node does not re-implement the route execution algorithm. It reuses the
existing `auto_scan_mission.py` runner as a subprocess so that:

- one mission validation path is preserved
- one Markdown/JSON/CSV report format is preserved
- route storage and web/task orchestration can evolve without forking the mission logic
