# A2 real Docker package

This package is intended to be built on the A2 host from the runnable workspace:

```bash
cd /home/unitree/a2_system_ws
./docker/build_real_image.sh
```

The image copies the A2 host Unitree SDK from `/opt/unitree_robotics`, builds the
ROS 2 workspace inside `/opt/a2_system_ws`, builds the Web frontend, and runs the
FastAPI Web console. By default the Docker entrypoint also autostarts the A2
source-code stack:

- newest saved 3D map found under `runtime/maps` -> navigation dry-run
- no saved 3D map -> JT128/DLIO mapping

The 3D navigation launch also starts `task_manager` and the
`auto_scan_mission` `/run_mission` action server, so route/inspection workflows
available in the a2sys source tree are exposed from the same container. Physical
motion remains disabled unless `A2_ENABLE_MOTION=true` and
`A2_LIVE_MOTION=true` are both set.

The build script defaults to these mirror images because the A2 site network may
not reach Docker Hub directly:

```text
docker.m.daocloud.io/library/node:20-bookworm-slim
docker.m.daocloud.io/library/ros:humble-ros-base-jammy
```

Override them if needed:

```bash
A2_NODE_IMAGE=node:20-bookworm-slim \
A2_ROS_IMAGE=ros:humble-ros-base-jammy \
./docker/build_real_image.sh
```

Run the container:

```bash
docker run -d --name a2-system-ws --restart unless-stopped \
  --net host --privileged \
  -e A2_DOCKER_START_MODE=auto \
  -e A2_JT128_INTERFACE=net1 \
  -e A2_SDK_INTERFACE=eth0 \
  -v /home/unitree/a2_system_ws/runtime/maps:/opt/a2_system_ws/runtime/maps \
  -v /home/unitree/a2_system_ws/runtime/logs:/opt/a2_system_ws/runtime/logs \
  -v /home/unitree/a2_system_ws/runtime/routes:/opt/a2_system_ws/runtime/routes \
  -v /home/unitree/a2_system_ws/runtime/reports:/opt/a2_system_ws/runtime/reports \
  a2-system-ws:real
```

Or with Compose:

```bash
cd /home/unitree/a2_system_ws
docker compose -f docker/docker-compose.a2.yml up -d --build
```

Useful startup modes:

```bash
# Default: map exists -> navigation dry-run, else mapping.
A2_DOCKER_START_MODE=auto docker compose -f docker/docker-compose.a2.yml up -d

# Web only, no robot stack.
A2_DOCKER_START_MODE=standby docker compose -f docker/docker-compose.a2.yml up -d

# Force mapping.
A2_DOCKER_START_MODE=mapping docker compose -f docker/docker-compose.a2.yml up -d

# Force navigation with a specific map, still dry-run unless live motion is set.
A2_DOCKER_START_MODE=navigation A2_NAV_MAP_ID=perfect4-29 \
  docker compose -f docker/docker-compose.a2.yml up -d
```

Live physical motion is intentionally opt-in:

```bash
A2_DOCKER_START_MODE=navigation A2_NAV_MAP_ID=perfect4-29 \
  A2_ENABLE_MOTION=true A2_LIVE_MOTION=true \
  docker compose -f docker/docker-compose.a2.yml up -d
```

Smoke test without starting robot motion:

```bash
./docker/smoke_test.sh
```

The smoke test forces `A2_DOCKER_START_MODE=standby`, runs the Web backend on
port `18080`, checks `/api/health`, checks that the ROS workspace is sourced,
then removes the test container.

Production URL:

```text
http://192.168.31.49:8080/
```

If the host `a2-web-console.service` is still running on port `8080`, stop it
before running the production container on the same port:

```bash
sudo systemctl stop a2-web-console.service
```
