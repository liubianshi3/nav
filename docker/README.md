# A2 real Docker package

This package is intended to be built on the A2 host from the runnable workspace:

```bash
cd /home/unitree/a2_system_ws
./docker/build_real_image.sh
```

The image copies the A2 host Unitree SDK from `/opt/unitree_robotics`, builds the
ROS 2 workspace inside `/opt/a2_system_ws`, builds the Web frontend, and runs the
FastAPI Web console. It does not start mapping or navigation automatically. The
Web UI starts and stops those stacks through the existing backend APIs.

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
  -v /home/unitree/a2_system_ws/runtime/maps:/opt/a2_system_ws/runtime/maps \
  -v /home/unitree/a2_system_ws/runtime/logs:/opt/a2_system_ws/runtime/logs \
  a2-system-ws:real
```

Smoke test without starting robot motion:

```bash
./docker/smoke_test.sh
```

The smoke test runs the Web backend on port `18080`, checks `/api/health`, checks
that the ROS workspace is sourced, then removes the test container.

Production URL:

```text
http://192.168.31.49:8080/
```

If the host `a2-web-console.service` is still running on port `8080`, stop it
before running the production container on the same port:

```bash
sudo systemctl stop a2-web-console.service
```
