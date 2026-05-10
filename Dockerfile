# === Stage 1: Build web frontend ===
FROM registry.cn-hangzhou.aliyuncs.com/linuxsuren/node:20-bookworm AS web-build
WORKDIR /web

COPY web_console/frontend/package*.json ./
RUN if [ -f package-lock.json ]; then npm ci; else npm install; fi

COPY web_console/frontend/ ./
RUN npm run build

# === Stage 2: Runtime image ===
FROM registry.cn-hangzhou.aliyuncs.com/linuxsuren/ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV A2_WORKSPACE=/opt/a2_system_ws
ENV UNITREE_SDK2_ROOT=/opt/unitree_robotics
ENV CONFIG_PATH=/opt/a2_system_ws/web_console/backend/config.docker.yaml
ENV LD_LIBRARY_PATH=/opt/unitree_robotics/lib:/opt/unitree_robotics/lib/x86_64

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash \
    build-essential \
    cmake \
    curl \
    iproute2 \
    iputils-ping \
    net-tools \
    procps \
    python3-colcon-common-extensions \
    python3-pip \
    python3-setuptools \
    python3-venv \
    python3-yaml \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-sensor-msgs-py \
    ros-humble-tf-transformations \
    ros-humble-robot-localization \
    ros-humble-imu-tools \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-autoware-internal-debug-msgs \
    ros-humble-autoware-map-msgs \
    ros-humble-slam-toolbox \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# Install Unitree SDK (bundled in docker/unitree_sdk/ for self-contained build)
COPY docker/unitree_sdk/ /opt/unitree_robotics/

# Install A2-specific SDK headers (shipped in repo under docker/a2_sdk_headers/)
COPY docker/a2_sdk_headers/a2/ /opt/unitree_robotics/include/unitree/robot/a2/

# Copy source code and build
WORKDIR /opt/a2_system_ws
COPY src ./src
COPY proto ./proto
COPY web_console/backend ./web_console/backend
COPY web_console/scripts ./web_console/scripts
COPY web_console/systemd ./web_console/systemd
COPY web_console/README.md ./web_console/README.md

# Copy pre-built web frontend from stage 1
COPY --from=web-build /backend/static ./web_console/backend/static

# Copy entrypoint
COPY docker/entrypoint.sh /usr/local/bin/a2-web-entrypoint

# Build all ROS2 packages (third-party autoware excluded, Hesai lidar driver included)
RUN chmod +x /usr/local/bin/a2-web-entrypoint \
    && chmod +x web_console/scripts/*.sh src/a2_system/tools/*.sh \
    && rm -rf src/third_party/autoware_localization/autoware_utils_pkg \
    && source /opt/ros/humble/setup.bash \
    && OUR_PACKAGES=$(colcon list \
        | grep -vE 'autoware_|fast_lio|livox_ros_driver2|direct_lidar_inertial_odometry' \
        | awk '{print $1}' \
        | tr '\n' ' ') \
    && colcon build --packages-select ${OUR_PACKAGES} \
    && pip3 install --no-cache-dir -r web_console/backend/requirements.txt \
    && mkdir -p runtime/maps runtime/logs \
    && rm -rf build log

EXPOSE 8080 50051

ENTRYPOINT ["/usr/local/bin/a2-web-entrypoint"]
LABEL description="A2 robot system — self-contained Docker image"
