ARG NODE_IMAGE=registry.cn-hangzhou.aliyuncs.com/linuxsuren/node:20-bookworm
ARG ROS_IMAGE=registry.cn-hangzhou.aliyuncs.com/linuxsuren/ros:humble-ros-base-jammy

FROM ${NODE_IMAGE} AS web-build
WORKDIR /web

COPY web_console/frontend/package*.json ./
RUN --mount=type=cache,target=/root/.npm \
    if [ -f package-lock.json ]; then npm ci; else npm install; fi

COPY web_console/frontend/ ./
RUN npm run build && test -f /backend/static/index.html

FROM ${ROS_IMAGE} AS runtime

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV A2_WORKSPACE=/opt/a2_system_ws
ENV UNITREE_SDK2_ROOT=/opt/unitree_robotics
ENV CONFIG_PATH=/opt/a2_system_ws/web_console/backend/config.docker.yaml
ENV LD_LIBRARY_PATH=/opt/unitree_robotics/lib/x86_64:/opt/unitree_robotics/lib
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ARG UBUNTU_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu
ARG UBUNTU_SECURITY_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu
ARG ROS2_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
RUN sed -i "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_MIRROR}|g; s|http://security.ubuntu.com/ubuntu|${UBUNTU_SECURITY_MIRROR}|g" /etc/apt/sources.list \
    && sed -i "s|^URIs: .*|URIs: ${ROS2_MIRROR}|" /etc/apt/sources.list.d/ros2.sources \
    && sed -i "s|^Types: .*|Types: deb|" /etc/apt/sources.list.d/ros2.sources

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    APT_OPTS=(-o Acquire::Retries=10 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30 -o Acquire::http::Pipeline-Depth=0) \
    && ok=0 \
    && for i in 1 2 3 4 5; do \
        apt-get "${APT_OPTS[@]}" update \
        && apt-get "${APT_OPTS[@]}" install -y --no-install-recommends --fix-missing \
            bash \
            build-essential \
            ccache \
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
            ros-humble-octomap \
            ros-humble-octomap-msgs \
            ros-humble-octomap-ros \
            ros-humble-octomap-server \
            ros-humble-pointcloud-to-laserscan \
            ros-humble-autoware-internal-debug-msgs \
            ros-humble-autoware-map-msgs \
            ros-humble-autoware-ndt-scan-matcher \
            ros-humble-slam-toolbox \
            ros-humble-pcl-ros \
            ros-humble-pcl-conversions \
        && ok=1 \
        && break; \
        dpkg --configure -a || true; \
        apt-get -f install -y || true; \
        apt-get clean; \
        echo "apt-get failed (attempt ${i}/5), retrying..." >&2; \
        sleep 10; \
      done \
    && test "${ok}" = "1" \
    && rm -rf /var/lib/apt/lists/*

# Use the bundled Unitree SDK so the image can build on hosts without buildx.
COPY docker/unitree_sdk/ /opt/unitree_robotics/
RUN test -f /opt/unitree_robotics/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake
RUN set -euo pipefail; \
    fix_so_link() { \
      dir="$1"; \
      soname="$2"; \
      realname="$3"; \
      realpath="${dir}/${realname}"; \
      sonamepath="${dir}/${soname}"; \
      if [ -f "${realpath}" ] && [ ! -e "${sonamepath}" ]; then \
        ln -s "${realname}" "${sonamepath}"; \
      fi; \
      if [ -e "${sonamepath}" ] && [ ! -L "${sonamepath}" ]; then \
        size="$(stat -c%s "${sonamepath}" 2>/dev/null || echo 0)"; \
        if [ "${size}" -lt 4096 ]; then \
          content="$(tr -d '\r\n\0' < "${sonamepath}" | head -c 128 || true)"; \
          if [ "${content}" = "${realname}" ] && [ -f "${realpath}" ]; then \
            rm -f "${sonamepath}"; \
            ln -s "${realname}" "${sonamepath}"; \
          fi; \
        fi; \
      fi; \
    }; \
    for d in /opt/unitree_robotics/lib /opt/unitree_robotics/lib/x86_64; do \
      fix_so_link "${d}" libddscxx.so.0 libddscxx.so; \
      fix_so_link "${d}" libddsc.so.0 libddsc.so; \
    done
COPY docker/a2_sdk_headers/a2/ /opt/unitree_robotics/include/unitree/robot/a2/

WORKDIR /opt/a2_system_ws
ARG PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple

COPY web_console/backend/requirements.txt ./web_console/backend/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    pip3 config set global.index-url ${PIP_INDEX_URL} \
    && pip3 install -U "pip<25" "setuptools<70" "packaging<24" wheel \
    && pip3 install -r web_console/backend/requirements.txt

COPY src ./src
COPY proto ./proto
RUN rm -rf src/third_party/autoware_localization/autoware_utils_pkg

RUN --mount=type=cache,target=/root/.ccache,sharing=locked \
    source /opt/ros/humble/setup.bash \
    && export CCACHE_DIR=/root/.ccache \
    && OUR_PACKAGES=$(colcon list \
        | grep -vE 'autoware_|fast_lio|livox_ros_driver2|direct_lidar_inertial_odometry' \
        | awk '{print $1}' \
        | tr '\n' ' ') \
    && colcon build --event-handlers console_direct+ --packages-select ${OUR_PACKAGES} --cmake-args \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    && rm -rf build log

COPY web_console/backend ./web_console/backend
COPY web_console/scripts ./web_console/scripts
COPY web_console/systemd ./web_console/systemd
COPY web_console/README.md ./web_console/README.md
COPY --from=web-build /backend/static ./web_console/backend/static
COPY docker/entrypoint.sh /usr/local/bin/a2-web-entrypoint
RUN sed -i 's/\r$//' /usr/local/bin/a2-web-entrypoint \
    && find /opt/a2_system_ws/web_console/scripts /opt/a2_system_ws/src/a2_system/tools -type f -name "*.sh" -print0 | xargs -0 sed -i 's/\r$//' \
    && chmod +x /usr/local/bin/a2-web-entrypoint \
    && chmod +x web_console/scripts/*.sh src/a2_system/tools/*.sh \
    && mkdir -p runtime/maps runtime/logs
RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    'source /opt/a2_system_ws/install/setup.bash' \
    'export LD_LIBRARY_PATH=/opt/unitree_robotics/lib/x86_64:/opt/unitree_robotics/lib:${LD_LIBRARY_PATH:-}' \
    'export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}' \
    > /etc/profile.d/a2_system_ws.sh \
    && chmod +x /etc/profile.d/a2_system_ws.sh

EXPOSE 8080
ENTRYPOINT ["/usr/local/bin/a2-web-entrypoint"]
