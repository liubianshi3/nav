ARG NODE_IMAGE=registry.cn-hangzhou.aliyuncs.com/linuxsuren/node:20-bookworm
ARG ROS_IMAGE=registry.cn-hangzhou.aliyuncs.com/linuxsuren/device-navigation-base-image:dev

FROM ${NODE_IMAGE} AS web-build
WORKDIR /web

COPY web_console/frontend/package*.json ./
RUN --mount=type=cache,target=/root/.npm \
    if [ -f package-lock.json ]; then npm ci; else npm install; fi

COPY web_console/frontend/ ./
RUN npm run build && test -f /backend/static/index.html

FROM ${ROS_IMAGE} AS runtime

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    APT_OPTS=(-o Acquire::Retries=10 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30 -o Acquire::http::Pipeline-Depth=0) \
    && ok=0 \
    && for i in 1 2 3 4 5; do \
        apt-get "${APT_OPTS[@]}" update \
        && apt-get "${APT_OPTS[@]}" install -y --no-install-recommends --fix-missing \
            libeigen3-dev \
            libomp-dev \
            ros-humble-rosidl-default-generators \
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

ARG TARGETARCH

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
RUN if [[ "${TARGETARCH:-}" == "amd64" ]]; then \
      test -f /opt/unitree_robotics/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake; \
    else \
      rm -rf /opt/unitree_robotics/lib/cmake/unitree_sdk2 \
        /opt/unitree_robotics/lib/x86_64 \
        /opt/unitree_robotics/lib/*.so* \
        /opt/unitree_robotics/lib/*.a; \
    fi

ARG PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
ARG A2_REQUIRE_UNITREE_SDK=OFF

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
        | grep -vE 'autoware_|fast_lio|livox_ros_driver2' \
        | awk '{print $1}' \
        | tr '\n' ' ') \
    && colcon build --event-handlers console_direct+ --packages-select ${OUR_PACKAGES} --cmake-args \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
        -DA2_REQUIRE_UNITREE_SDK=${A2_REQUIRE_UNITREE_SDK} \
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
    && chmod +x src/a2_system/scripts/*.py
RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    'source /opt/a2_system_ws/install/setup.bash' \
    'export LD_LIBRARY_PATH=/opt/unitree_robotics/lib/x86_64:/opt/unitree_robotics/lib:${LD_LIBRARY_PATH:-}' \
    'export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}' \
    > /etc/profile.d/a2_system_ws.sh \
    && chmod +x /etc/profile.d/a2_system_ws.sh

EXPOSE 8080
ENTRYPOINT ["/usr/local/bin/a2-web-entrypoint"]
