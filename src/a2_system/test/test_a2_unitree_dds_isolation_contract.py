from __future__ import annotations

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]


def _read(path: str) -> str:
    return (REPO_ROOT / path).read_text(encoding="utf-8")


def test_ros_bridges_do_not_link_or_include_unitree_sdk2() -> None:
    bridge_files = {
        "src/a2_control_bridge/CMakeLists.txt": _read("src/a2_control_bridge/CMakeLists.txt"),
        "src/a2_control_bridge/include/a2_control_bridge/a2_control_bridge_node.hpp": _read(
            "src/a2_control_bridge/include/a2_control_bridge/a2_control_bridge_node.hpp"
        ),
        "src/a2_sdk_bridge/CMakeLists.txt": _read("src/a2_sdk_bridge/CMakeLists.txt"),
        "src/a2_sdk_bridge/src/a2_sdk_bridge_node.cpp": _read("src/a2_sdk_bridge/src/a2_sdk_bridge_node.cpp"),
        "src/a2_sdk_bridge/src/a2_light_bridge_node.cpp": _read("src/a2_sdk_bridge/src/a2_light_bridge_node.cpp"),
    }

    forbidden_tokens = [
        "find_package(unitree_sdk2",
        "A2_ENABLE_UNITREE_SDK",
        "UNITREE_DDSC_LIB",
        "UNITREE_DDSCXX_LIB",
        "<unitree/",
        "unitree::robot",
        "SportClient",
        "ChannelFactory",
        "ChannelSubscriber",
        "ChannelPublisher",
    ]
    for path, source in bridge_files.items():
        for token in forbidden_tokens:
            assert token not in source, f"{path} still contains forbidden Unitree SDK token {token!r}"


def test_only_unitree_agent_owns_unitree_sdk2_and_ddsc() -> None:
    agent_cmake = _read("src/a2_unitree_agent/CMakeLists.txt")
    agent_source = _read("src/a2_unitree_agent/src/unitree_agent.cpp")

    assert "find_package(unitree_sdk2" in agent_cmake
    assert "UNITREE_DDSC_LIB" in agent_cmake
    assert "UNITREE_DDSCXX_LIB" in agent_cmake
    assert "<unitree/" in agent_source
    assert "unitree::robot" in agent_source


def test_ros_launch_and_standby_paths_keep_bridges_on_cyclonedds_only() -> None:
    runtime_files = {
        "src/a2_bringup/launch/jt128_3d_navigation.launch.py": _read(
            "src/a2_bringup/launch/jt128_3d_navigation.launch.py"
        ),
        "src/a2_bringup/launch/bringup.launch.py": _read("src/a2_bringup/launch/bringup.launch.py"),
        "docker/entrypoint.sh": _read("docker/entrypoint.sh"),
        "web_console/backend/stack_control.py": _read("web_console/backend/stack_control.py"),
        "Dockerfile": _read("Dockerfile"),
        "docker-compose.a2.yml": _read("docker-compose.a2.yml"),
        "src/a2_system/scripts/a2_battery_publisher.py": _read("src/a2_system/scripts/a2_battery_publisher.py"),
    }

    for path, source in runtime_files.items():
        assert "rmw_fastrtps_cpp" not in source, f"{path} still contains FastDDS RMW"
        assert "A2_UNITREE_RMW_IMPLEMENTATION" not in source, f"{path} still has bridge RMW override"
        assert "A2_CONTROL_BRIDGE_LD_PRELOAD" not in source, f"{path} still preloads libddsc into control bridge"
        assert "A2_SDK_BRIDGE_LD_PRELOAD" not in source, f"{path} still preloads libddsc into sdk bridge"
        assert "from unitree.robot" not in source, f"{path} still imports Unitree SDK from a ROS process"

    launch_text = runtime_files["src/a2_bringup/launch/jt128_3d_navigation.launch.py"]
    assert "rmw_cyclonedds_cpp" in launch_text
    assert "unitree_agent" in runtime_files["docker/entrypoint.sh"]
    assert "platform:" not in runtime_files["docker-compose.a2.yml"]
    assert "docker buildx" not in runtime_files["docker-compose.a2.yml"]
    assert "^a2_unitree_agent[[:space:]]" in runtime_files["Dockerfile"]


def test_verification_script_checks_process_boundaries() -> None:
    script = _read("scripts/verify_a2_dds_isolation.sh")

    assert "ros2 node list" in script
    assert "unitree_agent" in script
    assert "libddsc.so.0" in script
    assert "/run/a2/unitree_agent.sock" in script
    assert "rmw_fastrtps_cpp" in script
