#!/usr/bin/env python3

import argparse
import os
import pathlib
import shutil
import subprocess
import sys

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None


def load_yaml(path: pathlib.Path):
    if yaml is None or not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def default_config_dir() -> str:
    script_path = pathlib.Path(__file__).resolve()
    candidates = [
        script_path.parent / "config",
        script_path.parent.parent / "config",
    ]
    for candidate in candidates:
        if candidate.exists():
            return str(candidate)
    return str(candidates[0])


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True, check=False)


def ros2_pkg_exists(package_name: str) -> bool:
    ros2_bin = shutil.which("ros2")
    if ros2_bin is None:
        return False
    result = run([ros2_bin, "pkg", "prefix", package_name])
    return result.returncode == 0


def parse_link_rows():
    ip_bin = shutil.which("ip")
    if ip_bin is None:
        return []
    result = run([ip_bin, "-br", "link"])
    if result.returncode != 0:
        return []

    rows = []
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        rows.append({
            "name": parts[0],
            "state": parts[1],
            "raw": line,
        })
    return rows


def parse_ipv4_map():
    ip_bin = shutil.which("ip")
    if ip_bin is None:
        return {}
    result = run([ip_bin, "-4", "-br", "addr"])
    if result.returncode != 0:
        return {}

    ipv4_map = {}
    for line in result.stdout.splitlines():
        parts = line.split()
        if not parts:
            continue
        iface = parts[0]
        addresses = [token for token in parts[2:] if "." in token]
        ipv4_map[iface] = addresses
    return ipv4_map


def is_virtual_like(name: str) -> bool:
    prefixes = ("docker", "br-", "veth", "virbr", "vmnet", "wl", "tun", "tap", "tailscale", "Meta")
    return name.startswith(prefixes)


def is_wired_like(name: str) -> bool:
    return name.startswith(("en", "eth"))


def interface_ready_for_real(name: str, raw: str) -> bool:
    if name == "lo" or is_virtual_like(name):
        return False
    if "LOWER_UP" in raw:
        return True
    return "state UP" in raw


def summarize_interfaces():
    link_rows = parse_link_rows()
    ipv4_map = parse_ipv4_map()
    summaries = []
    for row in link_rows:
        name = row["name"]
        raw_detail = run(["ip", "-o", "link", "show", "dev", name]).stdout.strip()
        summaries.append({
            "name": name,
            "state": row["state"],
            "ipv4": ipv4_map.get(name, []),
            "ready_for_real": interface_ready_for_real(name, raw_detail),
            "wired_like": is_wired_like(name),
            "virtual_like": is_virtual_like(name),
            "raw": raw_detail or row["raw"],
        })
    return summaries


def main():
    parser = argparse.ArgumentParser(description="Preflight checks for the A2 host-side stack.")
    parser.add_argument(
        "--config-dir",
        default=os.environ.get("A2_CONFIG_DIR", default_config_dir()),
        help="Directory containing YAML config templates.",
    )
    parser.add_argument(
        "--interface",
        default=os.environ.get("A2_NETWORK_INTERFACE", ""),
        help="Preferred network interface to evaluate for real mode.",
    )
    parser.add_argument(
        "--mode",
        choices=["auto", "real"],
        default="auto",
        help="Evaluation mode. auto follows configuration.",
    )
    args = parser.parse_args()

    config_dir = pathlib.Path(args.config_dir)
    system_cfg = load_yaml(config_dir / "system.yaml")
    network_cfg = load_yaml(config_dir / "network.yaml")
    sdk_cfg = load_yaml(config_dir / "a2_sdk.yaml")
    motion_cfg = load_yaml(config_dir / "motion_limits.yaml")

    ros_distro = os.environ.get("ROS_DISTRO", "")
    rmw = os.environ.get("RMW_IMPLEMENTATION", "")
    sdk_root = pathlib.Path(os.environ.get("UNITREE_SDK2_ROOT", "/opt/unitree_robotics"))
    configured_iface = network_cfg.get("network", {}).get("network_interface", "")
    requested_iface = args.interface or configured_iface
    effective_mode = args.mode
    if effective_mode == "auto":
        effective_mode = "real"
    required_bins = ["ip", "ping", "ros2", "colcon"]
    optional_bins = ["rviz2"]
    required_packages = ["nav2_msgs", "tf2_ros"]
    optional_packages = ["nav2_bringup", "livox_ros_driver2", "fast_lio"]
    interface_summaries = summarize_interfaces()
    ready_real_candidates = [
        item["name"] for item in interface_summaries
        if item["ready_for_real"] and item["wired_like"] and not item["virtual_like"]
    ]
    existing_wired_candidates = [
        item["name"] for item in interface_summaries
        if item["wired_like"] and not item["virtual_like"] and item["name"] != "lo"
    ]

    print("=== A2 Preflight Check ===")
    print(f"config_dir          : {config_dir}")
    print(f"ros_distro          : {ros_distro or '<unset>'}")
    print(f"rmw_implementation  : {rmw or '<unset>'}")
    print(f"requested_mode      : {effective_mode}")
    print(f"configured_interface: {configured_iface or '<empty>'}")
    print(f"requested_interface : {requested_iface or '<empty>'}")
    print(f"unitree_sdk2_root   : {sdk_root}")

    missing = [name for name in ("system.yaml", "network.yaml", "a2_sdk.yaml") if not (config_dir / name).exists()]
    if missing:
        print(f"[WARN] missing config files: {', '.join(missing)}")

    if not ros_distro:
        print("[WARN] ROS_DISTRO is not set. Source /opt/ros/humble/setup.bash before launch.")
    elif ros_distro != "humble":
        print(f"[WARN] Expected ROS 2 humble, found {ros_distro}.")

    if not rmw:
        print("[WARN] RMW_IMPLEMENTATION is not set. rmw_cyclonedds_cpp is recommended for Unitree DDS.")

    if not sdk_root.exists():
        print("[WARN] UNITREE_SDK2_ROOT does not exist.")

    ip_cmd = shutil.which("ip")
    if ip_cmd:
        if interface_summaries:
            print("interfaces:")
            for item in interface_summaries:
                tags = []
                if item["wired_like"]:
                    tags.append("wired")
                if item["virtual_like"]:
                    tags.append("virtual")
                if item["ready_for_real"]:
                    tags.append("real-ready")
                ipv4 = ",".join(item["ipv4"]) if item["ipv4"] else "-"
                tag_str = ",".join(tags) if tags else "plain"
                print(f"  {item['name']:<18} state={item['state']:<8} ipv4={ipv4:<24} tags={tag_str}")
        else:
            print("[WARN] failed to inspect interfaces.")
    else:
        print("[WARN] `ip` command not found.")

    ros2_bin = shutil.which("ros2")
    if ros2_bin is None:
        print("[WARN] `ros2` command not found in PATH.")

    if effective_mode == "real":
        if not requested_iface:
            print("[WARN] real mode requested but network_interface is empty. Auto-detect will choose the first wired candidate.")
        if requested_iface:
            requested_summary = next((item for item in interface_summaries if item["name"] == requested_iface), None)
            if requested_summary is None:
                print(f"[WARN] requested interface `{requested_iface}` does not exist.")
            elif not requested_summary["ready_for_real"]:
                print(f"[WARN] requested interface `{requested_iface}` is present but not ready for real DDS traffic.")
        print(f"real_ready_candidates : {', '.join(ready_real_candidates) if ready_real_candidates else '<none>'}")
        if not ready_real_candidates and existing_wired_candidates:
            print(f"wired_candidates_only : {', '.join(existing_wired_candidates)}")
            print("[WARN] no wired interface is carrier-up. start_real_stack.sh will fall back to diagnostic mode.")
        if not ros2_pkg_exists("fast_lio"):
            print("[WARN] fast_lio is missing. Real 3D LiDAR/IMU SLAM will stay in orchestrator waiting mode.")

    print("tool_check          :")
    for name in required_bins:
        print(f"  {name:<18} {'ok' if shutil.which(name) else 'missing'}")
    for name in optional_bins:
        print(f"  {name:<18} {'ok' if shutil.which(name) else 'optional-missing'}")

    print("package_check       :")
    for name in required_packages:
        print(f"  {name:<18} {'ok' if ros2_pkg_exists(name) else 'missing'}")
    for name in optional_packages:
        print(f"  {name:<18} {'ok' if ros2_pkg_exists(name) else 'optional-missing'}")

    runtime_root = pathlib.Path(system_cfg.get("system", {}).get("map_root", "/tmp/a2_maps")).parent
    print(f"runtime_root        : {runtime_root}")
    if effective_mode == "real":
        if ready_real_candidates:
            suggested_iface = ready_real_candidates[0]
            print(
                f"network_config_step : install/a2_system/share/a2_system/configure_real_network.sh {suggested_iface}"
            )
            print(
                f"suggestion          : install/a2_system/share/a2_system/start_real_stack.sh {suggested_iface}"
            )
        else:
            print("suggestion          : no ready wired interface detected; connect the A2 Ethernet link first, then rerun this check.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
