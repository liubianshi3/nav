from setuptools import setup

package_name = "a2_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/abc_isolation_eval.launch.py",
            "launch/bringup.launch.py",
            "launch/sensors.launch.py",
            "launch/jt128_driver.launch.py",
            "launch/dlio_mapping.launch.py",
            "launch/octomap_mapping.launch.py",
            "launch/jt128_3d_navigation.launch.py",
            "launch/nav2_3d.launch.py",
            "launch/explore.launch.py",
            "launch/scan_mission.launch.py",
            "launch/collision_monitor.launch.py",
            "launch/ekf.launch.py",
            "launch/ekf_local.launch.py",
            "launch/octomap_mapping.launch.py",
        ]),
        # Legacy 2D launch files moved to launch/legacy/ — available for fallback but not in default path
        ("share/" + package_name + "/launch/legacy", [
            "launch/legacy/slam.launch.py",
            "launch/legacy/mapping.launch.py",
            "launch/legacy/localization.launch.py",
            "launch/legacy/nav2.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dell",
    maintainer_email="dell@example.com",
    description="Launch entrypoints for the host-side A2 autonomy stack.",
    license="Apache-2.0",
)
