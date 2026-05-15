from setuptools import setup

package_name = "sensor_sync"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dell",
    maintainer_email="dell@example.com",
    description="Host-side sensor freshness and skew monitoring.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sync_monitor = sensor_sync.sync_monitor:main",
            "pointcloud_guard = sensor_sync.pointcloud_guard:main",
            "pointcloud_relay = sensor_sync.pointcloud_relay:main",
            "pointcloud_to_laserscan = sensor_sync.pointcloud_to_laserscan:main",
        ],
    },
)
