from setuptools import setup

package_name = "safety_manager"

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
    description="Safety supervisor for lidar/state freshness and motion gating.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "safety_supervisor = safety_manager.safety_supervisor:main",
            "real_readiness_monitor = safety_manager.real_readiness_monitor:main",
        ],
    },
)
