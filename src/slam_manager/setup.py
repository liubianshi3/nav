from setuptools import setup

package_name = "slam_manager"

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
    description="Package container for external SLAM launch integration.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "slam_orchestrator = slam_manager.slam_orchestrator:main",
        ],
    },
)
