from setuptools import setup

package_name = "nav2_integration"

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
    description="Nav2-facing action bridge for A2 exploration and patrol goals.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "goal_bridge = nav2_integration.goal_bridge:main",
            "pose_goal_controller_3d = nav2_integration.pose_goal_controller_3d:main",
        ],
    },
)
