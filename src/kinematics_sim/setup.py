from setuptools import setup

package_name = "kinematics_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/simulator.yaml"]),
        ("share/" + package_name + "/launch", ["launch/simulator.launch.py"]),
    ],
    install_requires=["setuptools", "numpy", "sensor_msgs_py"],
    zip_safe=True,
    maintainer="dell",
    maintainer_email="dell@example.com",
    description="Lightweight kinematics simulator for offline A2 testing.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "simulator_node = kinematics_sim.simulator_node:main",
        ],
    },
)
