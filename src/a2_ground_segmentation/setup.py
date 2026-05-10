from setuptools import setup

package_name = "a2_ground_segmentation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/ground_segmentation.yaml"]),
        ("share/" + package_name + "/launch", ["launch/ground_segmentation.launch.py"]),
    ],
    install_requires=["setuptools", "numpy", "sensor_msgs_py"],
    zip_safe=True,
    maintainer="dell",
    maintainer_email="dell@example.com",
    description="Ray-based ground/obstacle segmentation and 2.5D traversability mapping for A2.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ground_segmentation_node = a2_ground_segmentation.ground_segmentation_node:main",
        ],
    },
)
