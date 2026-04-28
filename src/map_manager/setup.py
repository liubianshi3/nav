from setuptools import setup

package_name = "map_manager"

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
    description="Map save/load/version management for 3D mapping plus Nav2 projection workflows.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "map_manager_node = map_manager.map_manager_node:main",
            "mock_map_publisher = map_manager.mock_map_publisher:main",
            "native_map_relay = map_manager.native_map_relay:main",
            "occupancy_mapper = map_manager.occupancy_mapper:main",
        ],
    },
)
