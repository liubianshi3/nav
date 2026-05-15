from setuptools import setup

package_name = "localization_manager"

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
    description="Localization readiness gate for Nav2 and motion control.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "localization_gate = localization_manager.localization_gate:main",
            "manual_localization_publisher = localization_manager.manual_localization_publisher:main",
            "pcd_relocalizer_3d = localization_manager.pcd_relocalizer_3d:main",
            "ndt_health_monitor = localization_manager.ndt_health_monitor:main",
        ],
    },
)
