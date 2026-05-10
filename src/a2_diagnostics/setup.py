from setuptools import setup

package_name = "a2_diagnostics"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/diagnostic_aggregator.yaml"]),
        ("share/" + package_name + "/launch", ["launch/diagnostics.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dell",
    maintainer_email="dell@example.com",
    description="Standard diagnostic_msgs bridge and aggregator for a2_system_ws.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "status_adapter = a2_diagnostics.status_adapter_node:main",
            "diagnostic_aggregator = a2_diagnostics.diagnostic_aggregator:main",
        ],
    },
)
