from setuptools import setup

package_name = "nav_health_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/nav_health_monitor.yaml"]),
        ("share/" + package_name + "/launch", ["launch/nav_health_monitor.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dell",
    maintainer_email="dell@example.com",
    description="Navigation health monitoring and degradation control based on /diagnostics_agg.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "nav_health_monitor = nav_health_monitor.nav_health_monitor:main",
        ],
    },
)
