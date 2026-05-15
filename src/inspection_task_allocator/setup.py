from setuptools import setup

package_name = "inspection_task_allocator"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "pandas"],
    zip_safe=True,
    maintainer="dell",
    maintainer_email="dell@example.com",
    description="Standalone inspection task allocation simulator with A* planning.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "inspection_task_demo = inspection_task_allocator.demo_simulation:main",
        ],
    },
)
