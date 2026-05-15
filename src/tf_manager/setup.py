from setuptools import setup

package_name = "tf_manager"

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
    description="TF management package placeholder for static and semantic transforms.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "static_tf_manager = tf_manager.static_tf_manager:main",
        ],
    },
)
