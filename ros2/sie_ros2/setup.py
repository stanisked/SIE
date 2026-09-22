from setuptools import find_packages, setup

package_name = "sie_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sie_pipeline.launch.py"]),
        ("share/" + package_name + "/config", ["config/sie_pipeline.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SIE contributors",
    maintainer_email="maintainers@example.invalid",
    description="Safe ROS 2 data-contract layer for SIE.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "perception_contract_node = sie_ros2.perception_node:main",
            "navigation_contract_node = sie_ros2.navigation_node:main",
            "supervisor_contract_node = sie_ros2.supervisor_node:main",
        ],
    },
)
