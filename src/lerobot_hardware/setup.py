from setuptools import find_packages, setup

package_name = "lerobot_hardware"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/so101_physical.launch.py"]),
        (
            "share/" + package_name + "/config",
            ["config/hardware_params.yaml", "config/moveit_py_params.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="james",
    maintainer_email="james.watson-2@colorado.edu",
    description=(
        "Task-space command node (MoveIt 2 IK + motion planning) and physical "
        "joint driver node for controlling the real LeRobot SO-ARM101 arm."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pose_commander_node = lerobot_hardware.pose_commander_node:main",
            "hardware_driver_node = lerobot_hardware.hardware_driver_node:main",
        ],
    },
)
