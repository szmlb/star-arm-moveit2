from glob import glob

from setuptools import find_packages, setup

package_name = "starai_teleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="szmlb",
    maintainer_email="s.akihiro312@gmail.com",
    description="Live ROS2 teleoperation bridge for StarAI leader/follower arms.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_bridge = starai_teleop.teleop_bridge:main",
            "go_to_rest = starai_teleop.go_to_rest:main",
        ],
    },
)
