from glob import glob

from setuptools import setup

package_name = "zg_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Raj Srivastava",
    description="Thin ROS 2 wrapper around the ZERO-GAP swarm simulator.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "swarm_sim = zg_bringup.swarm_sim_node:main",
            "gcs_monitor = zg_bringup.gcs_monitor_node:main",
        ],
    },
)
