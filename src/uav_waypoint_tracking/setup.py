from glob import glob
from setuptools import find_packages, setup

package_name = "uav_waypoint_tracking"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/config/wind_profiles", glob("config/wind_profiles/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="zk",
    maintainer_email="zk@localhost.localdomain",
    description="PX4 SITL ROS 2 offboard waypoint tracking simulation package.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "waypoint_tracker = uav_waypoint_tracking.waypoint_tracker:main",
            "waypoint_visualizer = uav_waypoint_tracking.waypoint_visualizer:main",
            "trajectory_logger = uav_waypoint_tracking.trajectory_logger:main",
            "yolo_tracker = uav_waypoint_tracking.yolo_tracker:main",
            "yolo_annotator = uav_waypoint_tracking.yolo_annotator:main",
            "gimbal_target_tracker = uav_waypoint_tracking.gimbal_target_tracker:main",
            "gimbal_performance_monitor = uav_waypoint_tracking.gimbal_performance_monitor:main",
        ],
    },
)
