from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    node_namespace = LaunchConfiguration("node_namespace")
    waypoints_file = LaunchConfiguration("waypoints_file")
    vehicle_status_topic = LaunchConfiguration("vehicle_status_topic")
    vehicle_local_position_topic = LaunchConfiguration("vehicle_local_position_topic")
    offboard_control_mode_topic = LaunchConfiguration("offboard_control_mode_topic")
    trajectory_setpoint_topic = LaunchConfiguration("trajectory_setpoint_topic")
    vehicle_command_topic = LaunchConfiguration("vehicle_command_topic")
    current_index_topic = LaunchConfiguration("current_index_topic")
    target_system = LaunchConfiguration("target_system")
    target_component = LaunchConfiguration("target_component")
    source_system = LaunchConfiguration("source_system")
    source_component = LaunchConfiguration("source_component")
    vehicle_attitude_topic = LaunchConfiguration("vehicle_attitude_topic")
    vehicle_odometry_topic = LaunchConfiguration("vehicle_odometry_topic")
    gazebo_odometry_topic = LaunchConfiguration("gazebo_odometry_topic")
    waypoint_markers_topic = LaunchConfiguration("waypoint_markers_topic")
    waypoint_path_topic = LaunchConfiguration("waypoint_path_topic")
    vehicle_path_topic = LaunchConfiguration("vehicle_path_topic")
    enable_csv_logging = LaunchConfiguration("enable_csv_logging")
    log_root = LaunchConfiguration("log_root")
    run_id = LaunchConfiguration("run_id")
    rviz_frame_id = LaunchConfiguration("rviz_frame_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "node_namespace",
                default_value="",
                description="Optional ROS namespace for node names. Topics below are absolute by default.",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value="",
                description="Path to waypoint YAML file. Empty uses package default.",
            ),
            DeclareLaunchArgument(
                "vehicle_status_topic",
                default_value="/fmu/out/vehicle_status_v1",
                description="PX4 vehicle status topic. Current PX4 main publishes the _v1 topic.",
            ),
            DeclareLaunchArgument(
                "vehicle_local_position_topic",
                default_value="/fmu/out/vehicle_local_position_v1",
                description="PX4 local position topic. Current PX4 main publishes the _v1 topic.",
            ),
            DeclareLaunchArgument(
                "offboard_control_mode_topic",
                default_value="/fmu/in/offboard_control_mode",
                description="PX4 offboard control mode input topic.",
            ),
            DeclareLaunchArgument(
                "trajectory_setpoint_topic",
                default_value="/fmu/in/trajectory_setpoint",
                description="PX4 trajectory setpoint input topic.",
            ),
            DeclareLaunchArgument(
                "vehicle_command_topic",
                default_value="/fmu/in/vehicle_command",
                description="PX4 vehicle command input topic.",
            ),
            DeclareLaunchArgument(
                "current_index_topic",
                default_value="/waypoint_tracker/current_waypoint_index",
                description="Tracker current waypoint index topic.",
            ),
            DeclareLaunchArgument(
                "target_system",
                default_value="1",
                description="MAVLink system id used in VehicleCommand.target_system.",
            ),
            DeclareLaunchArgument(
                "target_component",
                default_value="1",
                description="MAVLink component id used in VehicleCommand.target_component.",
            ),
            DeclareLaunchArgument(
                "source_system",
                default_value="1",
                description="MAVLink source system id used in VehicleCommand.source_system.",
            ),
            DeclareLaunchArgument(
                "source_component",
                default_value="1",
                description="MAVLink source component id used in VehicleCommand.source_component.",
            ),
            DeclareLaunchArgument(
                "vehicle_attitude_topic",
                default_value="/fmu/out/vehicle_attitude",
                description="PX4 vehicle attitude topic used by trajectory_logger.",
            ),
            DeclareLaunchArgument(
                "vehicle_odometry_topic",
                default_value="/fmu/out/vehicle_odometry",
                description="PX4 vehicle odometry topic used by trajectory_logger.",
            ),
            DeclareLaunchArgument(
                "gazebo_odometry_topic",
                default_value="/model/x500_0/odometry_with_covariance",
                description="Gazebo ground truth odometry topic bridged to ROS 2.",
            ),
            DeclareLaunchArgument(
                "waypoint_markers_topic",
                default_value="/waypoint_markers",
                description="RViz waypoint marker topic.",
            ),
            DeclareLaunchArgument(
                "waypoint_path_topic",
                default_value="/waypoint_path",
                description="RViz planned waypoint path topic.",
            ),
            DeclareLaunchArgument(
                "vehicle_path_topic",
                default_value="/vehicle_path",
                description="RViz accumulated vehicle path topic.",
            ),
            DeclareLaunchArgument(
                "enable_csv_logging",
                default_value="true",
                description="Start ros_gz_bridge and trajectory_logger to write CSV trajectory logs.",
            ),
            DeclareLaunchArgument(
                "log_root",
                default_value="",
                description="Trajectory CSV log root. Empty uses ./log/trajectory_runs.",
            ),
            DeclareLaunchArgument(
                "run_id",
                default_value="",
                description="Trajectory CSV run directory name. Empty uses current timestamp.",
            ),
            DeclareLaunchArgument(
                "rviz_frame_id",
                default_value="map",
                description="RViz frame used by waypoint_visualizer. PX4 NED is converted to ROS ENU.",
            ),
            Node(
                package="uav_waypoint_tracking",
                executable="waypoint_tracker",
                name="waypoint_tracker",
                namespace=node_namespace,
                output="screen",
                parameters=[
                    {
                        "waypoints_file": waypoints_file,
                        "vehicle_status_topic": vehicle_status_topic,
                        "vehicle_local_position_topic": vehicle_local_position_topic,
                        "offboard_control_mode_topic": offboard_control_mode_topic,
                        "trajectory_setpoint_topic": trajectory_setpoint_topic,
                        "vehicle_command_topic": vehicle_command_topic,
                        "current_index_topic": current_index_topic,
                        "target_system": target_system,
                        "target_component": target_component,
                        "source_system": source_system,
                        "source_component": source_component,
                    }
                ],
            ),
            Node(
                package="uav_waypoint_tracking",
                executable="waypoint_visualizer",
                name="waypoint_visualizer",
                namespace=node_namespace,
                output="screen",
                parameters=[
                    {
                        "waypoints_file": waypoints_file,
                        "vehicle_local_position_topic": vehicle_local_position_topic,
                        "current_index_topic": current_index_topic,
                        "waypoint_markers_topic": waypoint_markers_topic,
                        "waypoint_path_topic": waypoint_path_topic,
                        "vehicle_path_topic": vehicle_path_topic,
                        "frame_id": rviz_frame_id,
                    }
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gazebo_truth_odometry_bridge",
                namespace=node_namespace,
                output="screen",
                condition=IfCondition(enable_csv_logging),
                arguments=[[gazebo_odometry_topic, "@nav_msgs/msg/Odometry@gz.msgs.OdometryWithCovariance"]],
            ),
            Node(
                package="uav_waypoint_tracking",
                executable="trajectory_logger",
                name="trajectory_logger",
                namespace=node_namespace,
                output="screen",
                condition=IfCondition(enable_csv_logging),
                parameters=[
                    {
                        "log_root": log_root,
                        "run_id": run_id,
                        "vehicle_local_position_topic": vehicle_local_position_topic,
                        "vehicle_attitude_topic": vehicle_attitude_topic,
                        "vehicle_odometry_topic": vehicle_odometry_topic,
                        "gazebo_odometry_topic": gazebo_odometry_topic,
                    }
                ],
            ),
        ]
    )
