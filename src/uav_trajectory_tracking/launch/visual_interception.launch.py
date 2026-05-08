from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    node_namespace = LaunchConfiguration("node_namespace")

    vehicle_status_topic = LaunchConfiguration("vehicle_status_topic")
    vehicle_local_position_topic = LaunchConfiguration("vehicle_local_position_topic")
    vehicle_attitude_topic = LaunchConfiguration("vehicle_attitude_topic")
    offboard_control_mode_topic = LaunchConfiguration("offboard_control_mode_topic")
    trajectory_setpoint_topic = LaunchConfiguration("trajectory_setpoint_topic")
    vehicle_command_topic = LaunchConfiguration("vehicle_command_topic")
    vehicle_command_ack_topic = LaunchConfiguration("vehicle_command_ack_topic")
    target_system = LaunchConfiguration("target_system")
    target_component = LaunchConfiguration("target_component")
    source_system = LaunchConfiguration("source_system")
    source_component = LaunchConfiguration("source_component")

    enable_camera_bridge = LaunchConfiguration("enable_camera_bridge")
    camera_image_bridge_qos = LaunchConfiguration("camera_image_bridge_qos")
    camera_gazebo_topic = LaunchConfiguration("camera_gazebo_topic")
    camera_image_topic = LaunchConfiguration("camera_image_topic")
    camera_info_gazebo_topic = LaunchConfiguration("camera_info_gazebo_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    enable_yolo_tracking = LaunchConfiguration("enable_yolo_tracking")
    enable_yolo_annotation = LaunchConfiguration("enable_yolo_annotation")
    yolo_tracking_config_file = LaunchConfiguration("yolo_tracking_config_file")
    yolo_weights_path = LaunchConfiguration("yolo_weights_path")
    yolo_tracks_topic = LaunchConfiguration("yolo_tracks_topic")
    yolo_tracks_annotated_image_topic = LaunchConfiguration(
        "yolo_tracks_annotated_image_topic"
    )
    yolo_annotation_max_publish_hz = LaunchConfiguration(
        "yolo_annotation_max_publish_hz"
    )

    enable_gimbal_tracking = LaunchConfiguration("enable_gimbal_tracking")
    gimbal_config_file = LaunchConfiguration("gimbal_config_file")
    gimbal_input_topic = LaunchConfiguration("gimbal_input_topic")
    gimbal_joint_state_gazebo_topic = LaunchConfiguration(
        "gimbal_joint_state_gazebo_topic"
    )
    gimbal_joint_state_topic = LaunchConfiguration("gimbal_joint_state_topic")
    gimbal_set_attitude_topic = LaunchConfiguration("gimbal_set_attitude_topic")
    gimbal_tracking_active_topic = LaunchConfiguration("gimbal_tracking_active_topic")
    enable_gimbal_performance_monitor = LaunchConfiguration(
        "enable_gimbal_performance_monitor"
    )
    gimbal_performance_metrics_topic = LaunchConfiguration(
        "gimbal_performance_metrics_topic"
    )

    visual_interception_config_file = LaunchConfiguration(
        "visual_interception_config_file"
    )
    visual_interception_diagnostics_topic = LaunchConfiguration(
        "visual_interception_diagnostics_topic"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "node_namespace",
                default_value="",
                description="Optional ROS namespace for interception nodes.",
            ),
            DeclareLaunchArgument(
                "vehicle_status_topic",
                default_value="/fmu/out/vehicle_status_v4",
                description="Host PX4 vehicle status topic.",
            ),
            DeclareLaunchArgument(
                "vehicle_local_position_topic",
                default_value="/fmu/out/vehicle_local_position_v1",
                description="Host PX4 local position topic.",
            ),
            DeclareLaunchArgument(
                "vehicle_attitude_topic",
                default_value="/fmu/out/vehicle_attitude",
                description="Host PX4 attitude topic.",
            ),
            DeclareLaunchArgument(
                "offboard_control_mode_topic",
                default_value="/fmu/in/offboard_control_mode",
                description="Host PX4 offboard control mode input topic.",
            ),
            DeclareLaunchArgument(
                "trajectory_setpoint_topic",
                default_value="/fmu/in/trajectory_setpoint",
                description="Host PX4 trajectory setpoint input topic.",
            ),
            DeclareLaunchArgument(
                "vehicle_command_topic",
                default_value="/fmu/in/vehicle_command",
                description="Host PX4 vehicle command input topic.",
            ),
            DeclareLaunchArgument(
                "vehicle_command_ack_topic",
                default_value="/fmu/out/vehicle_command_ack",
                description="Host PX4 vehicle command acknowledgement topic.",
            ),
            DeclareLaunchArgument(
                "target_system",
                default_value="1",
                description="MAVLink system id used in host VehicleCommand.target_system.",
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
                "enable_camera_bridge",
                default_value="true",
                description="Bridge the host gimbal camera image and CameraInfo to ROS 2.",
            ),
            DeclareLaunchArgument(
                "camera_image_bridge_qos",
                default_value="default",
                description="QoS profile for ros_gz_image image_bridge.",
            ),
            DeclareLaunchArgument(
                "camera_gazebo_topic",
                default_value="/world/trajectory_tracking/model/x500_0/link/camera_link/sensor/camera/image",
                description="Gazebo image topic produced by the host gimbal camera.",
            ),
            DeclareLaunchArgument(
                "camera_image_topic",
                default_value="/x500_0/camera/image_raw",
                description="ROS 2 image topic for the host camera.",
            ),
            DeclareLaunchArgument(
                "camera_info_gazebo_topic",
                default_value="/world/trajectory_tracking/model/x500_0/link/camera_link/sensor/camera/camera_info",
                description="Gazebo CameraInfo topic produced by the host gimbal camera.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/x500_0/camera/camera_info",
                description="ROS 2 CameraInfo topic for the host camera.",
            ),
            DeclareLaunchArgument(
                "enable_yolo_tracking",
                default_value="true",
                description="Start YOLO tracking for target observations.",
            ),
            DeclareLaunchArgument(
                "enable_yolo_annotation",
                default_value="false",
                description="Start optional annotated target image publisher.",
            ),
            DeclareLaunchArgument(
                "yolo_tracking_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("uav_trajectory_tracking"),
                        "config",
                        "yolo_tracking.yaml",
                    ]
                ),
                description="YAML parameter file for yolo_tracker.",
            ),
            DeclareLaunchArgument(
                "yolo_weights_path",
                default_value="yolov8s.pt",
                description="YOLO weights used by yolo_tracker.",
            ),
            DeclareLaunchArgument(
                "yolo_tracks_topic",
                default_value="/x500_0/yolo/tracks",
                description="YOLO tracker Detection2DArray output topic.",
            ),
            DeclareLaunchArgument(
                "yolo_tracks_annotated_image_topic",
                default_value="/x500_0/yolo/tracks_image",
                description="YOLO annotated image output topic.",
            ),
            DeclareLaunchArgument(
                "yolo_annotation_max_publish_hz",
                default_value="15.0",
                description="Maximum publish rate for the optional annotated image topic.",
            ),
            DeclareLaunchArgument(
                "enable_gimbal_tracking",
                default_value="true",
                description="Start visual-servo gimbal tracking.",
            ),
            DeclareLaunchArgument(
                "gimbal_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("uav_trajectory_tracking"),
                        "config",
                        "gimbal_tracking.yaml",
                    ]
                ),
                description="YAML parameter file for gimbal_target_tracker.",
            ),
            DeclareLaunchArgument(
                "gimbal_input_topic",
                default_value="/x500_0/yolo/tracks",
                description="Detection2DArray topic consumed by gimbal_target_tracker.",
            ),
            DeclareLaunchArgument(
                "gimbal_joint_state_gazebo_topic",
                default_value="/world/trajectory_tracking/model/x500_0/joint_state",
                description="Gazebo joint state topic containing host gimbal joints.",
            ),
            DeclareLaunchArgument(
                "gimbal_joint_state_topic",
                default_value="/x500_0/gimbal/joint_states",
                description="ROS 2 JointState topic consumed by interception and gimbal nodes.",
            ),
            DeclareLaunchArgument(
                "gimbal_set_attitude_topic",
                default_value="/fmu/in/gimbal_manager_set_attitude",
                description="PX4 gimbal manager attitude setpoint input topic.",
            ),
            DeclareLaunchArgument(
                "gimbal_tracking_active_topic",
                default_value="/x500_0/gimbal_target_tracker/tracking_active",
                description="Gimbal target tracker active-state topic.",
            ),
            DeclareLaunchArgument(
                "enable_gimbal_performance_monitor",
                default_value="true",
                description="Start gimbal visual-servo performance monitor.",
            ),
            DeclareLaunchArgument(
                "gimbal_performance_metrics_topic",
                default_value="/x500_0/gimbal_performance/metrics",
                description="DiagnosticArray topic with gimbal performance metrics.",
            ),
            DeclareLaunchArgument(
                "visual_interception_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("uav_trajectory_tracking"),
                        "config",
                        "visual_interception.yaml",
                    ]
                ),
                description="YAML parameter file for visual_pursuit_interceptor.",
            ),
            DeclareLaunchArgument(
                "visual_interception_diagnostics_topic",
                default_value="/x500_0/visual_pursuit_interceptor/diagnostics",
                description="DiagnosticArray topic for visual pursuit interception.",
            ),
            Node(
                package="ros_gz_image",
                executable="image_bridge",
                name="x500_0_camera_image_bridge",
                namespace=node_namespace,
                output="screen",
                condition=IfCondition(enable_camera_bridge),
                arguments=[camera_gazebo_topic],
                parameters=[{"qos": camera_image_bridge_qos}],
                remappings=[(camera_gazebo_topic, camera_image_topic)],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="x500_0_camera_info_bridge",
                namespace=node_namespace,
                output="screen",
                condition=IfCondition(enable_camera_bridge),
                arguments=[
                    [
                        camera_info_gazebo_topic,
                        "@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                    ]
                ],
                remappings=[(camera_info_gazebo_topic, camera_info_topic)],
            ),
            Node(
                package="uav_trajectory_tracking",
                executable="yolo_tracker",
                name="yolo_tracker",
                namespace=node_namespace,
                output="screen",
                condition=IfCondition(enable_yolo_tracking),
                parameters=[
                    yolo_tracking_config_file,
                    {
                        "weights_path": yolo_weights_path,
                        "image_topic": camera_image_topic,
                        "tracks_topic": yolo_tracks_topic,
                    },
                ],
            ),
            Node(
                package="uav_trajectory_tracking",
                executable="yolo_annotator",
                name="yolo_annotator",
                namespace=node_namespace,
                output="screen",
                condition=IfCondition(enable_yolo_annotation),
                parameters=[
                    {
                        "image_topic": camera_image_topic,
                        "tracks_topic": yolo_tracks_topic,
                        "annotated_image_topic": yolo_tracks_annotated_image_topic,
                        "max_publish_hz": ParameterValue(
                            yolo_annotation_max_publish_hz,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="x500_0_gimbal_joint_state_bridge",
                namespace=node_namespace,
                output="screen",
                condition=IfCondition(enable_gimbal_tracking),
                arguments=[
                    [
                        gimbal_joint_state_gazebo_topic,
                        "@sensor_msgs/msg/JointState[gz.msgs.Model",
                    ]
                ],
                remappings=[
                    (gimbal_joint_state_gazebo_topic, gimbal_joint_state_topic)
                ],
            ),
            Node(
                package="uav_trajectory_tracking",
                executable="gimbal_target_tracker",
                name="gimbal_target_tracker",
                namespace=node_namespace,
                output="screen",
                condition=IfCondition(enable_gimbal_tracking),
                parameters=[
                    gimbal_config_file,
                    {
                        "detections_topic": gimbal_input_topic,
                        "camera_info_topic": camera_info_topic,
                        "gimbal_joint_state_topic": gimbal_joint_state_topic,
                        "gimbal_set_attitude_topic": gimbal_set_attitude_topic,
                        "vehicle_command_topic": vehicle_command_topic,
                        "vehicle_command_ack_topic": vehicle_command_ack_topic,
                        "target_system": ParameterValue(target_system, value_type=int),
                        "target_component": ParameterValue(target_component, value_type=int),
                        "source_system": ParameterValue(source_system, value_type=int),
                        "source_component": ParameterValue(source_component, value_type=int),
                    },
                ],
            ),
            Node(
                package="uav_trajectory_tracking",
                executable="visual_pursuit_interceptor",
                name="visual_pursuit_interceptor",
                namespace=node_namespace,
                output="screen",
                parameters=[
                    {
                        "config_file": visual_interception_config_file,
                        "vehicle_status_topic": vehicle_status_topic,
                        "vehicle_local_position_topic": vehicle_local_position_topic,
                        "vehicle_attitude_topic": vehicle_attitude_topic,
                        "gimbal_joint_state_topic": gimbal_joint_state_topic,
                        "tracking_active_topic": gimbal_tracking_active_topic,
                        "offboard_control_mode_topic": offboard_control_mode_topic,
                        "trajectory_setpoint_topic": trajectory_setpoint_topic,
                        "vehicle_command_topic": vehicle_command_topic,
                        "diagnostics_topic": visual_interception_diagnostics_topic,
                        "target_system": ParameterValue(target_system, value_type=int),
                        "target_component": ParameterValue(target_component, value_type=int),
                        "source_system": ParameterValue(source_system, value_type=int),
                        "source_component": ParameterValue(source_component, value_type=int),
                    }
                ],
            ),
            Node(
                package="uav_trajectory_tracking",
                executable="gimbal_performance_monitor",
                name="gimbal_performance_monitor",
                namespace=node_namespace,
                output="screen",
                condition=IfCondition(enable_gimbal_performance_monitor),
                parameters=[
                    {
                        "detections_topic": gimbal_input_topic,
                        "tracking_active_topic": gimbal_tracking_active_topic,
                        "camera_info_topic": camera_info_topic,
                        "metrics_topic": gimbal_performance_metrics_topic,
                    }
                ],
            ),
        ]
    )
