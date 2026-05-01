from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
    enable_camera_bridge = LaunchConfiguration("enable_camera_bridge")
    enable_yolo_detection = LaunchConfiguration("enable_yolo_detection")
    camera_gazebo_topic = LaunchConfiguration("camera_gazebo_topic")
    camera_image_topic = LaunchConfiguration("camera_image_topic")
    yolo_weights_path = LaunchConfiguration("yolo_weights_path")
    yolo_detections_topic = LaunchConfiguration("yolo_detections_topic")
    yolo_annotated_image_topic = LaunchConfiguration("yolo_annotated_image_topic")
    yolo_confidence_threshold = LaunchConfiguration("yolo_confidence_threshold")
    yolo_iou_threshold = LaunchConfiguration("yolo_iou_threshold")
    yolo_image_size = LaunchConfiguration("yolo_image_size")
    yolo_max_detections = LaunchConfiguration("yolo_max_detections")
    yolo_classes = LaunchConfiguration("yolo_classes")
    yolo_device = LaunchConfiguration("yolo_device")
    enable_gimbal_tracking = LaunchConfiguration("enable_gimbal_tracking")
    gimbal_config_file = LaunchConfiguration("gimbal_config_file")
    gimbal_target_class_id = LaunchConfiguration("gimbal_target_class_id")
    gimbal_min_score = LaunchConfiguration("gimbal_min_score")
    gimbal_yaw_rate_gain_deg_s = LaunchConfiguration("gimbal_yaw_rate_gain_deg_s")
    gimbal_pitch_rate_gain_deg_s = LaunchConfiguration("gimbal_pitch_rate_gain_deg_s")
    gimbal_yaw_error_sign = LaunchConfiguration("gimbal_yaw_error_sign")
    gimbal_pitch_error_sign = LaunchConfiguration("gimbal_pitch_error_sign")

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
            DeclareLaunchArgument(
                "enable_camera_bridge",
                default_value="false",
                description="Bridge the x500_0 Gazebo gimbal camera image to ROS 2.",
            ),
            DeclareLaunchArgument(
                "enable_yolo_detection",
                default_value="false",
                description="Start YOLOv8 detection on the bridged x500_0 camera image.",
            ),
            DeclareLaunchArgument(
                "camera_gazebo_topic",
                default_value="/world/waypoint_tracking/model/x500_0/link/camera_link/sensor/camera/image",
                description="Gazebo image topic produced by the x500_0 gimbal camera.",
            ),
            DeclareLaunchArgument(
                "camera_image_topic",
                default_value="/x500_0/camera/image_raw",
                description="ROS 2 image topic for the bridged x500_0 camera image.",
            ),
            DeclareLaunchArgument(
                "yolo_weights_path",
                default_value="/home/zk/uav_waypoint_tracking_sim/yolov8s.pt",
                description="YOLOv8 weights used by yolo_detector.",
            ),
            DeclareLaunchArgument(
                "yolo_detections_topic",
                default_value="/x500_0/yolo/detections",
                description="YOLO vision_msgs/Detection2DArray output topic.",
            ),
            DeclareLaunchArgument(
                "yolo_annotated_image_topic",
                default_value="/x500_0/yolo/image_annotated",
                description="YOLO annotated image output topic.",
            ),
            DeclareLaunchArgument(
                "yolo_confidence_threshold",
                default_value="0.25",
                description="YOLO confidence threshold.",
            ),
            DeclareLaunchArgument(
                "yolo_iou_threshold",
                default_value="0.45",
                description="YOLO non-max suppression IoU threshold.",
            ),
            DeclareLaunchArgument(
                "yolo_image_size",
                default_value="640",
                description="YOLO inference image size.",
            ),
            DeclareLaunchArgument(
                "yolo_max_detections",
                default_value="100",
                description="Maximum YOLO detections per image.",
            ),
            DeclareLaunchArgument(
                "yolo_classes",
                default_value="",
                description="Optional comma-separated YOLO class ids. Empty detects all classes.",
            ),
            DeclareLaunchArgument(
                "yolo_device",
                default_value="",
                description="Optional YOLO device, for example cpu, 0, or cuda:0. Empty lets ultralytics choose.",
            ),
            DeclareLaunchArgument(
                "enable_gimbal_tracking",
                default_value="false",
                description="Start visual-servo gimbal tracking from YOLO detections.",
            ),
            DeclareLaunchArgument(
                "gimbal_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("uav_waypoint_tracking"),
                        "config",
                        "gimbal_tracking.yaml",
                    ]
                ),
                description="YAML parameter file for gimbal_target_tracker.",
            ),
            DeclareLaunchArgument(
                "gimbal_target_class_id",
                default_value="",
                description="Optional class id/name to track. Empty tracks the highest-score detection.",
            ),
            DeclareLaunchArgument(
                "gimbal_min_score",
                default_value="0.25",
                description="Minimum detection confidence used by gimbal tracking.",
            ),
            DeclareLaunchArgument(
                "gimbal_yaw_rate_gain_deg_s",
                default_value="45.0",
                description="Yaw rate gain in deg/s for full-scale horizontal image error.",
            ),
            DeclareLaunchArgument(
                "gimbal_pitch_rate_gain_deg_s",
                default_value="35.0",
                description="Pitch rate gain in deg/s for full-scale vertical image error.",
            ),
            DeclareLaunchArgument(
                "gimbal_yaw_error_sign",
                default_value="1.0",
                description="Yaw error sign. Flip to -1.0 if horizontal tracking moves away from target.",
            ),
            DeclareLaunchArgument(
                "gimbal_pitch_error_sign",
                default_value="-1.0",
                description="Pitch error sign. Flip to 1.0 if vertical tracking moves away from target.",
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
            Node(
                package="ros_gz_image",
                executable="image_bridge",
                name="x500_0_camera_image_bridge",
                namespace=node_namespace,
                output="screen",
                condition=IfCondition(enable_camera_bridge),
                arguments=[camera_gazebo_topic],
                remappings=[(camera_gazebo_topic, camera_image_topic)],
            ),
            Node(
                package="uav_waypoint_tracking",
                executable="yolo_detector",
                name="yolo_detector",
                namespace=node_namespace,
                output="screen",
                condition=IfCondition(enable_yolo_detection),
                parameters=[
                    {
                        "weights_path": yolo_weights_path,
                        "image_topic": camera_image_topic,
                        "detections_topic": yolo_detections_topic,
                        "annotated_image_topic": yolo_annotated_image_topic,
                        "confidence_threshold": ParameterValue(
                            yolo_confidence_threshold,
                            value_type=float,
                        ),
                        "iou_threshold": ParameterValue(
                            yolo_iou_threshold,
                            value_type=float,
                        ),
                        "image_size": ParameterValue(yolo_image_size, value_type=int),
                        "max_detections": ParameterValue(
                            yolo_max_detections,
                            value_type=int,
                        ),
                        "classes": yolo_classes,
                        "device": yolo_device,
                    }
                ],
            ),
            Node(
                package="uav_waypoint_tracking",
                executable="gimbal_target_tracker",
                name="gimbal_target_tracker",
                namespace=node_namespace,
                output="screen",
                condition=IfCondition(enable_gimbal_tracking),
                parameters=[
                    gimbal_config_file,
                    {
                        "detections_topic": yolo_detections_topic,
                        "image_topic": camera_image_topic,
                        "vehicle_command_topic": vehicle_command_topic,
                        "target_class_id": gimbal_target_class_id,
                        "min_score": ParameterValue(gimbal_min_score, value_type=float),
                        "yaw_rate_gain_deg_s": ParameterValue(
                            gimbal_yaw_rate_gain_deg_s,
                            value_type=float,
                        ),
                        "pitch_rate_gain_deg_s": ParameterValue(
                            gimbal_pitch_rate_gain_deg_s,
                            value_type=float,
                        ),
                        "yaw_error_sign": ParameterValue(
                            gimbal_yaw_error_sign,
                            value_type=float,
                        ),
                        "pitch_error_sign": ParameterValue(
                            gimbal_pitch_error_sign,
                            value_type=float,
                        ),
                        "target_system": ParameterValue(target_system, value_type=int),
                        "target_component": ParameterValue(
                            target_component,
                            value_type=int,
                        ),
                        "source_system": ParameterValue(source_system, value_type=int),
                        "source_component": ParameterValue(
                            source_component,
                            value_type=int,
                        ),
                    },
                ],
            ),
        ]
    )
