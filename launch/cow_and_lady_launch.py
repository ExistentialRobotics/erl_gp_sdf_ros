#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for cow_and_lady dataset SDF mapping."""

    # Declare launch arguments
    cow_and_lady_bag_arg = DeclareLaunchArgument(
        "cow_and_lady_bag",
        default_value=os.path.join(os.path.expanduser("~"), "Data", "CowAndLady", "ros2_bag"),
        description="Path to cow_and_lady.bag for testing",
    )
    gt_point_cloud_file_arg = DeclareLaunchArgument(
        "gt_point_cloud_file",
        default_value=os.path.join(os.path.expanduser("~"), "Data", "CowAndLady", "cow_and_lady_gt.ply"),
        description="Path to ground truth point cloud for visualization",
    )

    precision_arg = DeclareLaunchArgument(
        "precision",
        default_value="float",
        description="Options: float, double",
    )
    surf_mapping_method_arg = DeclareLaunchArgument(
        "surf_mapping_method",
        default_value="bayesian_hilbert",
        description="Options: gp_occ, bayesian_hilbert",
    )
    visualize_sdf_arg = DeclareLaunchArgument(
        "visualize_sdf",
        default_value="true",
        description="Whether to launch the sdf visualization node",
    )
    open_rviz_arg = DeclareLaunchArgument(
        "open_rviz",
        default_value="true",
        description="Whether to launch rviz for visualization",
    )
    open_rqt_plot_arg = DeclareLaunchArgument(
        "open_rqt_plot",
        default_value="true",
        description="Whether to launch rqt_plot for performance analysis",
    )
    rosbag_play_rate_arg = DeclareLaunchArgument(
        "rosbag_play_rate",
        default_value="2.0",
        description="Playback rate for rosbag",
    )

    # Rosbag player process
    # Note: This bag does not have /clock or tf data, so not using --clock
    # ROS bag play
    cmd = [
        "ros2",
        "bag",
        "play",
        LaunchConfiguration("cow_and_lady_bag"),
        "--rate",
        LaunchConfiguration("rosbag_play_rate"),
        "--delay",
        "2",
        "--start-offset",
        "5",
        "--clock",
    ]
    # LTS: humble (2027 EOL), jazzy (2029 EOL)
    # humble does not support --playback-duration
    if os.environ.get("ROS_DISTRO") != "humble":
        cmd.append("--playback-duration")
        cmd.append("128")
    rosbag_play_process = ExecuteProcess(cmd=cmd, name="rosbag_play", output="screen")

    # Static transform publishers
    map_odom_transformer = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_transformer",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    map_vicon_transformer = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_vicon_transformer",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            "map",
            "--child-frame-id",
            "vicon",
        ],
    )

    kinect_optical_transformer = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="kinect_optical_transformer",
        arguments=[
            "--x",
            "0.00114049",
            "--y",
            "0.0450936",
            "--z",
            "0.0430765",
            "--qx",
            "0.0924132",
            "--qy",
            "0.0976455",
            "--qz",
            "0.0702949",
            "--qw",
            "0.988425",
            "--frame-id",
            "kinect",
            "--child-frame-id",
            "camera_rgb_optical_frame",
        ],
    )

    optical_camera_transformer = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="optical_camera_transformer",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0.5",
            "--qy",
            "0.5",
            "--qz",
            "-0.5",
            "--qw",
            "0.5",
            "--frame-id",
            "camera_rgb_optical_frame",
            "--child-frame-id",
            "camera_frame",
        ],
    )

    # Transform to TF node
    transform_to_tf_node = Node(
        package="erl_common_ros",
        executable="transform_to_tf_node",
        name="transform_to_tf_node",
        output="screen",
        parameters=[{"transform_topic": "/kinect/vrpn_client/estimated_transform"}],
    )

    # Ground truth point cloud node
    gt_point_cloud_node = Node(
        package="erl_geometry_ros",
        executable="point_cloud_node",
        name="gt_point_cloud_node",
        output="screen",
        parameters=[
            {"frame_id": "map", "point_cloud_file": LaunchConfiguration("gt_point_cloud_file"), "publish_colors": True}
        ],
    )

    # SDF mapping node with dynamic parameter loading
    def create_sdf_mapping_node(context):
        """Create SDF mapping node with dynamic parameter loading."""
        precision = context.launch_configurations["precision"]
        surf_mapping_method = context.launch_configurations["surf_mapping_method"]

        # Get package share directories
        erl_gp_sdf_ros_share = get_package_share_directory("erl_gp_sdf_ros")
        erl_gp_sdf_share = get_package_share_directory("erl_gp_sdf")

        # Build config file paths
        config_file = os.path.join(erl_gp_sdf_ros_share, "config", "ros2", f"{surf_mapping_method}_{precision}_3d.yaml")
        surf_mapping_config = os.path.join(
            erl_gp_sdf_share, "config", "cow_and_lady", f"{surf_mapping_method}_surf_mapping_{precision}.yaml"
        )
        sdf_mapping_config = os.path.join(
            erl_gp_sdf_share, "config", "cow_and_lady", f"{surf_mapping_method}_sdf_mapping_{precision}.yaml"
        )
        scan_frame_config = os.path.join(erl_gp_sdf_share, "config", "cow_and_lady", "depth_frame.yaml")

        return [
            Node(
                package="erl_gp_sdf_ros",
                executable="sdf_mapping_node",
                name="sdf_mapping_node",
                output="screen",
                parameters=[
                    ParameterFile(config_file, allow_substs=True),
                    {
                        "map_dim": 3,
                        "double_precision": precision == "double",
                        "surface_mapping_setting_file": surf_mapping_config,
                        "sdf_mapping_setting_file": sdf_mapping_config,
                        "use_odom": False,
                        "odom_topic": "/kinect/vrpn_client/estimated_transform",
                        "odom_msg_type": "geometry_msgs/msg/TransformStamped",
                        "sensor_frame": "camera_rgb_optical_frame",
                        "scan_topic.path": "/camera/depth_registered/points",
                        "scan_type": "sensor_msgs/msg/PointCloud2",
                        "scan_frame_setting_file": scan_frame_config,
                        "scan_stride": 2,
                        "scan_in_local_frame": True,
                        "publish_tree": False,
                    },
                ],
            )
        ]

    # SDF visualization node
    sdf_visualization_node = Node(
        package="erl_gp_sdf_ros",
        executable="sdf_visualization_node",
        name="sdf_visualization_node",
        output="screen",
        parameters=[
            {
                "resolution": 0.02,
                "x_cells": 101,
                "y_cells": 101,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "publish_gradient": True,
                "publish_sdf_variance": True,
                "publish_gradient_variance": True,
                "publish_covariance": False,
                "publish_grid_map": True,
                "publish_point_cloud": True,
                "publish_rate": 5.0,
                "attached_to_frame": True,
                "world_frame": "map",
                "attached_frame": "camera_frame",
                "sdf_query_service.path": "sdf_query",
                "map_topic.path": "sdf_grid_map",
                "point_cloud_topic.path": "sdf_point_cloud",
            }
        ],
        condition=IfCondition(LaunchConfiguration("visualize_sdf")),
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("erl_gp_sdf_ros"), "rviz2", "cow_and_lady.rviz"]),
        ],
        condition=IfCondition(LaunchConfiguration("open_rviz")),
    )

    # rqt_plot node
    rqt_plot_node = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="rqt_plot",
        arguments=["/update_time/data", "/query_time/data"],
        condition=IfCondition(LaunchConfiguration("open_rqt_plot")),
    )

    # Create SDF mapping node using OpaqueFunction for dynamic parameter evaluation
    sdf_mapping_node = OpaqueFunction(function=create_sdf_mapping_node)

    return LaunchDescription(
        [
            # Launch arguments
            cow_and_lady_bag_arg,
            gt_point_cloud_file_arg,
            precision_arg,
            surf_mapping_method_arg,
            visualize_sdf_arg,
            open_rviz_arg,
            open_rqt_plot_arg,
            rosbag_play_rate_arg,
            # Static transforms
            map_odom_transformer,
            map_vicon_transformer,
            kinect_optical_transformer,
            optical_camera_transformer,
            # Nodes
            transform_to_tf_node,
            gt_point_cloud_node,
            sdf_mapping_node,
            sdf_visualization_node,
            rviz_node,
            rqt_plot_node,
            # Rosbag playback
            rosbag_play_process,
        ]
    )
