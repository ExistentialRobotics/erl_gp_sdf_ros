#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for ZED camera SDF mapping."""

    # Declare launch arguments
    zed_bag_dir_arg = DeclareLaunchArgument(
        "zed_bag_dir",
        default_value=os.path.join(os.path.expanduser("~"), "Data", "Zed", "zed_2025-06-09-15-10-37.ros2"),
        description="Path to ZED bag directory",
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
        default_value="0.8",
        description="Playback rate for rosbag",
    )
    play_rosbag_arg = DeclareLaunchArgument(
        "play_rosbag",
        default_value="true",
        description="Whether to play the rosbag file",
    )

    def create_sdf_mapping_node(context):
        """Create SDF mapping node based on surf_mapping_method."""
        precision = LaunchConfiguration("precision").perform(context)
        surf_mapping_method = LaunchConfiguration("surf_mapping_method").perform(context)

        # Get package share directory
        erl_gp_sdf_share = get_package_share_directory("erl_gp_sdf_ros")

        # Configuration files
        config_file = os.path.join(erl_gp_sdf_share, "config", "ros2", f"{surf_mapping_method}_{precision}_3d.yaml")

        if surf_mapping_method == "gp_occ":
            surf_mapping_config = os.path.join(erl_gp_sdf_share, "config", "zed", f"gp_occ_surf_mapping_{precision}.yaml")
            sdf_mapping_config = os.path.join(erl_gp_sdf_share, "config", "zed", f"gp_occ_sdf_mapping_{precision}.yaml")
            sensor_frame = "zed_left_camera_optical_frame"
            scan_topic = "/zed/zed_node/depth/depth_registered"
            scan_type = "sensor_msgs/msg/Image"
            convert_scan_to_points = False
            scan_stride = 1
        else:  # bayesian_hilbert
            surf_mapping_config = os.path.join(erl_gp_sdf_share, "config", "zed", f"bayesian_hilbert_surf_mapping_{precision}.yaml")
            sdf_mapping_config = os.path.join(erl_gp_sdf_share, "config", "zed", f"bayesian_hilbert_sdf_mapping_{precision}.yaml")
            sensor_frame = "zed_left_camera_frame"
            scan_topic = "/zed/zed_node/point_cloud/cloud_registered"
            scan_type = "sensor_msgs/msg/PointCloud2"
            convert_scan_to_points = True
            scan_stride = 2

        scan_frame_config = os.path.join(erl_gp_sdf_share, "config", "zed", "depth_frame.yaml")

        parameters = {
            "map_dim": 3,
            "double_precision": precision == "double",
            "surface_mapping_setting_file": surf_mapping_config,
            "sdf_mapping_setting_file": sdf_mapping_config,
            "use_odom": False,
            "odom_topic": "/zed/zed_node/odom",
            "odom_msg_type": "nav_msgs/msg/Odometry",
            "sensor_frame": sensor_frame,
            "scan_topic": scan_topic,
            "scan_type": scan_type,
            "convert_scan_to_points": convert_scan_to_points,
            "scan_frame_setting_file": scan_frame_config,
            "scan_stride": scan_stride,
            "scan_in_local_frame": True,
            "depth_scale": 1.0,
            "publish_tree": True,
        }

        return [
            Node(
                package="erl_gp_sdf_ros",
                executable="sdf_mapping_node",
                name="sdf_mapping_node",
                output="screen",
                parameters=[
                    ParameterFile(config_file, allow_substs=True),
                    parameters,
                ],
            )
        ]

    # SDF mapping node (conditional based on surf_mapping_method)
    sdf_mapping_node_opaque = OpaqueFunction(function=create_sdf_mapping_node)

    # SDF visualization node (follow camera)
    sdf_visualization_node_follow = Node(
        package="erl_gp_sdf_ros",
        executable="sdf_visualization_node",
        name="sdf_visualization_node_follow",
        namespace="sdf_local",
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
                "publish_rate": 10.0,
                "attached_to_frame": True,
                "world_frame": "map",
                "attached_frame": "zed_left_camera_frame",
                "service_name": "/sdf_query",
                "map_topic_name": "sdf_grid_map",
                "point_cloud_topic_name": "sdf_point_cloud",
            }
        ],
        condition=IfCondition(LaunchConfiguration("visualize_sdf")),
    )

    # SDF visualization node (global view)
    sdf_visualization_node_global = Node(
        package="erl_gp_sdf_ros",
        executable="sdf_visualization_node",
        name="sdf_visualization_node_global",
        namespace="sdf_global",
        output="screen",
        parameters=[
            {
                "resolution": 0.1,
                "x_cells": 101,
                "y_cells": 101,
                "x": 0.0,
                "y": 0.0,
                "z": -0.8,
                "publish_gradient": True,
                "publish_sdf_variance": True,
                "publish_gradient_variance": True,
                "publish_covariance": False,
                "publish_grid_map": True,
                "publish_point_cloud": True,
                "publish_rate": 10.0,
                "attached_to_frame": False,
                "world_frame": "map",
                "attached_frame": "zed_left_camera_frame",
                "service_name": "/sdf_query",
                "map_topic_name": "sdf_grid_map",
                "point_cloud_topic_name": "sdf_point_cloud",
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
            PathJoinSubstitution([FindPackageShare("erl_gp_sdf_ros"), "rviz2", "zed.rviz"])
        ],
        condition=IfCondition(LaunchConfiguration("open_rviz")),
    )

    # Rosbag play process
    rosbag_play_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("zed_bag_dir"),
            "--rate",
            LaunchConfiguration("rosbag_play_rate"),
            "--clock"
        ],
        name="rosbag_play",
        output="screen",
        condition=IfCondition(LaunchConfiguration("play_rosbag"))
    )

    # ZED wrapper launch (when not playing rosbag)
    # Note: This assumes there's a ROS2 version of zed_wrapper launch file
    # You may need to adjust the package name and launch file name
    zed_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("zed_wrapper"),
            "launch",
            "zed_camera.launch.py",
        ]),
        launch_arguments={
            "camera_model": "zed"
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("play_rosbag"))
    )

    # RQT plot node (if needed)
    rqt_plot_node = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="rqt_plot",
        condition=IfCondition(LaunchConfiguration("open_rqt_plot")),
        arguments=["/update_time/temperature", "/query_time/temperature"],
    )

    return LaunchDescription([
        # Launch arguments
        zed_bag_dir_arg,
        precision_arg,
        surf_mapping_method_arg,
        visualize_sdf_arg,
        open_rviz_arg,
        open_rqt_plot_arg,
        rosbag_play_rate_arg,
        play_rosbag_arg,

        # Nodes and processes
        sdf_mapping_node_opaque,
        sdf_visualization_node_follow,
        sdf_visualization_node_global,
        rviz_node,
        rosbag_play_process,
        zed_launch,
        rqt_plot_node,
    ])
