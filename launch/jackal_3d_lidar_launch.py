#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for jackal 3D LiDAR SDF mapping."""

    # Declare launch arguments
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
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="jackal2",
        description="Robot name for topic namespacing",
    )
    rosbag_dir_arg = DeclareLaunchArgument(
        "rosbag_dir",
        default_value=os.path.join(os.environ["HOME"], "Data", "jackal_lidar_3d", "jackal2_map_Lab_ros2"),
        description="Path to the rosbag directory to play",
    )
    rosbag_play_rate_arg = DeclareLaunchArgument(
        "rosbag_play_rate",
        default_value="0.25",
        description="Rate at which to play the rosbag file",
    )
    play_rosbag_arg = DeclareLaunchArgument(
        "play_rosbag",
        default_value="true",
        description="Whether to play the rosbag file",
    )

    # Static transform publisher (map -> odom)
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

    def create_sdf_mapping_node(context):
        """Create SDF mapping node with dynamic parameter loading."""
        precision = context.launch_configurations["precision"]
        surf_mapping_method = context.launch_configurations["surf_mapping_method"]
        robot_name = context.launch_configurations["robot_name"]

        # Get package share directory
        pkg_share = get_package_share_directory("erl_gp_sdf_ros")

        # Build config file paths
        config_file_1 = os.path.join(pkg_share, "config", "ros2", f"{surf_mapping_method}_{precision}_3d.yaml")
        config_file_2 = os.path.join(pkg_share, "config", "ros2", f"use_lidar_frame_3d_{precision}.yaml")
        surf_mapping_config = os.path.join(
            pkg_share, "config", "jackal_3d", f"{surf_mapping_method}_surf_mapping_lidar_{precision}.yaml"
        )
        sdf_mapping_config = os.path.join(
            pkg_share, "config", "jackal_3d", f"{surf_mapping_method}_sdf_mapping_lidar_{precision}.yaml"
        )
        scan_frame_config = os.path.join(pkg_share, "config", "jackal_3d", "lidar_frame_3d_360.yaml")

        return [
            Node(
                package="erl_gp_sdf_ros",
                executable="sdf_mapping_node",
                name="sdf_mapping_node",
                output="screen",
                parameters=[
                    ParameterFile(config_file_1, allow_substs=True),
                    ParameterFile(config_file_2, allow_substs=True),
                    {
                        "map_dim": 3,
                        "double_precision": precision == "double",
                        "surface_mapping_setting_file": surf_mapping_config,
                        "sdf_mapping_setting_file": sdf_mapping_config,
                        "use_odom": False,
                        "odom_topic": f"/{robot_name}/dlo/odom_node/odom",
                        "odom_msg_type": "nav_msgs/msg/Odometry",
                        "world_frame": "map",
                        "sensor_frame": f"{robot_name}/os_sensor",
                        "scan_topic": f"/{robot_name}/ouster/points",
                        "scan_type": "sensor_msgs/msg/PointCloud2",
                        "scan_frame_setting_file": scan_frame_config,
                        "scan_stride": 1,
                        "convert_scan_to_points": surf_mapping_method == "bayesian_hilbert",
                        "scan_in_local_frame": True,
                        "depth_scale": 0.001,
                        "publish_tree": True,
                    },
                ],
            )
        ]

    def create_sdf_visualization_node(context):
        """Create SDF visualization node."""
        robot_name = context.launch_configurations["robot_name"]

        return [
            Node(
                package="erl_gp_sdf_ros",
                executable="sdf_visualization_node",
                name="sdf_visualization_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("visualize_sdf")),
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
                        "attached_frame": f"{robot_name}/os_sensor",
                        "service_name": "sdf_query",
                        "map_topic_name": "sdf_grid_map",
                        "point_cloud_topic_name": "sdf_point_cloud",
                    }
                ],
            )
        ]

    def create_rviz_node(context):
        """Create RViz node."""
        robot_name = context.launch_configurations["robot_name"]
        pkg_share = get_package_share_directory("erl_gp_sdf_ros")
        rviz_config = os.path.join(pkg_share, "rviz2", f"{robot_name}_3d_lidar.rviz")

        return [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                condition=IfCondition(LaunchConfiguration("open_rviz")),
                arguments=["-d", rviz_config],
            )
        ]

    rqt_plot_node = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="rqt_plot",
        condition=IfCondition(LaunchConfiguration("open_rqt_plot")),
        arguments=["/update_time/temperature", "/query_time/temperature"],
    )

    # Create nodes using OpaqueFunction for dynamic parameter evaluation
    sdf_mapping_node = OpaqueFunction(function=create_sdf_mapping_node)
    sdf_visualization_node = OpaqueFunction(function=create_sdf_visualization_node)
    rviz_node = OpaqueFunction(function=create_rviz_node)

    # rosbag play node
    rosbag_play_node = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("rosbag_dir"),
            "--rate",
            LaunchConfiguration("rosbag_play_rate"),
            "--clock",
        ],
        condition=IfCondition(LaunchConfiguration("play_rosbag")),
        output="screen",
    )

    return LaunchDescription(
        [
            # Launch arguments
            precision_arg,
            surf_mapping_method_arg,
            visualize_sdf_arg,
            open_rviz_arg,
            open_rqt_plot_arg,
            robot_name_arg,
            rosbag_dir_arg,
            rosbag_play_rate_arg,
            play_rosbag_arg,
            # Nodes
            map_odom_transformer,
            sdf_mapping_node,
            sdf_visualization_node,
            rviz_node,
            rqt_plot_node,
            rosbag_play_node,
        ]
    )
