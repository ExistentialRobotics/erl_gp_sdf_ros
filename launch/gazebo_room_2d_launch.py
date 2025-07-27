#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for Gazebo room 2D SDF mapping."""

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
    scan_publish_rate_arg = DeclareLaunchArgument(
        "scan_publish_rate",
        default_value="60.0",
        description="Rate at which the laser scan is published",
    )

    # Gazebo room 2D node
    gazebo_room_2d_node = Node(
        package="erl_geometry_ros",
        executable="gazebo_room_2d_node",
        name="gazebo_room_2d_node",  # No explicit namespace, so it uses root namespace
        output="screen",
        parameters=[
            {
                "data_folder": PathJoinSubstitution([FindPackageShare("erl_geometry"), "data", "gazebo"]),
                "laser_frame": "front_laser",
                "map_frame": "map",
                "topic_name": "scan",  # Namespace is `/`, so the actual path is `/scan`.
                "publish_rate": LaunchConfiguration("scan_publish_rate"),
            }
        ],
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

        # Get package share directories
        erl_gp_sdf_ros_share = get_package_share_directory("erl_gp_sdf_ros")
        erl_gp_sdf_share = get_package_share_directory("erl_gp_sdf")

        # Build config file paths
        config_file = os.path.join(erl_gp_sdf_ros_share, "config", "ros2", f"{surf_mapping_method}_{precision}_2d.yaml")
        surf_mapping_config = os.path.join(
            erl_gp_sdf_share, "config", "gazebo", f"{surf_mapping_method}_surf_mapping_{precision}.yaml"
        )
        sdf_mapping_config = os.path.join(
            erl_gp_sdf_share, "config", "gazebo", f"{surf_mapping_method}_sdf_mapping_{precision}.yaml"
        )
        scan_frame_config = os.path.join(erl_gp_sdf_share, "config", "gazebo", "lidar_frame_2d.yaml")

        return [
            Node(
                package="erl_gp_sdf_ros",
                executable="sdf_mapping_node",
                name="sdf_mapping_node",
                output="screen",
                parameters=[
                    ParameterFile(config_file, allow_substs=True),
                    {
                        "map_dim": 2,
                        "double_precision": precision == "double",
                        "surface_mapping_setting_file": surf_mapping_config,
                        "sdf_mapping_setting_file": sdf_mapping_config,
                        "scan_topic": "scan",
                        "scan_frame_setting_file": scan_frame_config,
                        "publish_tree": True,
                    },
                ],
            )
        ]

    # SDF visualization node group
    sdf_visualization_node = Node(
        package="erl_gp_sdf_ros",
        executable="sdf_visualization_node",
        name="sdf_visualization_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("visualize_sdf")),
        parameters=[
            {
                "resolution": 0.07,
                "x_cells": 401,
                "y_cells": 401,
                "x": 7.5,
                "y": -5.5,
                "z": -0.01,
                "publish_gradient": True,
                "publish_sdf_variance": True,
                "publish_gradient_variance": True,
                "publish_covariance": False,
                "publish_grid_map": True,
                "publish_point_cloud": True,
                "publish_rate": 20.0,
                "attached_to_frame": False,
                "world_frame": "map",
                "service_name": "sdf_query",
                "map_topic_name": "sdf_grid_map",
                "point_cloud_topic_name": "sdf_point_cloud",
            }
        ],
    )

    # RViz node group
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("erl_gp_sdf_ros"), "rviz2", "gazebo_room_2d.rviz"]),
        ],
        condition=IfCondition(LaunchConfiguration("open_rviz")),
    )

    # rqt_plot node group
    rqt_plot_node = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="rqt_plot",
        arguments=["/update_time/temperature", "/query_time/temperature"],
        condition=IfCondition(LaunchConfiguration("open_rqt_plot")),
    )

    # Create SDF mapping node using OpaqueFunction for dynamic parameter evaluation
    sdf_mapping_node = OpaqueFunction(function=create_sdf_mapping_node)

    return LaunchDescription(
        [
            # Launch arguments
            precision_arg,
            surf_mapping_method_arg,
            visualize_sdf_arg,
            open_rviz_arg,
            open_rqt_plot_arg,
            scan_publish_rate_arg,
            # Nodes
            gazebo_room_2d_node,
            map_odom_transformer,
            sdf_mapping_node,
            sdf_visualization_node,
            rviz_node,
            rqt_plot_node,
        ]
    )
