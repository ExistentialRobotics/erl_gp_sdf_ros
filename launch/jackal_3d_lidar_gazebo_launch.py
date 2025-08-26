#!/usr/bin/env python3

import os
#TODO fix rviz config, add documentation on how to use Humble for clearpath simulator.
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    """
    Launch SDF mapping & visualization nodes that subscribe to *existing*
    namespaced topics (e.g., /jackal2/*), including /jackal2/tf and /jackal2/tf_static.
    """
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="jackal2", description="Namespace for the robot (e.g., jackal2)"
    )
    precision_arg = DeclareLaunchArgument(
        "precision", default_value="float", description="Options: float, double"
    )
    surf_mapping_method_arg = DeclareLaunchArgument(
        "surf_mapping_method", default_value="bayesian_hilbert",
        description="Options: gp_occ, bayesian_hilbert"
    )
    visualize_sdf_arg = DeclareLaunchArgument(
        "visualize_sdf", default_value="true", description="Whether to launch the SDF visualization node"
    )
    open_rviz_arg = DeclareLaunchArgument(
        "open_rviz", default_value="true", description="Whether to launch RViz for visualization"
    )
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="jackal2",
        description="Robot name used in frame/topic strings (usually same as namespace)"
    )
    setting_type_arg = DeclareLaunchArgument(
        "setting_type", default_value="lidar",
        description='Setting type token expected by the node (e.g., "lidar" or "lidar3d")'
    )
    world_frame_arg = DeclareLaunchArgument(
        "world_frame", default_value="odom",
        description="World frame that SDF is attached to (typically odom)"
    )
    sensor_frame_arg = DeclareLaunchArgument(
        "sensor_frame", default_value="lidar3d_0_laser",
        description="Sensor frame ID published in TF (prefix with robot if your TF tree uses it)"
    )
    tf_remaps = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]
    def create_sdf_mapping_node(context):
        ns = context.launch_configurations["namespace"]
        precision = context.launch_configurations["precision"]
        method = context.launch_configurations["surf_mapping_method"]
        robot = context.launch_configurations["robot_name"]
        setting_type = context.launch_configurations["setting_type"]
        world_frame = context.launch_configurations["world_frame"]
        sensor_frame = context.launch_configurations["sensor_frame"]
        precision = context.launch_configurations["precision"]
        pkg_share = get_package_share_directory("erl_gp_sdf_ros")
        base1 = os.path.join(pkg_share, "config", "ros2", f"{method}_{precision}_3d.yaml")
        base2 = os.path.join(pkg_share, "config", "ros2", f"use_lidar_frame_3d_{precision}.yaml")
        ns1 = os.path.join(pkg_share, "config", "ros2", f"{ns}_{method}_{precision}_3d.yaml")
        ns2 = os.path.join(pkg_share, "config", "ros2", f"{ns}_use_lidar_frame_3d_{precision}.yaml")
        config_file_1 = ns1 if os.path.exists(ns1) else base1
        config_file_2 = ns2 if os.path.exists(ns2) else base2
        surf_mapping_config = os.path.join(
            pkg_share, "config", "jackal_3d", f"{method}_surf_mapping_lidar_{precision}.yaml"
        )
        sdf_mapping_config = os.path.join(
            pkg_share, "config", "jackal_3d", f"{method}_sdf_mapping_lidar_{precision}.yaml"
        )
        scan_frame_config = os.path.join(pkg_share, "config", "jackal_3d", "lidar_frame_3d_360_gazebo.yaml")
        odom_topic = f"/{robot}/platform/odom"
        scan_topic = f"/{robot}/sensors/lidar3d_0/points"
        type_token = (
            "erl::gp_sdf::LogEdfGaussianProcess<double>::Setting"
            if precision == "double"
            else "erl::gp_sdf::LogEdfGaussianProcess<float>::Setting"
        )
        return [Node(
            package="erl_gp_sdf_ros",
            executable="sdf_mapping_node",
            name="sdf_mapping_node",
            namespace=ns,
            output="screen",
            parameters=[
                ParameterFile(config_file_1, allow_substs=True),
                ParameterFile(config_file_2, allow_substs=True),
                {
                    "map_dim": 3,
                    "double_precision": precision == "double",
                    "surface_mapping_setting_file": surf_mapping_config,
                    "sdf_mapping_setting_file": sdf_mapping_config,
                    "sdf_mapping_setting_type": setting_type,
                    "use_odom": False,
                    "odom_topic": odom_topic,
                    "odom_msg_type": "nav_msgs/msg/Odometry",
                    "world_frame": world_frame,
                    "sensor_frame": sensor_frame,
                    "scan_topic": scan_topic,
                    "scan_type": "sensor_msgs/msg/PointCloud2",
                    "scan_frame_setting_file": scan_frame_config,
                    "scan_stride": 1,
                    "convert_scan_to_points": method == "bayesian_hilbert",
                    "scan_in_local_frame": True,
                    "depth_scale": 0.001,

                    # Outputs
                    "publish_tree": True,
                },
            ],
            remappings=tf_remaps,
            arguments=["--ros-args", "--log-level", "rclcpp:=info"],
        )]

    def create_sdf_visualization_node(context):
        ns = context.launch_configurations["namespace"]
        robot = context.launch_configurations["robot_name"]
        world_frame = context.launch_configurations["world_frame"]
        sensor_frame = context.launch_configurations["sensor_frame"]

        return [Node(
            package="erl_gp_sdf_ros",
            executable="sdf_visualization_node",
            name="sdf_visualization_node",
            namespace=ns,
            output="screen",
            condition=IfCondition(LaunchConfiguration("visualize_sdf")),
            parameters=[{
                "resolution": 0.02,
                "x_cells": 101,
                "y_cells": 101,
                "x": 0.0, "y": 0.0, "z": 0.0,
                "publish_gradient": True,
                "publish_sdf_variance": True,
                "publish_gradient_variance": True,
                "publish_covariance": False,
                "publish_grid_map": True,
                "publish_point_cloud": True,
                "publish_rate": 10.0,
                "attached_to_frame": True,
                "world_frame": world_frame,
                "target_frame": world_frame,  
                "attached_frame": sensor_frame,
                "service_name": "sdf_query",
                "map_topic_name": "sdf_grid_map",
                "point_cloud_topic_name": "sdf_point_cloud",
            }],
            remappings=tf_remaps,
        )]

    def create_rviz_node(context):
        ns = context.launch_configurations["namespace"]
        robot = context.launch_configurations["robot_name"]
        pkg_share = get_package_share_directory("erl_gp_sdf_ros")
        rviz_config = os.path.join(pkg_share, "rviz2", f"{robot}_3d_lidar_gazebo.rviz")

        return [Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=ns,
            output="screen",
            condition=IfCondition(LaunchConfiguration("open_rviz")),
            arguments=["-d", rviz_config],
            remappings=tf_remaps,
        )]

    # Build nodes via OpaqueFunction so we can evaluate substitutions early
    sdf_mapping_node = OpaqueFunction(function=create_sdf_mapping_node)
    sdf_visualization_node = OpaqueFunction(function=create_sdf_visualization_node)
    rviz_node = OpaqueFunction(function=create_rviz_node)

    return LaunchDescription([
        # Args
        namespace_arg,
        precision_arg,
        surf_mapping_method_arg,
        visualize_sdf_arg,
        open_rviz_arg,
        robot_name_arg,
        setting_type_arg,
        world_frame_arg,
        sensor_frame_arg,

        # Nodes
        sdf_mapping_node,
        sdf_visualization_node,
        rviz_node,
    ])
