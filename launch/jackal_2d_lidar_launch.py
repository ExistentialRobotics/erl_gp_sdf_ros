#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    """Generate launch description for jackal 2D LiDAR SDF mapping."""

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
    # open_rqt_plot_arg = DeclareLaunchArgument(
    #     "open_rqt_plot",
    #     default_value="true",
    #     description="Whether to launch rqt_plot for performance analysis",
    # )
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="jackal2",
        description="Robot name for topic namespacing",
    )

    world_frame_arg = DeclareLaunchArgument(
        "world_frame",
        default_value="odom",
        description="World frame for the map",
    )
    robot_name = LaunchConfiguration("robot_name")
    sensor_frame_arg = DeclareLaunchArgument(
        "sensor_frame",
        default_value=[LaunchConfiguration("robot_name"), "/dlio/os_sensor"],
        description="Sensor frame",
    )
    rosbag_dir_arg = DeclareLaunchArgument(
        "rosbag_dir",
        default_value=os.path.join(os.environ["HOME"], "Data", "jackal_lidar_2d", "jackal2_map_Lab_ros2"),
        description="Path to the rosbag directory to play",
    )
    rosbag_play_rate_arg = DeclareLaunchArgument(
        "rosbag_play_rate",
        default_value="1.0",
        description="Rate at which to play the rosbag file",
    )
    rosbag_play_start_arg = DeclareLaunchArgument(
        "rosbag_play_start",
        default_value="0.0",
        description="Start time for rosbag playback",
    )
    rosbag_play_duration_arg = DeclareLaunchArgument(
        "rosbag_play_duration",
        default_value="10000.0",
        description="Duration for rosbag playback",
    )
    play_rosbag_arg = DeclareLaunchArgument(
        "play_rosbag",
        default_value="true",
        description="Whether to play the rosbag file",
    )

    use_sim_time = LaunchConfiguration("play_rosbag")

    # Pointcloud to laserscan node
    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        namespace=robot_name,
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", ["/", LaunchConfiguration("robot_name"), "/dlio/odom_node/pointcloud/deskewed"]),
            ("scan", "scan"),
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "target_frame": [LaunchConfiguration("robot_name"), "/dlio/os_sensor"],
                "transform_tolerance": 0.01,
                "min_height": 0.2,
                "max_height": 2.0,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0087,  # M_PI/360.0
                "scan_time": 0.3333,
                "range_min": 0.5,
                "range_max": 10.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
                "concurrency_level": 1,
            }
        ],
        arguments=["--ros-args", "--log-level", "rclcpp:=info"],
    )

    # Static transform publisher (map -> odom)
    map_odom_transformer = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_transformer",
        parameters=[{"use_sim_time": use_sim_time}],
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
        world_frame = context.launch_configurations["world_frame"]
        sensor_frame = context.launch_configurations["sensor_frame"]
        robot_name = context.launch_configurations["robot_name"]

        # Get package share directory
        pkg_share = get_package_share_directory("erl_gp_sdf_ros")

        # Build config file paths
        config_file = os.path.join(pkg_share, "config", "ros2", f"{robot_name}_{surf_mapping_method}_{precision}_2d.yaml")
        surf_mapping_config = os.path.join(
            pkg_share, "config", "jackal_2d", surf_mapping_method, f"surf_mapping_{precision}_scale0.1.yaml"
        )
        sdf_mapping_config = os.path.join(
            pkg_share, "config", "jackal_2d", surf_mapping_method, f"sdf_mapping_{precision}.yaml"
        )
        scan_frame_config = os.path.join(pkg_share, "config", "jackal_2d", "lidar_frame_2d.yaml")

        return [
            Node(
                package="erl_gp_sdf_ros",
                namespace=robot_name,
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
                        "use_odom": False,
                        "world_frame": world_frame,
                        "sensor_frame": sensor_frame,
                        "scan_topic.path": "scan",  # under the robot namespace
                        "scan_topic.qos_reliability": "best_effort",  # pointcloud_to_laserscan publishes with best effort reliability
                        "scan_type": "laser",
                        "scan_frame_setting_file": scan_frame_config,
                        "scan_stride": 1,
                        "convert_scan_to_points": surf_mapping_method == "bayesian_hilbert",
                        "scan_in_local_frame": True,
                        "publish_tree": True,
                    },
                ],
                # remappings=tf_remaps,
                arguments=["--ros-args", "--log-level", "rclcpp:=info"],
            )
        ]

    def create_sdf_visualization_node(context):
        """Create SDF visualization node."""
        world_frame = context.launch_configurations["world_frame"]
        sensor_frame = context.launch_configurations["sensor_frame"]
        robot_name = context.launch_configurations["robot_name"]

        return [
            Node(
                package="erl_gp_sdf_ros",
                namespace=robot_name,
                executable="sdf_visualization_node",
                name="sdf_visualization_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("visualize_sdf")),
                parameters=[
                    {
                        "resolution": 0.1,
                        "x_cells": 201,
                        "y_cells": 131,
                        "x": -2.5,
                        "y": -1.5,
                        "z": 0.0,
                        "publish_gradient": True,
                        "publish_sdf_variance": True,
                        "publish_gradient_variance": False,
                        "publish_covariance": False,
                        "publish_grid_map": True,
                        "publish_point_cloud": False,
                        "publish_rate": 4.0,
                        "attached_to_frame": False,
                        "world_frame": world_frame,
                        "attached_frame": sensor_frame,
                        "sdf_query_service.path": "sdf_query",
                        "map_topic.path": "sdf_grid_map",
                        "point_cloud_topic.path": "sdf_point_cloud",
                    }
                ],
            )
        ]

    def create_rviz_node(context):
        """Create RViz node."""
        robot_name = context.launch_configurations["robot_name"]
        pkg_share = get_package_share_directory("erl_gp_sdf_ros")
        rviz_config = os.path.join(pkg_share, "rviz2", f"{robot_name}_2d_lidar.rviz")

        return [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                condition=IfCondition(LaunchConfiguration("open_rviz")),
                arguments=["-d", rviz_config],
            )
        ]

    # def create_rqt_plot_node(context):
    #     """Create rqt_plot node."""
    #     return [
    #         Node(
    #             package="rqt_plot",
    #             executable="rqt_plot",
    #             namespace=LaunchConfiguration("robot_name"),
    #             name="rqt_plot",
    #             condition=IfCondition(LaunchConfiguration("open_rqt_plot")),
    #             arguments=["/update_time/data", "/query_time/data"],
    #         )
    #     ]

    # Create nodes using OpaqueFunction for dynamic parameter evaluation
    sdf_mapping_node = OpaqueFunction(function=create_sdf_mapping_node)
    sdf_visualization_node = OpaqueFunction(function=create_sdf_visualization_node)
    rviz_node = OpaqueFunction(function=create_rviz_node)
    # rqt_plot_node = OpaqueFunction(function=create_rqt_plot_node)  # doesn't plot the data automatically

    # rosbag play node
    rosbag_play_node = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("rosbag_dir"),
            "--rate",
            LaunchConfiguration("rosbag_play_rate"),
            "--start-offset",
            LaunchConfiguration("rosbag_play_start"),
            "--playback-duration",
            LaunchConfiguration("rosbag_play_duration"),
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
            # open_rqt_plot_arg,
            robot_name_arg,
            world_frame_arg,
            sensor_frame_arg,
            rosbag_dir_arg,
            rosbag_play_rate_arg,
            rosbag_play_start_arg,
            rosbag_play_duration_arg,
            play_rosbag_arg,
            # Nodes
            pointcloud_to_laserscan_node,
            map_odom_transformer,
            sdf_mapping_node,
            sdf_visualization_node,
            rviz_node,
            # rqt_plot_node,
            rosbag_play_node,
        ]
    )
