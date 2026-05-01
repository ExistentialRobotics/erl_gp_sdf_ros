#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    """Generate launch description for cow_and_lady dataset SDF mapping."""

    # Declare launch arguments

    # precision_arg = DeclareLaunchArgument(
    #     "precision",
    #     default_value="float",
    #     description="Options: float, double",
    # )
    # surf_mapping_method_arg = DeclareLaunchArgument(
    #     "surf_mapping_method",
    #     default_value="bayesian_hilbert",
    #     description="Options: gp_occ, bayesian_hilbert",
    # )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="use_sim_time",
    )
    visualize_sdf_arg = DeclareLaunchArgument(
        "visualize_sdf",
        default_value="true",
        description="Whether to launch the sdf visualization node",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS log level for the nodes (e.g. debug, info, warn, error)",
    )

    cfg_dir = PathJoinSubstitution([FindPackageShare("erl_gp_sdf_ros"), "config", "xarm"])
    ros_params_file = ParameterFile(
        PathJoinSubstitution([cfg_dir, "ros_params.yaml"]),
        allow_substs=True,
    )
    sdf_mapping_node = Node(
        package="erl_gp_sdf_ros",
        executable="sdf_mapping_node",
        name="sdf_mapping_node",
        output="screen",
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        parameters=[
            ros_params_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "surface_mapping_setting_file": PathJoinSubstitution([cfg_dir, "surf_mapping.yaml"]),
                "sdf_mapping_setting_file": PathJoinSubstitution([cfg_dir, "sdf_mapping.yaml"]),
            },
        ],
    )

    sdf_visualization_node = Node(
        package="erl_gp_sdf_ros",
        executable="sdf_visualization_node",
        name="sdf_visualization_node",
        output="screen",
        ros_arguments=["--log-level", LaunchConfiguration("log_level")],
        parameters=[
            ros_params_file,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("visualize_sdf")),
    )

    return LaunchDescription(
        [
            # Launch arguments
            use_sim_time_arg,
            visualize_sdf_arg,
            log_level_arg,
            # Nodes
            sdf_mapping_node,
            sdf_visualization_node,
        ]
    )
