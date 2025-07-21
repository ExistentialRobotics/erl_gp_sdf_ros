cmake_minimum_required(VERSION 3.16)

project(erl_gp_sdf_ros LANGUAGES CXX #
        VERSION 0.1.0 #
        DESCRIPTION "erl_gp_sdf_ros is a ROS wrapper of erl_gp_sdf.")
message(STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")

if (NOT COMMAND erl_project_setup)
    find_package(erl_cmake_tools REQUIRED)
endif ()
erl_project_setup(ERL_PACKAGES erl_gp_sdf)
erl_setup_ros(
    CATKIN_COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen
    grid_map_ros
    grid_map_msgs
    erl_gp_sdf_msgs
    erl_geometry_msgs
    rviz
    #
    CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen
    grid_map_ros
    grid_map_msgs
    erl_gp_sdf_msgs
    erl_geometry_msgs
    rviz
    #
    ROS2_COMPONENTS
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    tf2_eigen
    erl_gp_sdf_msgs
    erl_geometry_msgs
    grid_map_ros
    grid_map_msgs
    rviz_default_plugins)
erl_add_ros_src()
erl_install(EXECUTABLES ${${PROJECT_NAME}_COLLECTED_EXECUTABLES})
erl_mark_project_found()
