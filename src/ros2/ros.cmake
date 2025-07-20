message(STATUS "ROS2 activated, building ROS2 stuff")

add_executable(sdf_mapping_node src/ros2/sdf_mapping_node.cpp)
erl_target_dependencies(sdf_mapping_node)
erl_collect_targets(EXECUTABLES sdf_mapping_node)

add_executable(sdf_visualization_node src/ros2/sdf_visualization_node.cpp)
erl_target_dependencies(sdf_visualization_node)
erl_collect_targets(EXECUTABLES sdf_visualization_node)
