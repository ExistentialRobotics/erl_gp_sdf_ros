message(STATUS "ROS1 activated, building ROS1 stuff")

add_executable(sdf_mapping_node src/ros1/sdf_mapping_node.cpp)
target_include_directories(sdf_mapping_node PRIVATE ${Qt5_INCLUDE_DIRS})
erl_target_dependencies(sdf_mapping_node PRIVATE ${Qt5_LIBRARIES})
erl_collect_targets(EXECUTABLES sdf_mapping_node)

add_executable(sdf_visualization_node src/ros1/sdf_visualization_node.cpp)
erl_target_dependencies(sdf_visualization_node)
erl_collect_targets(EXECUTABLES sdf_visualization_node)
