message(STATUS "ROS2 activated, building ROS2 stuff")

# Generate C++ bindings for the Cap'n Proto query schema. With default
# CAPNPC_SRC_PREFIX = CMAKE_CURRENT_SOURCE_DIR and default CAPNPC_OUTPUT_DIR =
# CMAKE_CURRENT_BINARY_DIR, the generated header lands at
# ${CMAKE_CURRENT_BINARY_DIR}/capnp/sdf_query.capnp.h.
capnp_generate_cpp(
    ERL_GP_SDF_ROS_CAPNP_SRCS
    ERL_GP_SDF_ROS_CAPNP_HDRS
    capnp/sdf_query.capnp)

add_executable(sdf_mapping_node
    src/ros2/sdf_mapping_node.cpp
    ${ERL_GP_SDF_ROS_CAPNP_SRCS})
target_include_directories(sdf_mapping_node PRIVATE
    ${QT_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(sdf_mapping_node PRIVATE CapnProto::capnp-rpc)
erl_target_dependencies(sdf_mapping_node PRIVATE ${QT_LIBRARIES})
erl_collect_targets(EXECUTABLES sdf_mapping_node)

add_executable(sdf_visualization_node src/ros2/sdf_visualization_node.cpp)
erl_target_dependencies(sdf_visualization_node)
erl_collect_targets(EXECUTABLES sdf_visualization_node)

add_executable(occ_visualization_node src/ros2/occ_visualization_node.cpp)
erl_target_dependencies(occ_visualization_node)
erl_collect_targets(EXECUTABLES occ_visualization_node)

# Install the .capnp schema so Python clients (and other consumers) can load it
# from the standard ROS2 share/<package> directory.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/capnp
        DESTINATION share/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.capnp")
