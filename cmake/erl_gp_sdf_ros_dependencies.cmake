if(ROS1_ACTIVATED)
    erl_config_qt5(Core Widgets)
elseif(ROS2_ACTIVATED)
    erl_config_qt(Core Widgets) # Qt5 or Qt6
endif()

# Cap'n Proto: required only by ROS2 sdf_mapping_node for the RPC query
# interface. CapnProto ships its own CMake config with capnp_generate_cpp.
if(ROS2_ACTIVATED)
    find_package(CapnProto REQUIRED)
endif()
