###############################################################################
# Tools
###############################################################################

add_executable(asmd_version_info version_info.cpp)
target_link_libraries(asmd_version_info kobuki_asmd)

add_executable(asmd_demo_kobuki_simple_run simple_run.cpp)
target_link_libraries(asmd_demo_kobuki_simple_run kobuki_asmd)

install(TARGETS asmd_version_info
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
