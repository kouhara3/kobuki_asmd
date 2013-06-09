###############################################################################
# Tests
###############################################################################

add_executable(asmd_kobuki_velocity_commands velocity_commands.cpp)
target_link_libraries(asmd_kobuki_velocity_commands kobuki_asmd)

add_executable(asmd_demo_kobuki_initialisation initialisation.cpp)
target_link_libraries(asmd_demo_kobuki_initialisation kobuki_asmd)

add_executable(asmd_demo_kobuki_sigslots sigslots.cpp)
target_link_libraries(asmd_demo_kobuki_sigslots kobuki_asmd)

add_executable(asmd_demo_kobuki_simple_loop simple_loop.cpp)
target_link_libraries(asmd_demo_kobuki_simple_loop kobuki_asmd)

install(TARGETS asmd_kobuki_velocity_commands asmd_demo_kobuki_initialisation asmd_demo_kobuki_sigslots asmd_demo_kobuki_simple_loop
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
