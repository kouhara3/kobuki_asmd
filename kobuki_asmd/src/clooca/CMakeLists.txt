###############################################################################
# Tests
###############################################################################

add_executable(asmd_test_clooca main.cpp MEXU.cpp stm.cpp)
target_link_libraries(asmd_test_clooca kobuki_asmd ${ROS_LIBRARIES} ${catkin_LIBRARIES} X11 m GL GLU glut)

install(TARGETS  asmd_test_clooca
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
