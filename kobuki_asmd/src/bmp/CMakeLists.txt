###############################################################################
# Picture
###############################################################################

add_executable(asmd_BmpMain main.cpp)
target_link_libraries(asmd_BmpMain kobuki_asmd ${ROS_LIBRARIES} ${catkin_LIBRARIES} X11 m GL GLU glut)

 
install(TARGETS asmd_BmpMain
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
