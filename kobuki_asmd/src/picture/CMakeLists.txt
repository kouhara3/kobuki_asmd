###############################################################################
# Picture
###############################################################################

add_executable(asmd_SaveMap SaveMap.cpp )
target_link_libraries(asmd_SaveMap kobuki_asmd ${ROS_LIBRARIES} ${catkin_LIBRARIES} X11 m GL GLU glut)

 
install(TARGETS asmd_SaveMap
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
