###############################################################################
# Picture
###############################################################################

add_executable(asmd_SaveMap SaveMap.cpp )
target_link_libraries(asmd_SaveMap kobuki_asmd ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS asmd_SaveMap
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
