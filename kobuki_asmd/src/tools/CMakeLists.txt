###############################################################################
# QTConfigure
###############################################################################


###############################################################################
# Tools
###############################################################################

add_executable(asmd_version_info version_info.cpp)
target_link_libraries(asmd_version_info kobuki_asmd)

#add_executable(asmd_Coordinate Coordinate.cpp)
#target_link_libraries(asmd_Coordinate kobuki_asmd)

add_executable(asmd_Map Map.cpp )
target_link_libraries(asmd_Map kobuki_asmd ${QT_LIBRARIES} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(asmd_Utilities Utilities.cpp)
#target_link_libraries(asmd_Utilities kobuki_asmd)

#add_executable(asmd_KobukiManager KobukiManager.cpp)
#target_link_libraries(asmd_KobukiManager kobuki_asmd)

install(TARGETS asmd_version_info asmd_Map
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
