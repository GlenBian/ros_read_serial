##############################################################################
# NODELET
##############################################################################

add_library(youibot_bms_nodelet youibot_bms_nodelet.cpp)
#add_dependencies(youibot_nodelet youibot_ros )
target_link_libraries(youibot_bms_nodelet youibot_bms ${catkin_LIBRARIES})

install(TARGETS youibot_bms_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
