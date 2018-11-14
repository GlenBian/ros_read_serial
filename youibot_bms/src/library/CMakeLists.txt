##############################################################################
# SOURCES
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# LIBRARY
##############################################################################

add_library(youibot_bms ${SOURCES})
message("SOURCES: ${SOURCES}")
add_dependencies(youibot_bms ${catkin_EXPORTED_TARGETS})
#message("catkin_EXPORTED_TARGETS: ${catkin_EXPORTED_TARGETS}")
target_link_libraries(youibot_bms ${catkin_LIBRARIES})
#message("catkin_LIBRARIES: ${catkin_LIBRARIES}")
install(TARGETS youibot_bms
       DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
