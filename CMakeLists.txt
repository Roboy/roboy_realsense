PROJECT(roboy_realsense C CXX)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g ")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

#==============================================================================
# Dependencies
#==============================================================================

# ros
find_package(catkin REQUIRED COMPONENTS roboy_communication_middleware roscpp std_msgs aruco_detect controller_manager common_utilities)
catkin_package( CATKIN_DEPENDS message_runtime roboy_communication_middleware common_utilities INCLUDE_DIRS include)
include_directories(${catkin_INCLUDE_DIRS} )

find_package(librealsense REQUIRED)

INCLUDE_DIRECTORIES(
        include/
    )

#==============================================================================
# Application
#==============================================================================

add_executable(roboy_realsense src/main.cpp src/roboyRealsense.cpp)
add_dependencies(roboy_realsense roboy_communication_middleware_gencpp) 
target_link_libraries(roboy_realsense ${catkin_LIBRARIES} ${librealsense_LIBRARIES})

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        roboy_realsense
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
