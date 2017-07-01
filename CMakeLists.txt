PROJECT(roboy_error_detection C CXX)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g ")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

#==============================================================================
# Dependencies
#==============================================================================

# ros
find_package(catkin REQUIRED COMPONENTS roboy_communication_middleware roscpp std_msgs controller_manager common_utilities)
catkin_package( CATKIN_DEPENDS message_runtime roboy_communication_middleware common_utilities INCLUDE_DIRS include)
include_directories(${catkin_INCLUDE_DIRS} )

find_package(librealsense REQUIRED)

INCLUDE_DIRECTORIES(
        include/
)

#==============================================================================
# Application
#==============================================================================

add_executable(roboy_error_detection src/main.cpp src/roboyRealsense.cpp)
add_dependencies(roboy_error_detection roboy_communication_middleware_gencpp)

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        roboy_error_detection
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        package.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )