cmake_minimum_required(VERSION 2.8.3)

project(roboy_error_detection)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g ")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

#==============================================================================
# Dependencies
#==============================================================================

# ros
find_package(catkin REQUIRED COMPONENTS roboy_system_notifications roboy_communication_control roscpp std_msgs controller_manager)
catkin_package( CATKIN_DEPENDS message_runtime roboy_communication_control roboy_system_notifications INCLUDE_DIRS include)
include_directories(${catkin_INCLUDE_DIRS} )

INCLUDE_DIRECTORIES(
        include/
)

#==============================================================================
# Application
#==============================================================================

add_executable(roboy_error_detection src/main.cpp src/roboyErrorDetection.cpp)
add_dependencies(roboy_communication_control_gencpp)
target_link_libraries(roboy_error_detection ${catkin_LIBRARIES})

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