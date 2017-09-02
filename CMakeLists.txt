PROJECT(roboy_error_detection C CXX)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g ")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

#==============================================================================
# Dependencies
#==============================================================================

# ros
find_package(catkin REQUIRED COMPONENTS roboy_system_notification roboy_communication_middleware roscpp std_msgs)
catkin_package( CATKIN_DEPENDS message_runtime roboy_system_notification roboy_communication_middleware INCLUDE_DIRS include)
include_directories(${catkin_INCLUDE_DIRS} )

INCLUDE_DIRECTORIES(
        include/
)

#==============================================================================
# Application
#==============================================================================
add_library(roboy_error_detection src/roboyErrorDetection.cpp)
add_dependencies(roboy_error_detection roboy_system_notification_gencpp roboy_communication_middleware_gencpp)
target_link_libraries(roboy_error_detection ${catkin_LIBRARIES} roboy_system_notification)

add_library(roboy_error_detection_main_base src/main/main_common.cpp)
add_dependencies(roboy_error_detection_main_base roboy_system_notification_gencpp roboy_communication_middleware_gencpp)
target_link_libraries(roboy_error_detection_main_base ${catkin_LIBRARIES} roboy_error_detection)

add_executable(motor_health_check src/main/main_motor_health_check.cpp)
add_dependencies(motor_health_check roboy_system_notification_gencpp roboy_communication_middleware_gencpp)
target_link_libraries(motor_health_check ${catkin_LIBRARIES} roboy_error_detection roboy_error_detection_main_base)

add_executable(motor_dead_check src/main/main_motor_dead_check.cpp)
add_dependencies(motor_dead_check roboy_system_notification_gencpp roboy_communication_middleware_gencpp)
target_link_libraries(motor_dead_check ${catkin_LIBRARIES} roboy_error_detection roboy_error_detection_main_base)

add_executable(motor_alive_check src/main/main_motor_alive_check.cpp)
add_dependencies(motor_alive_check roboy_system_notification_gencpp roboy_communication_middleware_gencpp)
target_link_libraries(motor_alive_check ${catkin_LIBRARIES} roboy_error_detection roboy_error_detection_main_base)

add_executable(joint_rel_angle_check src/main/main_joint_rel_angle_check.cpp)
add_dependencies(joint_rel_angle_check roboy_system_notification_gencpp roboy_communication_middleware_gencpp)
target_link_libraries(joint_rel_angle_check ${catkin_LIBRARIES} roboy_error_detection roboy_error_detection_main_base)

add_executable(joint_magnet_check src/main/main_joint_magnet_check.cpp)
add_dependencies(joint_magnet_check roboy_system_notification_gencpp roboy_communication_middleware_gencpp)
target_link_libraries(joint_magnet_check ${catkin_LIBRARIES} roboy_error_detection roboy_error_detection_main_base)

add_executable(motor_tendon_inconsistence src/main/main_motor_tendon_inconsistence.cpp)
add_dependencies(motor_tendon_inconsistence roboy_system_notification_gencpp roboy_communication_middleware_gencpp)
target_link_libraries(motor_tendon_inconsistence ${catkin_LIBRARIES} roboy_error_detection roboy_error_detection_main_base)

add_executable(presentation src/main/main_presentation.cpp)
add_dependencies(presentation roboy_system_notification_gencpp roboy_communication_middleware_gencpp)
target_link_libraries(presentation ${catkin_LIBRARIES} roboy_error_detection roboy_error_detection_main_base)
