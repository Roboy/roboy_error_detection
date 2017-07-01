#pragma once

#include <ros/ros.h>
#include <roboy_communication_control/SystemNotification.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <roboy_system_notification/roboySystemNotification.hpp>
#include "common_utilities.hpp"
#include <vector>
#include <tuple>

using namespace std;

class RoboyErrorDetection{
public:
    RoboyErrorDetection();

    /**
     * Listen if the motor with the given id is dead. If the defined motor is really dead, a notification will be sent over
     * ROS topic.
     * @param motorId id of the motor, which should be observed
     * @param logLevel message level of the published message
     */
    void listenForDeadMotor(int motorId, int logLevel=WARNING_LEVEL);

    void handleMotorStatusErrors(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);

private:
    ros::NodeHandlePtr nh;
    ros::NodeHandle n;
    ros::Subscriber motors_sub;
    std::vector<std::tuple<int,int>> sub_dead_motors = {}; // [(motorId, reportLevel)]

    RoboySystemNotification notifier;
};