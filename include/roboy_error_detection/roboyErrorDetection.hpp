#pragma once

#include <ros/ros.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <roboy_system_notification/roboySystemNotification.hpp>
#include <map>
#include <list>
#include <vector>
#include <tuple>

using namespace std;

class RoboyErrorDetection{
public:
    RoboyErrorDetection(ros::NodeHandlePtr nh);

    /**
     * Listen if the motor with the given id is dead. If the defined motor is really dead, a notification will be sent over
     * ROS topic.
     * @param motorId id of the motor, which should be observed
     * @param logLevel message level of the published message
     */
    void listenForDeadMotor(int motorId, NotificationLevel logLevel=WARNING_LEVEL);

    void handleMotorStatusErrors(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);

private:
    std::map<int, std::list<NotificationLevel>> mapSubscribedDeadMotorsToNotificationLevels;
    ros::Subscriber motorSub;

    RoboySystemNotification notifier;
};