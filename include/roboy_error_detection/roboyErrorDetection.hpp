#pragma once

#include <ros/ros.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <roboy_system_notification/roboySystemNotification.hpp>
#include <map>
#include <list>
#include <vector>
#include <tuple>
#include "common_utilities.hpp"

using namespace std;

typedef uint16_t NotificationInterval;
typedef tuple<NotificationInterval> NotificationData;
typedef std::map<NotificationLevel, NotificationData> NotificationDataMap;
typedef int MotorID;

class RoboyErrorDetection {
public:
    RoboyErrorDetection(ros::NodeHandlePtr nh);

    /**
     * Listen if the motor with the given id is dead. If the defined motor is really dead, a notification will be sent over
     * ROS topic.
     * @param motorId id of the motor, which should be observed
     * @param minPublishIntervalInMs interval, which defines the MIN sleep time until the next message will be published. Set 0 to disable waiting a minimum time.
     * @param logLevel message level of the published message
     */
    void
    listenForMotorHealth(MotorID motorId, NotificationInterval minPublishIntervalInMs = 5000, NotificationLevel logLevel = WARNING_LEVEL);

    void handleMotorStatusErrors(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);

private:
    std::map<MotorID, NotificationDataMap> subscriptionsForMotorHealth; // a motor id maps on a list of notifications and their extra information
    ros::Subscriber motorSub;
    RoboySystemNotification notifier;

    static bool isLogLevelInList(std::list <NotificationLevel> notificationsList, NotificationLevel logLevel) {
        return std::find(notificationsList.begin(), notificationsList.end(), logLevel) != notificationsList.end();
    }

    ros::Time lastMotorHealthCheckTime;

    void handleMotorHealthCheck(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);

    void publishMessage(NotificationLevel level, NotificationCode notificationCode, uint16_t objectId);
};