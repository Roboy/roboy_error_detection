#include "roboy_error_detection/roboyErrorDetection.hpp"

RoboyErrorDetection::RoboyErrorDetection(ros::NodeHandlePtr nh) {
    ROS_INFO("Calls constructor");
    notifier.setNodeHandler(nh);

    motorSub = nh->subscribe("/roboy/middleware/MotorStatus", 1000, &RoboyErrorDetection::handleMotorStatusErrors, this);
    ROS_DEBUG("Subscribed to /roboy/middleware/MotorStatus");
};

void RoboyErrorDetection::handleMotorStatusErrors(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    handleMotorHealthCheck(msg);
}

void RoboyErrorDetection::listenForMotorHealth(MotorID motorId, NotificationInterval minPublishIntervalInMs, NotificationLevel logLevel) {
    // TODO: check for invalid input

    // check if we are not still listening to motor --> add new entry
    if (subscriptionsForMotorHealth.find(motorId) ==
        subscriptionsForMotorHealth.end()) {
        subscriptionsForMotorHealth[motorId] = {};
    }

    NotificationData notificationData (minPublishIntervalInMs);
    // TODO: check what happens if subscription still exists (existing error subscription with 2000ms, what happens if another come with 1000ms?)
    subscriptionsForMotorHealth[motorId][logLevel] = notificationData;
}

void RoboyErrorDetection::handleMotorHealthCheck(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    // determine time since last health check for given motor
    float timeSinceLastMotorHealthCheck = 0.0;
    if (lastMotorHealthCheckTime.isValid()) {
        timeSinceLastMotorHealthCheck =
                (ros::Time::now() - lastMotorHealthCheckTime).toSec() * 1000; // time in ms
    }
    lastMotorHealthCheckTime = ros::Time::now(); // update time of last motor check --> is current now

    ROS_INFO("Received motor health check message");
    // check all subscribed motors if we should send a health message
    for(auto const &motorEntry : subscriptionsForMotorHealth) {
        MotorID motorId = motorEntry.first;
        ROS_INFO("Found entry with motor ID %d", motorId);

        // loop and process all subscriptions for this motor
        for(auto const &motorSubscriptions : motorEntry.second) {
            NotificationLevel lvl = motorSubscriptions.first;
            NotificationData subscriptionData = motorSubscriptions.second;

            NotificationInterval minPublishIntervalInMs = std::get<0>(subscriptionData);
            if (minPublishIntervalInMs == 0 || minPublishIntervalInMs < timeSinceLastMotorHealthCheck) {
                bool isMotorDead = msg->current[motorId] == 0;
                publishMessage(lvl, isMotorDead ? MOTOR_DEAD_NOTIFICATION : MOTOR_ALIVE_NOTIFICATION, motorId);
            }
        }
    }
    /**
     * notificationsList = mapSubscribedDeadMotorsToNotificationLevels[motorId];
    if (!isLogLevelInList(notificationsList, logLevel)) {
    subscriptionsForMotorHealth[motorId].push_back(logLevel);
    ROS_INFO("ROS sends now a notification for motor %d over the topic %s", motorId, topicAddresses[logLevel]);
    } else {
    ROS_WARNING("Notification for motor %d still exists --> ignored", motorId);
    }
     */
}

void RoboyErrorDetection::publishMessage(NotificationLevel level, NotificationCode notificationCode, uint16_t objectId) {
    switch(level) {
        case DANGER_LEVEL:
            notifier.sendDangerMessage(notificationCode, objectId);
            break;
        case ERROR_LEVEL:
            notifier.sendErrorMessage(notificationCode, objectId);
            break;
        case WARNING_LEVEL:
            notifier.sendWarningMessage(notificationCode, objectId);
            break;
        case INFO_LEVEL:
            notifier.sendInfoMessage(notificationCode, objectId);
            break;
        case DEBUG_LEVEL:
            notifier.sendDebugMessage(notificationCode, objectId);
            break;
        default:
            ROS_WARN("Undefined notification level --> notification could not be sent");
    }
}