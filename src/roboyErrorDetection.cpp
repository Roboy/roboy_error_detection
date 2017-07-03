#include "roboy_error_detection/roboyErrorDetection.hpp"

RoboyErrorDetection::RoboyErrorDetection(ros::NodeHandlePtr nh) {
    notifier.setNodeHandler(nh);

    motorSub = nh->subscribe("/roboy/middleware/MotorStatus", 1, &RoboyErrorDetection::handleMotorStatusErrors, this);
};

void RoboyErrorDetection::handleMotorStatusErrors(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    // TODO: handle motor states and publish on error
    bool dead = false; // TODO: define if motor is dead: https://github.com/Roboy/myoFPGA/blob/5de1e1751091c34a3da620666ded3e0603e280d3/myoFPGA/src/interface/src/main_window.cpp#L187
    bool shouldCheckMotorDeadStatus = false; // TODO: check if motor id is in sub_dead_motors

    if (dead && shouldCheckMotorDeadStatus) {
        // TODO: send message according to the log level
    }
}

void RoboyErrorDetection::listenForDeadMotor(int motorId, NotificationLevel logLevel) {
    // TODO: check for invalid input

    // check if we are not still listening to motor --> add new entry
    if (mapSubscribedDeadMotorsToNotificationLevels.find(motorId) == mapSubscribedDeadMotorsToNotificationLevels.end()) {
        mapSubscribedDeadMotorsToNotificationLevels[motorId] = {logLevel};
    } else {
        bool isLogLevelStillInList = (
                std::find(mapSubscribedDeadMotorsToNotificationLevels[motorId].begin(),
                          mapSubscribedDeadMotorsToNotificationLevels[motorId].end(),
                          logLevel
                ) != mapSubscribedDeadMotorsToNotificationLevels[motorId].end()
        );
        if (!isLogLevelStillInList) {
            mapSubscribedDeadMotorsToNotificationLevels[motorId].push_back(logLevel);
            ROS_INFO("ROS sends now ");
        }
    }
}