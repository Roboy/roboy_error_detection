#include "roboy_error_detection/roboyErrorDetection.hpp"
#include <functional>
#include <typeinfo>
#include <cassert>

RoboyErrorDetection::RoboyErrorDetection(ros::NodeHandlePtr nh) {
    ROS_DEBUG("Calls constructor");
    notifier.setNodeHandler(nh);

    this->subscribeToMotorStatus(nh);
    this->subscribeToJointStatus(nh);
};

void RoboyErrorDetection::listenForMotorHealth(ObjectID motorId, NotificationInterval durationOfValidity,
                                               NotificationLevel logLevel) {
    // TODO: check for invalid input
    // TODO: check what happens if subscription still exists (existing error subscription with 2000ms, what happens if another come with 1000ms?)

    MinimalNotificationData notificationData(durationOfValidity);
    addSubscription(motorId, MOTOR_HEALTH_SUBSCRIPTION, notificationData, logLevel);
}

void RoboyErrorDetection::listenIfMotorIsDead(ObjectID motorId, NotificationInterval durationOfValidity,
                                              NotificationLevel logLevel) {
    // TODO: check for invalid input
    // TODO: check what happens if subscription still exists (existing error subscription with 2000ms, what happens if another come with 1000ms?)

    MinimalNotificationData notificationData(durationOfValidity);
    addSubscription(motorId, MOTOR_IS_DEAD_SUBSCRIPTION, notificationData, logLevel);
}

void RoboyErrorDetection::listenIfMotorIsAlive(ObjectID motorId, NotificationInterval durationOfValidity,
                                               NotificationLevel logLevel) {
    // TODO: check for invalid input
    // TODO: check what happens if subscription still exists (existing error subscription with 2000ms, what happens if another come with 1000ms?)

    MinimalNotificationData notificationData(durationOfValidity);
    addSubscription(motorId, MOTOR_IS_ALIVE_SUBSCRIPTION, notificationData, logLevel);
}

void RoboyErrorDetection::listenForInvalidRelativeJointAngle(ObjectID jointId, angle minAngle, angle maxAngle,
                                                             NotificationLevel logLevel) {
    // TODO: check for invalid input
    // TODO: check what happens if subscription still exists (existing error subscription with 2000ms, what happens if another come with 1000ms?)

    JointAngleIntervalSubscriptionData notificationData(minAngle, maxAngle);
    addSubscription(jointId, JOINT_INVALID_REL_ANGLE_SUBSCRIPTION, notificationData, logLevel);
}

void RoboyErrorDetection::listenForJointMagnetStatus(ObjectID jointId, NotificationInterval durationOfValidity,
                                                     NotificationLevel logLevel) {
    // TODO: check for invalid input

    MinimalNotificationData notificationData(durationOfValidity);
    addSubscription(jointId, JOINT_MAGNET_CHECK_SUBSCRIPTION, notificationData, logLevel);
}

void RoboyErrorDetection::listenForMotorTendonInconsistence(ObjectID motorId, ObjectID jointId,
                                                             NotificationInterval durationOfValidity, tacho minTacho,
                                                             tacho maxTacho, NotificationLevel logLevel) {
    // TODO: check for invalid input

    MotorTendonInconsistenceNotificationData notificationData(jointId, durationOfValidity, minTacho, maxTacho);
    addSubscription(motorId, MOTOR_IS_RUNNING_BUT_TENDON_NOT_SUBSCRIPTION, notificationData, logLevel);
}

void RoboyErrorDetection::addSubscription(ObjectID objectId, SubscriptionType subscriptionType,
                                          NotificationData notificationData, NotificationLevel notificationLevel) {
    if (subscriptions[subscriptionType].find(objectId) ==
        subscriptions[subscriptionType].end()) {
        subscriptions[subscriptionType][objectId] = {};
    }

    subscriptions[MOTOR_HEALTH_SUBSCRIPTION][objectId][notificationLevel] = notificationData;
}

void RoboyErrorDetection::handleMotorStatusErrors(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    latestMotorStatus = *msg;
    receivedMotorStatus = true;

    handleMotorHealthCheck(msg);
    // handleMotorIsAliveCheck(msg);
    // handleMotorIsDeadCheck(msg);
}

void RoboyErrorDetection::handleJointStatusErrors(const roboy_communication_middleware::JointStatus::ConstPtr &msg) {
    latestJointStatus = *msg;
    receivedJointStatus = true;

    handleJointInvalidRelAngleCheck(msg);
    handleMotorIsRunningButTendonNot(msg);
    handleJointMagnetErrors(msg);
}

void RoboyErrorDetection::subscribeToMotorStatus(ros::NodeHandlePtr nh) {
    motorSub = nh->subscribe("/roboy/middleware/MotorStatus", 1000, &RoboyErrorDetection::handleMotorStatusErrors,
                             this);
    ROS_DEBUG("Subscribed to /roboy/middleware/MotorStatus");
}

void RoboyErrorDetection::subscribeToJointStatus(ros::NodeHandlePtr nh) {
    jointSub = nh->subscribe("/roboy/middleware/JointStatus", 1000, &RoboyErrorDetection::handleJointStatusErrors,
                             this);
    ROS_DEBUG("Subscribed to /roboy/middleware/JointStatus");
}

void RoboyErrorDetection::handleMotorHealthCheck(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    ROS_DEBUG("Check for motor health status");

    MotorChallengeFunc challenge = [](const roboy_communication_middleware::MotorStatus::ConstPtr &msg,
                                      ObjectID objectId) {
        bool isMotorDead = msg->current[objectId] == 0;

        return isMotorDead ? MOTOR_DEAD_NOTIFICATION : MOTOR_ALIVE_NOTIFICATION;
    };

    handleMotorStatusAfterInterval<MinimalNotificationData>(msg, MOTOR_HEALTH_SUBSCRIPTION, challenge);
}

void RoboyErrorDetection::handleMotorIsDeadCheck(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    ROS_DEBUG("Check for motor is dead check");

    MotorChallengeFunc challenge = [](const roboy_communication_middleware::MotorStatus::ConstPtr &msg,
                                      ObjectID objectId) {
        bool isMotorDead = msg->current[objectId] == 0;

        return isMotorDead ? MOTOR_DEAD_NOTIFICATION : UNDEFINED_NOTIFICATION;
    };

    handleMotorStatusAfterInterval<MinimalNotificationData>(msg, MOTOR_IS_DEAD_SUBSCRIPTION, challenge);
}

void RoboyErrorDetection::handleMotorIsAliveCheck(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    ROS_DEBUG("Check for motor alive status");

    MotorChallengeFunc challenge = [](const roboy_communication_middleware::MotorStatus::ConstPtr &msg,
                                      ObjectID objectId) {
        bool isMotorDead = msg->current[objectId] == 0;

        return !isMotorDead ? MOTOR_ALIVE_NOTIFICATION : UNDEFINED_NOTIFICATION;
    };

    handleMotorStatusAfterInterval<MinimalNotificationData>(msg, MOTOR_IS_ALIVE_SUBSCRIPTION, challenge);
}

void
RoboyErrorDetection::handleJointInvalidRelAngleCheck(const roboy_communication_middleware::JointStatus::ConstPtr &msg) {
    // check all subscribed motors if we should send/check for a message
    for (auto const &jointEntry : subscriptions[JOINT_INVALID_REL_ANGLE_SUBSCRIPTION]) {
        ObjectID jointId = jointEntry.first;
        ROS_INFO("Found entry with joint ID %d", jointId);

        // loop and process all subscriptions for this motor
        for (auto const &jointSubscriptions : jointEntry.second) {
            NotificationLevel lvl = jointSubscriptions.first;
            JointAngleIntervalSubscriptionData subscriptionData = boost::get<JointAngleIntervalSubscriptionData>(
                    jointSubscriptions.second);

            angle currentJointValue = msg->relAngles[jointId];
            angle minAngle = std::get<0>(subscriptionData), maxAngle = std::get<1>(subscriptionData);

            // check if angle is a valid value
            if (minAngle > currentJointValue || maxAngle < currentJointValue) {
                publishMessage(lvl, JOINT_INVALID_REL_ANGLE_NOTIFICATION, jointId, getRealDurationOfValidity(0));
            }
        }
    }
}

void
RoboyErrorDetection::handleMotorIsRunningButTendonNot(
        const roboy_communication_middleware::JointStatus::ConstPtr &msg
) {
    // only allow to run this method if we still received at least one joint and one motor message
    if (receivedMotorStatus || receivedJointStatus) {
        return;
    }

    for (auto const &motorEntry : subscriptions[MOTOR_IS_RUNNING_BUT_TENDON_NOT_SUBSCRIPTION]) {
        ObjectID motorId = motorEntry.first;

        if (lastMotorTendonInconsistentCheck.find(motorId) == lastMotorTendonInconsistentCheck.end()) {
            lastMotorTendonInconsistentCheck[motorId] = {};
        }

        // only allow method if motor is running
        if (latestMotorStatus.velocity[motorId] == 0) {
            continue;
        }

        // loop and process all subscriptions for this motor
        for (auto const &motorSubscriptions : motorEntry.second) {
            NotificationLevel lvl = motorSubscriptions.first;
            MotorTendonInconsistenceNotificationData subscriptionData = boost::get<MotorTendonInconsistenceNotificationData>(
                    motorSubscriptions.second);

            ObjectID jointId = std::get<0>(subscriptionData);

            if (lastMotorTendonInconsistentCheck[motorId].find(lvl) ==
                lastMotorTendonInconsistentCheck[motorId].end()) {
                lastMotorTendonInconsistentCheck[motorId][lvl] = ros::Time(0); // set invalid time
            }

            // determine time since last health check for given motor
            float timeSinceLastMotorTendonInconsistentCheck = 0.0;
            if (lastMotorTendonInconsistentCheck[motorId][lvl].isValid()) {
                timeSinceLastMotorTendonInconsistentCheck =
                        (ros::Time::now() - lastMotorTendonInconsistentCheck[motorId][lvl]).toSec() *
                        1000; // time in ms
            }

            tacho currentTacho = msg->tacho[jointId];
            NotificationInterval durationOfValidity = std::get<1>(subscriptionData);
            tacho minTacho = std::get<2>(subscriptionData), maxTacho = std::get<3>(subscriptionData);

            if (!lastMotorTendonInconsistentCheck[motorId][lvl].isValid() || durationOfValidity == 0 ||
                durationOfValidity < timeSinceLastMotorTendonInconsistentCheck) {
                if (minTacho > currentTacho || maxTacho < currentTacho) {
                    publishMessage(lvl, MOTOR_IS_RUNNING_BUT_TENDON_NOT_NOTIFICATION, motorId,
                                   getRealDurationOfValidity(durationOfValidity));
                    lastMotorTendonInconsistentCheck[motorId][lvl] = ros::Time::now();
                }
            }
        }
    }
}

void RoboyErrorDetection::handleJointMagnetErrors(const roboy_communication_middleware::JointStatus::ConstPtr &msg) {
    for (auto const &jointEntry : subscriptions[JOINT_MAGNET_CHECK_SUBSCRIPTION]) {
        ObjectID jointId = jointEntry.first;

        if (lastJointMagnetCheck.find(jointId) == lastJointMagnetCheck.end()) {
            lastJointMagnetCheck[jointId] = {};
        }

        // loop and process all subscriptions for this motor
        for (auto const &jointSubscriptions : jointEntry.second) {
            NotificationLevel lvl = jointSubscriptions.first;
            MinimalNotificationData subscriptionData =
                    boost::get<MinimalNotificationData>(jointSubscriptions.second);

            if (lastJointMagnetCheck[jointId].find(lvl) == lastJointMagnetCheck[jointId].end()) {
                lastJointMagnetCheck[jointId][lvl] = ros::Time(0); // set invalid time
            }

            // determine time since last health check for given motor
            float timeSinceLastJointMagnetCheck = 0.0;
            if (lastJointMagnetCheck[jointId][lvl].isValid()) {
                timeSinceLastJointMagnetCheck =
                        (ros::Time::now() - lastJointMagnetCheck[jointId][lvl]).toSec() * 1000; // time in ms
            }

            tacho currentTacho = msg->tacho[jointId];
            NotificationInterval durationOfValidity = std::get<0>(subscriptionData);

            if (!lastJointMagnetCheck[jointId][lvl].isValid() || durationOfValidity == 0 ||
                durationOfValidity < timeSinceLastJointMagnetCheck) {
                if (msg->tooFar[jointId]) {
                    publishMessage(lvl, JOINT_TOO_FAR_NOTIFICATION, jointId,
                                   getRealDurationOfValidity(durationOfValidity));
                } else if (msg->tooClose[jointId]) {
                    publishMessage(lvl, JOINT_TOO_CLOSE_NOTIFICATION, jointId,
                                   getRealDurationOfValidity(durationOfValidity));
                }
            }
            lastJointMagnetCheck[jointId][lvl] = ros::Time::now();
        }
    }
}

template<typename NotificationDataType>
void RoboyErrorDetection::handleMotorStatusAfterInterval(
        const roboy_communication_middleware::MotorStatus::ConstPtr &msg,
        SubscriptionType subscriptionType,
        MotorChallengeFunc challengeFunc
) {
    // check all subscribed motors if we should send a health message
    for (auto const &motorEntry : subscriptions[subscriptionType]) {
        ObjectID motorId = motorEntry.first;
        ROS_INFO("Found entry with motor ID %d", motorId);

        if (lastMotorHealthCheckTime.find(motorId) == lastMotorHealthCheckTime.end()) {
            lastMotorHealthCheckTime[motorId] = {};
        }

        // loop and process all subscriptions for this motor
        for (auto const &motorSubscriptions : motorEntry.second) {
            NotificationLevel lvl = motorSubscriptions.first;
            NotificationDataType subscriptionData;
            try {
                boost::get<NotificationDataType>(motorSubscriptions.second);
            } catch(std::exception &e) {
                continue;
            }

            if (lastMotorHealthCheckTime[motorId].find(lvl) == lastMotorHealthCheckTime[motorId].end()) {
                lastMotorHealthCheckTime[motorId][lvl] = ros::Time(0); // set invalid time
            }

            // determine time since last health check for given motor
            float timeSinceLastMotorHealthCheck = 0.0;
            if (lastMotorHealthCheckTime[motorId][lvl].isValid()) {
                timeSinceLastMotorHealthCheck =
                        (ros::Time::now() - lastMotorHealthCheckTime[motorId][lvl]).toSec() * 1000; // time in ms
            }

            NotificationInterval durationOfValidity = std::get<0>(subscriptionData);
            ROS_DEBUG("Time since last publishment: %f and required interval %d", timeSinceLastMotorHealthCheck,
                      durationOfValidity);
            if (!lastMotorHealthCheckTime[motorId][lvl].isValid() || durationOfValidity == 0 ||
                durationOfValidity < timeSinceLastMotorHealthCheck) {
                NotificationCode notificationCode = challengeFunc(msg, motorId);

                // do we really want to send it?
                if (notificationCode != UNDEFINED_NOTIFICATION) {
                    publishMessage(lvl, notificationCode, motorId, getRealDurationOfValidity(durationOfValidity));

                    lastMotorHealthCheckTime[motorId][lvl] = ros::Time::now(); // update time of last motor check --> is current now
                }

            }
        }
    }
}

void
RoboyErrorDetection::publishMessage(NotificationLevel level, NotificationCode notificationCode, uint16_t objectId,
                                    uint32_t durationOfValidity) {
    switch (level) {
        case DANGER_LEVEL:
            notifier.sendDangerMessage(notificationCode, notificationMessages[notificationCode],
                                       notificationExtraMessages[notificationCode], std::to_string(objectId),
                                       durationOfValidity);
            break;
        case ERROR_LEVEL:
            notifier.sendErrorMessage(notificationCode, notificationMessages[notificationCode],
                                      notificationExtraMessages[notificationCode], std::to_string(objectId),
                                      durationOfValidity);
            break;
        case WARNING_LEVEL:
            notifier.sendWarningMessage(notificationCode, notificationMessages[notificationCode],
                                        notificationExtraMessages[notificationCode], std::to_string(objectId),
                                        durationOfValidity);
            break;
        case INFO_LEVEL:
            notifier.sendInfoMessage(notificationCode, notificationMessages[notificationCode],
                                     notificationExtraMessages[notificationCode], std::to_string(objectId),
                                     durationOfValidity);
            break;
        case DEBUG_LEVEL:
            notifier.sendDebugMessage(notificationCode, notificationMessages[notificationCode],
                                      notificationExtraMessages[notificationCode], std::to_string(objectId),
                                      durationOfValidity);
            break;
        default:
            notifier.sendWarningMessage(notificationCode, notificationMessages[UNDEFINED_NOTIFICATION],
                                        notificationExtraMessages[notificationCode], std::to_string(objectId),
                                        durationOfValidity);
    }
}