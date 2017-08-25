#pragma once

#include <ros/ros.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <roboy_communication_middleware/JointStatus.h>
#include <roboy_system_notification/roboySystemNotification.hpp>
#include <map>
#include "boost/variant.hpp"
#include <list>
#include <vector>
#include <tuple>
#include "common_utilities.hpp"

using namespace std;

typedef int ObjectID;
typedef uint32_t angle;
typedef uint16_t tacho;

typedef uint16_t NotificationInterval;
typedef tuple <NotificationInterval> MinimalNotificationData;
typedef tuple <angle, angle> JointAngleIntervalSubscriptionData;
typedef tuple <NotificationInterval, tacho, tacho> MotorTendentInconstenceSubscriptionData;
typedef boost::variant <MinimalNotificationData, JointAngleIntervalSubscriptionData, MotorTendentInconstenceSubscriptionData> NotificationData;
typedef std::map <NotificationLevel, NotificationData> NotificationDataMap;
typedef std::map <ObjectID, NotificationDataMap> ObjectToNotificationDataMap;

typedef NotificationCode (* MotorChallengeFunc)(const roboy_communication_middleware::MotorStatus::ConstPtr &msg, ObjectID objectID);
typedef NotificationCode (* JointChallengeFunc)(const roboy_communication_middleware::JointStatus::ConstPtr &msg, ObjectID objectID);

class RoboyErrorDetection {
public:
    RoboyErrorDetection(ros::NodeHandlePtr nh);

    /**
     * Listen if the motor with the given id is health
     * ROS topic.
     * @param motorId id of the motor, which should be observed
     * @param durationOfValidity interval, which defines the MIN sleep time in ms until the next message will be published. Set 0 to disable waiting a minimum time.
     * @param logLevel message level of the published message
     */
    void
    listenForMotorHealth(ObjectID motorId, NotificationInterval durationOfValidity = 5000,
                         NotificationLevel logLevel = WARNING_LEVEL);

    /**
     * Listen if the motor with the given id is dead. If the defined motor is really dead, a notification will be sent over
     * ROS topic.
     * @param motorId id of the motor, which should be observed
     * @param durationOfValidity interval, which defines the MIN sleep time in ms until the next message will be published. Set 0 to disable waiting a minimum time.
     * @param logLevel message level of the published message
     */
    void
    listenIfMotorIsDead(ObjectID motorId, NotificationInterval durationOfValidity = 5000,
                        NotificationLevel logLevel = WARNING_LEVEL);

    /**
     * Listen if the motor with the given id is alive. If the defined motor is alive, a notification will be sent over
     * ROS topic.
     * @param motorId id of the motor, which should be observed
     * @param durationOfValidity interval, which defines the MIN sleep time in ms until the next message will be published. Set 0 to disable waiting a minimum time.
     * @param logLevel message level of the published message
     */
    void
    listenIfMotorIsAlive(ObjectID motorId, NotificationInterval durationOfValidity = 10000,
                         NotificationLevel logLevel = INFO_LEVEL);

    /**
     * Listen if a relative angle of a specific joint is invalid. Invalid means in this case that the angle is not in the specified interval
     * [minAngle, maxAngle]. If this is the case, a message is published over ROS topic.
     * @param jointId id of the joint, which should be observed
     * @param minAngle minimum relative angle
     * @param maxAngle maximum relative angle
     * @param logLevel message level of the published message
     */
    void
    listenForInvalidRelativeJointAngle(ObjectID jointId, angle minAngle, angle maxAngle,
                                       NotificationLevel logLevel = WARNING_LEVEL);

    /**
     * Listen if a relative angle of a specific joint is invalid. Invalid means in this case that the angle is not in the specified interval
     * [minAngle, maxAngle]. If this is the case, a message is published over ROS topic.
     * @param jointId id of the joint, which should be observed
     * @param minAngle minimum relative angle
     * @param maxAngle maximum relative angle
     * @param durationOfValidity interval, which defines the MIN sleep time in ms until the next message will be published. Set 0 to disable waiting a minimum time.
     * @param logLevel message level of the published message
     */
    void
    listenForMotorTendentInconsistence(ObjectID jointId, tacho minTacho, tacho maxTacho,
                                       NotificationInterval durationOfValidity, NotificationLevel logLevel = WARNING_LEVEL);

    /**
     * Listen if a specific joint throws the error: too close or too far. If this is the case, a message is published over ROS topic.
     * @param jointId id of the joint, which should be observed
     * @param durationOfValidity interval, which defines the MIN sleep time in ms until the next message will be published. Set 0 to disable waiting a minimum time.
     * @param logLevel message level of the published message
     */
    void listenForJointMagnetStatus(ObjectID jointId, NotificationInterval durationOfValidity = 2000, NotificationLevel logLevel = WARNING_LEVEL);

    void handleMotorStatusErrors(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);

    void handleJointStatusErrors(const roboy_communication_middleware::JointStatus::ConstPtr &msg);

private:
    uint32_t MIN_DURATION_OF_VALIDITY = 50;
    enum SubscriptionType {
        UNKNOWN_SUBSCRIPTION = 0,
        MOTOR_HEALTH_SUBSCRIPTION,
        MOTOR_IS_DEAD_SUBSCRIPTION,
        MOTOR_IS_ALIVE_SUBSCRIPTION,
        JOINT_INVALID_REL_ANGLE_SUBSCRIPTION,
        MOTOR_IS_RUNNING_BUT_TENDENT_NOT_SUBSCRIPTION,
        JOINT_MAGNET_CHECK_SUBSCRIPTION
    };
    std::map <SubscriptionType, ObjectToNotificationDataMap> subscriptions;

    ros::Subscriber motorSub, jointSub;
    RoboySystemNotification notifier;

    static bool isLogLevelInList(std::list <NotificationLevel> notificationsList, NotificationLevel logLevel) {
        return std::find(notificationsList.begin(), notificationsList.end(), logLevel) != notificationsList.end();
    }

    std::map <ObjectID, std::map<NotificationLevel, ros::Time>> lastMotorHealthCheckTime,
            lastMotorTendentInconsistentCheck, lastJointMagnetCheck;

    void handleJointInvalidRelAngleCheck(const roboy_communication_middleware::JointStatus::ConstPtr &msg);
    void handleJointMagnetErrors(const roboy_communication_middleware::JointStatus::ConstPtr &msg);
    void handleMotorIsRunningButTendentNot(const roboy_communication_middleware::JointStatus::ConstPtr &msg);

    void handleMotorHealthCheck(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);
    void handleMotorIsAliveCheck(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);
    void handleMotorIsDeadCheck(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);

    void subscribeToMotorStatus(ros::NodeHandlePtr nh);
    void subscribeToJointStatus(ros::NodeHandlePtr nh);

    void publishMessage(NotificationLevel level, NotificationCode notificationCode, uint16_t objectId);

    void addSubscription(ObjectID objectId, SubscriptionType subscriptionType, NotificationData notificationData,
                         NotificationLevel notificationLevel);

    template<typename NotificationDataType>
    void handleMotorStatusAfterInterval(const roboy_communication_middleware::MotorStatus::ConstPtr &msg,
                                                        SubscriptionType subscriptionType,
                                                        MotorChallengeFunc challengeFunc
    );

    uint32 getRealDurationOfValidity(uint32_t durationOfValidity) {
        return durationOfValidity < 50 ? durationOfValidity : MIN_DURATION_OF_VALIDITY;
    }
};