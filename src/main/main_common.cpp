#include "roboy_error_detection/roboyErrorDetection.hpp"
#include "roboy_error_detection/main_common.hpp"

void handleDebugNotification(const roboy_communication_control::DebugNotification::ConstPtr &msg) {
    ROS_INFO("Received debug message with code %d | msg %s | extra %s | object ID %s", msg->code,
             msg->msg.c_str(), msg->extra.c_str(), msg->object.c_str());
}

void handleInfoNotification(const roboy_communication_control::InfoNotification::ConstPtr &msg) {
    ROS_INFO("Received info message with msg code %d | msg %s | extra %s | object ID %s", msg->code,
             msg->msg.c_str(), msg->extra.c_str(), msg->object.c_str());
}

void handleWarningNotification(const roboy_communication_control::WarningNotification::ConstPtr &msg) {
    ROS_INFO("Received warning message with msg code %d | msg %s | extra %s | object ID %s", msg->code,
             msg->msg.c_str(), msg->extra.c_str(), msg->object.c_str());
}

void handleErrorNotification(const roboy_communication_control::ErrorNotification::ConstPtr &msg) {
    ROS_INFO("Received error message with msg code %d | msg %s | extra %s | object ID %s", msg->code,
             msg->msg.c_str(), msg->extra.c_str(), msg->object.c_str());
}

void handleDangerNotification(const roboy_communication_control::DangerNotification::ConstPtr &msg) {
    ROS_INFO("Received danger message with msg code %d | msg %s | extra %s | object ID %s", msg->code,
             msg->msg.c_str(), msg->extra.c_str(), msg->object.c_str());
}

void sendMotorHealthStateMsg(ros::Publisher publisher, bool isMotorHealthy) {
    int dummyMsgID = 1;
    int dummyPwmRef = 0;
    int dummyPosition = 0;
    short dummyVelocity = 0;
    short dummyDisplacement = 0;
    short currentValue = isMotorHealthy ? 1 : 0;

    roboy_communication_middleware::MotorStatus msg;
    msg.id = dummyMsgID;
    msg.pwmRef = {dummyPwmRef};
    msg.position = {dummyPosition};
    msg.velocity = {dummyVelocity};
    msg.displacement = {dummyDisplacement};
    msg.current = {currentValue};

    publisher.publish(msg);
}

void sendJointAngle(ros::Publisher publisher, uint32_t absAngle, uint32_t relAngle, uint32_t tacho,
                    uint8_t agcGain, bool tooFar, bool tooClose, int msgId) {
    roboy_communication_middleware::JointStatus msg;
    msg.id = msgId;
    msg.absAngles = {absAngle};
    msg.relAngles = {relAngle};
    msg.tacho = {tacho};
    msg.agcGain = {agcGain};
    msg.tooFar = {tooFar};
    msg.tooClose = {tooClose};

    publisher.publish(msg);
}

void initSystemNotificationSubscriber(ros::NodeHandlePtr nh) {
    nh->subscribe(topicAddresses[DEBUG_LEVEL], 1, handleDebugNotification);
    nh->subscribe(topicAddresses[INFO_LEVEL], 1, handleInfoNotification);
    nh->subscribe(topicAddresses[WARNING_LEVEL], 1, handleWarningNotification);
    nh->subscribe(topicAddresses[ERROR_LEVEL], 1, handleErrorNotification);
    nh->subscribe(topicAddresses[DANGER_LEVEL], 1, handleDangerNotification);
}