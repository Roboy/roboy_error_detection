#include "roboy_error_detection/roboyErrorDetection.hpp"

void handleDebugNotification(const roboy_communication_control::DebugNotification::ConstPtr &msg) {
    ROS_INFO("Received debug message with code %d | msg %s | object ID %s", msg->code, msg->msg.c_str(), msg->object.c_str());
}

void handleInfoNotification(const roboy_communication_control::InfoNotification::ConstPtr &msg) {
    ROS_INFO("Received info message with code %d | msg %s | object ID %s", msg->code, msg->msg.c_str(), msg->object.c_str());
}

void handleWarningNotification(const roboy_communication_control::WarningNotification::ConstPtr &msg) {
    ROS_INFO("Received warning message with code %d | msg %s | object ID %s", msg->code, msg->msg.c_str(), msg->object.c_str());
}

void handleErrorNotification(const roboy_communication_control::ErrorNotification::ConstPtr &msg) {
    ROS_INFO("Received error message with msg code %d | msg %s | object ID %s", msg->code, msg->msg.c_str(), msg->object.c_str());
}

void handleDangerNotification(const roboy_communication_control::DangerNotification::ConstPtr &msg) {
    ROS_INFO("Received danger message with msg code %d | msg %s | object ID %s", msg->code, msg->msg.c_str(), msg->object.c_str());
}

int main(int argc, char* argv[])
{
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboyErrorDetection");
    }

    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Publisher motor_publisher = nh->advertise<roboy_communication_middleware::MotorStatus>("/roboy/middleware/MotorStatus", 1000);
    RoboyErrorDetection handler(nh);

    // setup for listening to motor1 health
    int MOTOR_ID = 0;
    handler.listenForMotorHealth(MOTOR_ID);

    // listening to warning topic to get notified on a warning
    nh->subscribe(topicAddresses[DEBUG_LEVEL], 1, handleDebugNotification);
    nh->subscribe(topicAddresses[INFO_LEVEL], 1, handleInfoNotification);
    nh->subscribe(topicAddresses[WARNING_LEVEL], 1, handleWarningNotification);
    nh->subscribe(topicAddresses[ERROR_LEVEL], 1, handleErrorNotification);
    nh->subscribe(topicAddresses[DANGER_LEVEL], 1, handleDangerNotification);

    while (motor_publisher.getNumSubscribers() == 0) {
        ROS_INFO("Sleep one second until publisher is setup");
        usleep(1000*1000);
    }

    int counter = 0;
    while (ros::ok()) {
        ROS_INFO_THROTTLE(5, "Listening for Roboy Errors!");
        ros::spinOnce();
        ros::Duration(2).sleep();
        counter++;
    }

    return 0;
}