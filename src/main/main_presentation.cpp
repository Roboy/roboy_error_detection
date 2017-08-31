#include "roboy_error_detection/roboyErrorDetection.hpp"
#include "roboy_error_detection/main_common.hpp"

void handleDebugNotification(const roboy_communication_control::DebugNotification::ConstPtr &msg) {
    ROS_INFO("Received debug message with code %d | msg %s | extra %s | object ID %s | duration %d", msg->code, msg->msg.c_str(), msg->extra.c_str(), msg->object.c_str(), msg->validityDuration);
}

void handleInfoNotification(const roboy_communication_control::InfoNotification::ConstPtr &msg) {
    ROS_INFO("Received info message with code %d | msg %s | extra %s | object ID %s | duration %d", msg->code, msg->msg.c_str(), msg->extra.c_str(), msg->object.c_str(), msg->validityDuration);
}

void handleWarningNotification(const roboy_communication_control::WarningNotification::ConstPtr &msg) {
    ROS_INFO("Received warning message with code %d | msg %s | extra %s | object ID %s | duration %d", msg->code, msg->msg.c_str(), msg->extra.c_str(), msg->object.c_str(), msg->validityDuration);
}

void handleErrorNotification(const roboy_communication_control::ErrorNotification::ConstPtr &msg) {
    ROS_INFO("Received error message with msg code %d | msg %s | extra %s | object ID %s | duration %d", msg->code, msg->msg.c_str(), msg->extra.c_str(), msg->object.c_str(), msg->validityDuration);
}

void handleDangerNotification(const roboy_communication_control::DangerNotification::ConstPtr &msg) {
    ROS_INFO("Received danger message with msg code %d | msg %s | extra %s | object ID %s | duration %d", msg->code, msg->msg.c_str(), msg->extra.c_str(), msg->object.c_str(), msg->validityDuration);
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
    initSystemNotificationSubscriber(nh);

    while (motor_publisher.getNumSubscribers() == 0) {
        ROS_INFO("Sleep one second until publisher is setup");
        usleep(1000*1000);
    }

    int counter = 0;
    ros::Time begin = ros::Time::now();
    ros::Time now;
    while (ros::ok()) {
        ROS_INFO_THROTTLE(5, "Listening for Roboy Errors!");
        now = ros::Time::now();
        if ((now - begin).toSec() > 11) {
            // send an example motor is dead command
            // ROS_INFO("Sends motor is DEAD command");
            sendMotorHealthStateMsg(motor_publisher, false);
        } else {
            // send an example motor is alive command
            // ROS_INFO("Sends motor is ALIVE command");
            sendMotorHealthStateMsg(motor_publisher, true);
        }

        ros::spinOnce();
        ros::Duration(2).sleep();
        counter++;
    }

    return 0;
}