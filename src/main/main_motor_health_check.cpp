#include "roboy_error_detection/roboyErrorDetection.hpp"
#include "roboy_error_detection/main_common.hpp"

int main(int argc, char *argv[]) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboyErrorDetection");
    }

    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Publisher motor_publisher = nh->advertise<roboy_communication_middleware::MotorStatus>(
            "/roboy/middleware/MotorStatus", 1000);
    RoboyErrorDetection handler(nh);

    // setup for listening to motor1 health
    ObjectID MOTOR_ID = 0;
    handler.listenForMotorHealth(MOTOR_ID);

    // listening to warning topic to get notified on a warning
    initSystemNotificationSubscriber(nh);

    while (motor_publisher.getNumSubscribers() == 0) {
        ROS_INFO("Sleep one second until publisher is setup");
        usleep(1000 * 1000);
    }

    int counter = 0;
    while (ros::ok()) {
        if (counter % 2 == 0) {
            // send an example motor is dead command
            ROS_INFO("Sends motor is DEAD command");
            sendMotorHealthStateMsg(motor_publisher, false);
        } else {
            // send an example motor is alive command
            ROS_INFO("Sends motor is ALIVE command");
            sendMotorHealthStateMsg(motor_publisher, true);
        }

        ROS_INFO_THROTTLE(5, "Listening for Roboy Errors!");
        ros::spinOnce();
        ros::Duration(2).sleep();
        counter++;
    }

    return 0;
}