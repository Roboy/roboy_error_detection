#include "roboy_error_detection/roboyErrorDetection.hpp"
#include "roboy_error_detection/main_common.hpp"
#include <stdlib.h> // required to generate dummy joint value

int main(int argc, char *argv[]) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboyErrorDetection");
    }

    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
    ros::Publisher joint_publisher = nh->advertise<roboy_communication_middleware::JointStatus>(
            "/roboy/middleware/JointStatus", 1000);
    ros::Publisher motor_publisher = nh->advertise<roboy_communication_middleware::MotorStatus>(
            "/roboy/middleware/MotorStatus", 1000);
    RoboyErrorDetection handler(nh);

    ObjectID motorId = 0;
    ObjectID jointId = 0;
    tacho minTacho = 0 + 1;
    tacho maxTacho = 1023 - 1;
    handler.listenForMotorTendonInconsistence(motorId, jointId, minTacho, maxTacho);

    // listening to warning topic to get notified on a warning
    initSystemNotificationSubscriber(nh);

    while (motor_publisher.getNumSubscribers() == 0 || joint_publisher.getNumSubscribers() == 0) {
        ROS_INFO("Sleep one second until publisher is setup");
        usleep(1000 * 1000);
    }

    int counter = 0;
    int tacho;
    int min = 1;
    int max = 3;
    while (ros::ok()) {
        bool randomTooCloseParameter = rand() % 1 == 0;
        bool randomTooFarParameter = rand() % 1 == 0;
        ROS_INFO("Create too far angle parameter: %d and %d", randomTooCloseParameter, randomTooFarParameter);

        int randomCommandIndex = min + (rand() % static_cast<int>(max - min + 1));
        if (randomCommandIndex == 1) {
            ROS_INFO("Motor is not running -> should not fire");
            sendVelocityStateMsg(motor_publisher, 0);
            tacho = 10;
        } else if (randomCommandIndex == 2) {
            ROS_INFO("Motor is running and joint tacho = 0 --> should not fire");
            sendVelocityStateMsg(motor_publisher, 10);
            tacho = 0;
        } else {
            ROS_INFO("Motor is running and joint tacho > 0");
            sendVelocityStateMsg(motor_publisher, 10);
            tacho = 10;
        }
        sendJointAngle(joint_publisher, 0, 0, tacho, 0, false, false);

        ros::spinOnce();
        ros::Duration(3).sleep();
        counter++;
    }

    return 0;
}