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
    RoboyErrorDetection handler(nh);

    // setup for listening to joint1 health
    int JOINT_ID = 0;
    handler.listenForJointMagnetStatus(JOINT_ID);

    // listening to warning topic to get notified on a warning
    initSystemNotificationSubscriber(nh);

    while (joint_publisher.getNumSubscribers() == 0) {
        ROS_INFO("Sleep one second until publisher is setup");
        usleep(1000 * 1000);
    }

    int counter = 0;
    while (ros::ok()) {
        bool randomTooCloseParameter = rand() % 1 == 0;
        bool randomTooFarParameter = rand() % 1 == 0;
        ROS_INFO("Create too far angle parameter: %d and %d", randomTooCloseParameter, randomTooFarParameter);
        sendJointAngle(joint_publisher, 0, 0, 0, 0, randomTooFarParameter, randomTooCloseParameter);

        ROS_INFO_THROTTLE(5, "Listening for Roboy Errors!");
        ros::spinOnce();
        ros::Duration(2).sleep();
        counter++;
    }

    return 0;
}