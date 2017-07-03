#include "roboy_error_detection/roboyErrorDetection.hpp"

int main(int argc, char* argv[])
{
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboyErrorDetection");
    }

    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
    RoboyErrorDetection handler(nh);

    int MOTOR_ID = 1;
    handler.listenForDeadMotor(1);

    while (ros::ok()) {
        ROS_INFO_THROTTLE(5, "Listening for Roboy Errors!");
    }

    return 0;
}