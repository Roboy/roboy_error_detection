#include "roboy_error_detection/roboyErrorDetection.hpp"

int main(int argc, char* argv[])
{
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboyErrorDetection");
    }
    RoboyErrorDetection handler;
    // handler.

    return 0;
}