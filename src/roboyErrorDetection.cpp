#include "roboy_error_detection/roboyErrorDetection.hpp"
#include <functional>   // std::bind

RoboyErrorDetection::RoboyErrorDetection() {
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    motors_sub = nn.subscribe("/roboy/middleware/MotorStatus", 1, &RoboyErrorDetection::handleMotorStatusErrors, this);
};

void RoboyErrorDetection::handleMotorStatusErrors(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
    // TODO: handle motor states and publish on error
    bool dead = false; // TODO: define if motor is dead: https://github.com/Roboy/myoFPGA/blob/5de1e1751091c34a3da620666ded3e0603e280d3/myoFPGA/src/interface/src/main_window.cpp#L187
    bool shouldCheckMotorDeadStatus = false; // TODO: check if motor id is in sub_dead_motors

    if (dead && shouldCheckMotorDeadStatus) {
        // TODO: send message according to the log level
    }
}

void RoboyErrorDetection::listenForDeadMotor(int motorId, int logLevel=WARNING_LEVEL) {
    // TODO: check for invalid input

    std::tuple<int,int> motorLogCombination (motorId,logLevel);
    sub_dead_motors.push_back(motorLogCombination);
}