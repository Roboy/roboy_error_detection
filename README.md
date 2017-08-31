# Roboy Error Detection Module

This module helps you to send system messages in a very easy way.

## Installation

### 0. Ensure that ROS master is running

For this package, you need a ROS master running. Therefore, you have to run it before processing the next steps. 
If you run your ROS core on the same host, you can run it with: 

```bash
roscore &
```

You can find more information about the ROS core here: [http://wiki.ros.org/roscore](http://wiki.ros.org/roscore)

### 1. Add dependency `roboy_communication` to workspace
```bash
cd YOUR_WORKSPACE_DIR/src
git clone -b feature/error-detection-msgs https://github.com/CapChrisCap/roboy_communication
```

### 2. Add dependency `roboy_system_notification` to workspace
```bash
cd YOUR_WORKSPACE_DIR/src
git clone -b feature/error-detection-msgs https://github.com/CapChrisCap/roboy_system_notification
```

### 4. Add this repository to your workspace
```bash
cd YOUR_WORKSPACE_DIR/src
git clone https://github.com/CapChrisCap/roboy_error_detection
```

### 5. Important! Add packages to setup required connections

This package subscribes to the following topics: 
 - `/roboy/middleware/MotorStatus`
 - `/roboy/middleware/JointStatus`
 
The package can receive over these topics the current angle of the joints as well as the current status of the motors. To make this run, 
we need at Roboy the following packages: 
```bash
cd YOUR_WORKSPACE_DIR/src
git clone -b feature/error-detection-msgs https://github.com/CapChrisCap/roboy_communication
git clone -b master https://github.com/Roboy/roboy_managing_node # responsible for setting up all Roboy ROS controllers to publish the topics
git clone -b master https://github.com/Roboy/roboy_interface # required to start the openPowerLink connection
git clone -b master https://github.com/Roboy/common_utilities # required because it provides general utitilities for the packages above
```

### 6. Source setup files
For a ROS Kinetic: 
```bash
cd YOUR_WORKSPACE_DIR
source /opt/ros/kinetic/setup.bash 
```

### 7. Build the package
```bash
cd YOUR_WORKSPACE_DIR
catkin_make
source devel/setup.bash
```

### 8. Check if topics exist
To check if the installation was successful, execute the test main command 
of this repository to check, whether the topics are published: 
```bash
rosrun roboy_error_detection roboy_error_detection_test
```

After that, you should see something like this: 
```txt
[ INFO] [1499760654.670448800]: Set ROS Node handler
[ INFO] [1499760654.708822500]: Sends motor is DEAD command
[ INFO] [1499760654.708916600]: Listening for Roboy Errors!
[ INFO] [1499760654.711709500]: Received motor health check message
[ INFO] [1499760654.712072800]: Found entry with motor ID 0
[ INFO] [1499760654.712662900]: Time since last publishment: 1499760689152.000000 and required interval 5000
[ INFO] [1499760654.712956600]: Sent warning message!
[ INFO] [1499760656.713399500]: Sends motor is ALIVE command
[ INFO] [1499760658.713792700]: Sends motor is DEAD command
[ INFO] [1499760658.714690400]: Received motor health check message
....
```

## Usage

### Setup real Roboy environment

 - Ensure that ROS core is running: `roscore &`
 - Start openPowerLink connection with the user interface `rosrun interface interface` (for more information see https://devanthro.atlassian.net/wiki/display/CO/Tutorial+of+how+to+control+the+PaBiLegs)
 
### Include the roboy_error_detection package in your workspace

#### a) Update `CMakeLists.txt` file

Now, add this package name (normally, it should be `roboy_error_detection`) to the 
`CMakeLists.txt` file of your project. Finally, it should look similar to this: 

```txt
[...]

find_package([...] roboy_error_detection [...])
catkin_package([...] roboy_error_detection [...])
[...]
```

#### b) Update `package.xml` file

Next, add the package name (normally, it should be `roboy_error_detection`) to your `package.json` file 
to mark it as a build and run dependency. After that, it will look similar to this: 

```txt
[...]
  <build_depend>roboy_error_detection</run_depend>
[...]
  <run_depend>roboy_error_detection</run_depend>
[...]
```

### 2. Use the `RoboyErrorDetection` class within your project

#### API `RoboyErrorDetection`

##### void listenForMotorHealth(MotorID motorId, NotificationInterval minPublishIntervalInMs = 5000, NotificationLevel logLevel = WARNING_LEVEL)

The method makes the program listen for the motors health. You can also set the parameter `minPublishIntervalInMs` to define the interval from one health check to the next one. 
Finally, you can define over the logLevel over which channel the health check result should be published. 

 - The `motorId` is the motor id, which identifies the motor you want to observe
 - The `minPublishIntervalInMs` Minimum time in milliseconds from one health check to the next health check of this motor
 - The `logLevel` parameter defines the system notification level, which also determines over which topic the health check message is published ([see all log levels](https://github.com/CapChrisCap/roboy_system_notification/blob/master/include/roboy_system_notification/common_utilities.hpp#L12))

Output: 

 - the message ID is generated randomly
 - possible output formats, depending on the log level: 
   - [DEBUG_LEVEL](https://github.com/CapChrisCap/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/DebugNotification.msg)
   - [INFO_LEVEL](https://github.com/CapChrisCap/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/InfoNotification.msg)
   - [WARNING_LEVEL](https://github.com/CapChrisCap/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/WarningNotification.msg)
   - [ERROR_LEVEL](https://github.com/CapChrisCap/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/ErrorNotification.msg)
   - [DANGER_LEVEL](https://github.com/CapChrisCap/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/DangerNotification.msg)
 - message code is either
   - [Motor is Dead ID](https://github.com/CapChrisCap/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L3)
   - or [Motor is Alive ID](https://github.com/CapChrisCap/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L4)
 - the `msg` parameter is received from [this map, depending on the message code](https://github.com/CapChrisCap/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L7)
 - the `objectId` is the ID of the observed motor

#### Example: 

```cpp
#include "roboy_error_detection/roboyErrorDetection.hpp"

void handleDebugNotification(const roboy_communication_control::DebugNotification::ConstPtr &msg) {
    ROS_INFO("Received debug message with msg ID %d | code %d | msg %s | object ID %d", msg->id, msg->code, msg->msg.c_str(), msg->objectId);
}

void handleInfoNotification(const roboy_communication_control::InfoNotification::ConstPtr &msg) {
    ROS_INFO("Received info message with msg ID %d | code %d | msg %s | object ID %d", msg->id, msg->code, msg->msg.c_str(), msg->objectId);
}

void handleWarningNotification(const roboy_communication_control::WarningNotification::ConstPtr &msg) {
    ROS_INFO("Received warning message with msg ID %d | code %d | msg %s | object ID %d", msg->id, msg->code, msg->msg.c_str(), msg->objectId);
}

void handleErrorNotification(const roboy_communication_control::ErrorNotification::ConstPtr &msg) {
    ROS_INFO("Received error message with msg ID %d | code %d | msg %s | object ID %d", msg->id, msg->code, msg->msg.c_str(), msg->objectId);
}

void handleDangerNotification(const roboy_communication_control::DangerNotification::ConstPtr &msg) {
    ROS_INFO("Received danger message with msg ID %d | code %d | msg %s | object ID %d", msg->id, msg->code, msg->msg.c_str(), msg->objectId);
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
    msg.pwmRef = { dummyPwmRef };
    msg.position = { dummyPosition };
    msg.velocity = { dummyVelocity };
    msg.displacement = { dummyDisplacement };
    msg.current = { currentValue };

    publisher.publish(msg);
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
        if (counter %2 == 0) {
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
```

> Example taken from the [main.cpp file](src/main/main_motor_health_check.cpp)
