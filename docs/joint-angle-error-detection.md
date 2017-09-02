# Joint Angle Error Detection

## Listen for Invalid Joint Angles

During the runtime, it is possible that you will receive invalid values from the joint angle sensors. To get notified about this 
unwanted behavior, you can subscribe to this error pattern. 

* **Method:**
```cpp
void listenForInvalidRelativeJointAngle(ObjectID jointId, angle minAngle, angle maxAngle, NotificationLevel logLevel = WARNING_LEVEL)
```
* **Arguments**

    | Name    | Type | Default | Required? | Description |
    | ----    | ---- | ------- | --------- | ----------- | 
    | jointId | `ObjectID` | - | `true` | `jointId` is the joint id, which identifies the joint you want to observe |
    | minAngle | `angle` | - | `true` | minimum relative angle |
    | maxAngle | `angle` | - | `true` | maximum relative angle |
    | logLevel| [DEBUG_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/DebugNotification.msg), [INFO_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/InfoNotification.msg), [WARNING_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/WarningNotification.msg), [ERROR_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/ErrorNotification.msg), [DANGER_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/DangerNotification.msg) | WARNING_LEVEL | `false` | Defines the system notification level, which also determines over which topic the health check message is published ([see all log levels](https://github.com/Roboy/roboy_system_notification/blob/master/include/roboy_system_notification/common_utilities.hpp#L12)) |

* **Output Topics:**

    General format depending on the log level: 
    
    - [DEBUG_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/DebugNotification.msg)
    - [INFO_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/InfoNotification.msg)
    - [WARNING_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/WarningNotification.msg)
    - [ERROR_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/ErrorNotification.msg)
    - [DANGER_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/DangerNotification.msg)
 
    **Modified parameters:** 
    
    | name     | condition | Description |
    | -------- | --------- | ------------------------- |
    | msg      | -         | [message string list](https://github.com/Roboy/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L15) |
    | code     | Angle is not in interval | [JOINT_INVALID_REL_ANGLE_NOTIFICATION](https://github.com/Roboy/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L7) |
    | object | - | same as input parameter `jointId` - ID of to be observed object: same as `jointId` parameter | 
    
* **Example**:

```cpp
#include "roboy_error_detection/roboyErrorDetection.hpp"

// [...]

int main(int argc, char* argv[])
{
    // [...]
    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
    RoboyErrorDetection handler(nh);

    int JOINT_ID = 0;
    int minJointAngle = 0, maxJointAngle = 100;
    handler.listenForInvalidRelativeJointAngle(JOINT_ID, minJointAngle, maxJointAngle);
    // [...]
    
    return 0;
}
```
> Example taken from the [main_joint_rel_angle_check.cpp file](https://github.com/Roboy/roboy_error_detection/src/main/main_joint_rel_angle_check.cpp)
    
## Listen for Too Close Joint Sensor Magnet

Joint sensors have internal magnets, which help them to discover the joint angles. But it can happen that they are too close 
or too far away, so that they can not calculate the angles. Therefore, this error pattern was introduced. 
[DEPRECATED]: In the new joint sensors, this problem won't happen any more.

* **Method:**
```cpp
void listenForJointMagnetStatus(ObjectID jointId, NotificationInterval durationOfValidity = 2000, NotificationLevel logLevel = WARNING_LEVEL)
```
* **Arguments**

    | Name    | Type | Default | Required? | Description |
    | ----    | ---- | ------- | --------- | ----------- | 
    | jointId | `ObjectID` | - | `true` | `jointId` is the joint id, which identifies the joint you want to observe |
    | durationOfValidity| `NotificationInterval` | 2000 | `false` | Minimum time in milliseconds from one health check to the next health check of this motor |
    | logLevel| [DEBUG_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/DebugNotification.msg), [INFO_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/InfoNotification.msg), [WARNING_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/WarningNotification.msg), [ERROR_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/ErrorNotification.msg), [DANGER_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/DangerNotification.msg) | WARNING_LEVEL | `false` | Defines the system notification level, which also determines over which topic the health check message is published ([see all log levels](https://github.com/Roboy/roboy_system_notification/blob/master/include/roboy_system_notification/common_utilities.hpp#L12)) |

* **Output Topics:**

    General format depending on the log level: 
    
    - [DEBUG_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/DebugNotification.msg)
    - [INFO_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/InfoNotification.msg)
    - [WARNING_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/WarningNotification.msg)
    - [ERROR_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/ErrorNotification.msg)
    - [DANGER_LEVEL](https://github.com/Roboy/roboy_communication/blob/feature/error-detection-msgs/roboy_communication_control/msg/DangerNotification.msg)
 
    **Modified parameters:** 
    
    | name     | condition | Description |
    | -------- | --------- | ------------------------- |
    | msg      | -         | [message string list](https://github.com/Roboy/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L15) |
    | code     | Magnet too far away | [JOINT_TOO_FAR_NOTIFICATION](https://github.com/Roboy/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L7) |
    | code     | Magnet too close | [JOINT_TOO_CLOSE_NOTIFICATION](https://github.com/Roboy/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L8) |
    | object | - | same as input parameter `jointId` - ID of to be observed object: same as `jointId` parameter | 
    
* **Example**:

```cpp
#include "roboy_error_detection/roboyErrorDetection.hpp"

// [...]

int main(int argc, char* argv[])
{
    // [...]
    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
    RoboyErrorDetection handler(nh);

    int JOINT_ID = 0;
    handler.listenForJointMagnetStatus(JOINT_ID);
    // [...]
    
    return 0;
}
```
> Example taken from the [main_joint_magnet_check.cpp file](https://github.com/Roboy/roboy_error_detection/src/main/main_joint_magnet_check.cpp)
    


