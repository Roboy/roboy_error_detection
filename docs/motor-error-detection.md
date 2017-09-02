# Motor Error Detection

## Listen for Motor Health

The program checks in a defined interval, whether a defined motor is healthy or not. The result is then published over ROS (topics are specified [here](https://github.com/Roboy/roboy_system_notification)).  

* **Method:**
```cpp
void listenForMotorHealth(ObjectID motorId, NotificationInterval durationOfValidity = 5000, NotificationLevel logLevel = WARNING_LEVEL)
```
* **Arguments**

    | Name    | Type | Default | Required? | Description |
    | ----    | ---- | ------- | --------- | ----------- | 
    | motorId | `ObjectID` | - | `true` | `motorId` is the motor id, which identifies the motor you want to observe |
    | durationOfValidity| `NotificationInterval` | 5000 | `false` | Minimum time in milliseconds from one health check to the next health check of this motor |
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
    | msg      | -         | [message string list](https://github.com/Roboy/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L7) |
    | code     | Motor is dead | [MOTOR_DEAD_NOTIFICATION](https://github.com/Roboy/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L3) |
    |          | Motor is alive | [MOTOR_ALIVE_NOTIFICATION](https://github.com/Roboy/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L4) |
    | object | - | same as input parameter `motorId` - ID of to be observed object: same as `motorId` parameter | 
    
* **Example**:

```cpp
#include "roboy_error_detection/roboyErrorDetection.hpp"

// [...]

int main(int argc, char* argv[])
{
    // [...]
    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
    RoboyErrorDetection handler(nh);

    // setup for listening to motor1 health
    ObjectID MOTOR_ID = 123;
    handler.listenForMotorHealth(MOTOR_ID);

    return 0;
}
```
> Example taken from the [main.cpp file](https://github.com/Roboy/roboy_error_detection/src/main/main_motor_health_check.cpp)
    
## Listen if Motor is Dead

The program checks in a defined interval, whether a defined motor is dead (does not respond) or not. **Only if** the motor is dead, the result is then published over ROS (topics are specified [here](https://github.com/Roboy/roboy_system_notification)).  

* **Method:**
```cpp
void listenIfMotorIsDead(ObjectID motorId, NotificationInterval durationOfValidity = 5000, NotificationLevel logLevel = WARNING_LEVEL)
```
* **Arguments**

    | Name    | Type | Default | Required? | Description |
    | ----    | ---- | ------- | --------- | ----------- | 
    | motorId | `ObjectID` | - | `true` | `motorId` is the motor id, which identifies the motor you want to observe |
    | durationOfValidity| `NotificationInterval` | 5000 | `false` | Minimum time in milliseconds from one health check to the next health check of this motor |
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
    | msg      | -         | [message string list](https://github.com/Roboy/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L7) |
    | code     | - | [MOTOR_DEAD_NOTIFICATION](https://github.com/Roboy/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L3) |
    | object | - | same as input parameter `motorId` - ID of to be observed object: same as `motorId` parameter | 
    
* **Example**:

```cpp
#include "roboy_error_detection/roboyErrorDetection.hpp"

// [...]

int main(int argc, char* argv[])
{
    // [...]
    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
    RoboyErrorDetection handler(nh);

    // setup for listening to motor1 health
    ObjectID MOTOR_ID = 123;
    handler.listenIfMotorIsDead(MOTOR_ID);

    return 0;
}
```
> Example taken from the [main_motor_dead_check.cpp file](https://github.com/Roboy/roboy_error_detection/src/main/main_motor_dead_check.cpp)
  
 
## Listen if Motor is Alive

The program checks in a defined interval, whether a defined motor is alive (does respond) or not. **Only if** the motor is alive, the result is then published over ROS (topics are specified [here](https://github.com/Roboy/roboy_system_notification)).  

* **Method:**
```cpp
void listenForMotorHealth(ObjectID motorId, NotificationInterval durationOfValidity = 5000, NotificationLevel logLevel = WARNING_LEVEL)
```
* **Arguments**

    | Name    | Type | Default | Required? | Description |
    | ----    | ---- | ------- | --------- | ----------- | 
    | motorId | `ObjectID` | - | `true` | `motorId` is the motor id, which identifies the motor you want to observe |
    | durationOfValidity| `NotificationInterval` | 5000 | `false` | Minimum time in milliseconds from one health check to the next health check of this motor |
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
    | msg      | -         | [message string list](https://github.com/Roboy/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L7) |
    | code     | - | [MOTOR_ALIVE_NOTIFICATION](https://github.com/Roboy/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L4) |
    | object | - | same as input parameter `motorId` - ID of to be observed object: same as `motorId` parameter | 
    
* **Example**:

```cpp
#include "roboy_error_detection/roboyErrorDetection.hpp"

// [...]

int main(int argc, char* argv[])
{
    // [...]
    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
    RoboyErrorDetection handler(nh);

    // setup for listening to motor1 health
    ObjectID MOTOR_ID = 123;
    handler.listenForMotorHealth(MOTOR_ID);

    return 0;
}
```
> Example taken from the [main_motor_alive_check.cpp file](https://github.com/Roboy/roboy_error_detection/src/main/main_motor_alive_check.cpp)
    
## Listen for Motor Tendon Inconsistency

It is possible that a motor is moving but the tendon does not. If this happen, then something went wrong (e.g. broken tendon). Therefore, this method was introduced and now, you have the possibility to get notified if this happen.    
Thereby, the system checks whether the motor velocity is not zero (=> motor is running) and if the tacho is in the interval ]minTacho,maxTacho[ (=> tendon is moving).
                                       
* **Method:**
```cpp
void listenForMotorTendonInconsistence(ObjectID motorId, ObjectID jointId, tacho minTacho, tacho maxTacho, NotificationInterval durationOfValidity = 5000, NotificationLevel logLevel = WARNING_LEVEL)
```
* **Arguments**

   | Name    | Type | Default | Required? | Description |
   | ----    | ---- | ------- | --------- | ----------- | 
   | motorId | `ObjectID` | - | `true` | `motorId` is the motor id, which identifies the motor you want to observe |
   | jointId | `ObjectID` | - | `true` | `jointId` is the joint id, which measures the tendon activity of the to be observed motor |
   | minTacho| `tacho` | - | `true` | minimum tacho valure of a running tendon |
   | maxTacho| `tacho` | - | `true` |  maximum tacho value of a running tendon |
   | durationOfValidity| `NotificationInterval` | 5000 | `false` | Minimum time in milliseconds from one health check to the motor inconsistency check of this joint |
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
   | msg      | -         | [message string list](https://github.com/Roboy/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L7) |
   | code     | - | [MOTOR_IS_RUNNING_BUT_TENDON_NOT_NOTIFICATION](https://github.com/Roboy/roboy_error_detection/blob/master/include/roboy_error_detection/common_utilities.hpp#L6) |
   | object | - | same as input parameter `motorId` - ID of to be observed object: same as `motorId` parameter | 
   
* **Example**:

```cpp
#include "roboy_error_detection/roboyErrorDetection.hpp"

// [...]

int main(int argc, char* argv[])
{
   // [...]
   ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle);
   RoboyErrorDetection handler(nh);

   ObjectID motorId = 123;
   ObjectID jointId = 321;
   tacho minTacho = 0 + 1;
   tacho maxTacho = 1023 - 1;
   handler.listenForMotorTendonInconsistence(motorId, jointId, minTacho, maxTacho);

   return 0;
}
```
> Example taken from the [main_motor_tendon_inconsistency.cpp file](https://github.com/Roboy/roboy_error_detection/src/main/main_motor_tendon_inconsistency.cpp)
