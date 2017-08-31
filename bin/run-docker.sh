#!/bin/bash

set -e

roscore &
source /opt/ros/kinetic/setup.bash
catkin_make
source devel/setup.bash
rostopic echo /roboy/system_notification/warning &
rosrun roboy_error_detection motor_tendent_inconsistence

/usr/sbin/sshd -D