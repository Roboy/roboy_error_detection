#!/bin/bash

set -e

roscore &
source /opt/ros/kinetic/setup.bash
catkin_make
source devel/setup.bash

/usr/sbin/sshd -D