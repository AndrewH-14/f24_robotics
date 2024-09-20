#!/bin/bash

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch webots_ros2_homework1_python f23_robotics_1_launch.py
ps -deaf | grep ros