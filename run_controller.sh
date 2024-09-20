#!/bin/bash

source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 run webots_ros2_homework1_python webots_ros2_homework1_python
