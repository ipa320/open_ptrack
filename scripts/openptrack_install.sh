#!/bin/bash

cd ~/catkin_ws

catkin_make --pkg opt_msgs
catkin_make --force-cmake

