#!/bin/bash

# iis_komo only works when working directory is correctly set
cd `rospack find iis_komo`/config
# set namespace
export ROS_NAMESPACE=simulation
# start iis_komo node
rosrun iis_komo iis_komo