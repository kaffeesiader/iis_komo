#!/bin/bash

# check input argument
if [ -z "$1" ]
  then
    N="simulation"
  else 
    N=$1
fi

echo "Launching iis_komo for namespace '$N'"

# iis_komo only works when working directory is correctly set
cd `rospack find iis_komo`/config
# set namespace
export ROS_NAMESPACE=$N
# start iis_komo node
rosrun iis_komo iis_komo
