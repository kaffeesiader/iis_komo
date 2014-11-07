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
# this directory needs to contain MT.cfg (file or symlink)
cd `rospack find iis_komo`/tmp
# set namespace
export ROS_NAMESPACE=$N
# start iis_komo node
rosrun iis_komo iis_komo
