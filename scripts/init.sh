#!/bin/bash

rostopic pub /simulation/right_arm/settings/switch_mode -1 std_msgs/Int32 10 &
rostopic pub /simulation/left_arm/settings/switch_mode -1 std_msgs/Int32 10 &

rostopic pub /simulation/right_arm/joint_control/set_velocity_limit -1 std_msgs/Float32 1.0 & 
rostopic pub /simulation/left_arm/joint_control/set_velocity_limit -1 std_msgs/Float32 1.0 & 

# collidable box
rostopic pub /simulation/scene/AddPrimitiveShape planning_scene_plugin/AddPrimitiveShape -1 "object_id: 'obstacle1' 
pose:
  position: {x: 0.1, y: 0.25, z: 0.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
disable_collision_checking: false
mass: 1.0
type: 1
dimensions: [0.2,0.2,0.2]" &

sleep 1

rostopic pub /simulation/scene/AddPrimitiveShape planning_scene_plugin/AddPrimitiveShape -1 "object_id: 'obstacle2' 
pose:
  position: {x: 0.1, y: 0.8, z: 0.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
disable_collision_checking: false
mass: 1.0
type: 1
dimensions: [0.2,0.2,0.2]" &
