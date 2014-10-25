#!/bin/bash

set -x

rosservice call /simulation/motion_control/move "eef_link: 'right_sdh_tip_link'
target: [0.1, 0.0, 0.1, 0, 3.1415, 0]"

read x

rosservice call /simulation/motion_control/move "eef_link: 'right_sdh_tip_link'
target: [0.1, 0.5, 0.1, 0, 3.1415, 0]"

read x

rosservice call /simulation/motion_control/move "eef_link: 'left_sdh_tip_link'
target: [0.1, 0.5, 0.1, 0, 3.1415, 0]"

