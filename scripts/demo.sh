#!/bin/bash

set -x

rosservice call /simulation/motion_control/move "eef_link: 'right_sdh_tip_link'
target: [0.3, 0.0, 0.2, 0, 3.1415, 0]"

read x

rosservice call /simulation/motion_control/move "eef_link: 'right_sdh_tip_link'
target: [0.3, 0.5, 0.2, 0, 3.1415, 0]"

read x

rosservice call /simulation/motion_control/move "eef_link: 'right_sdh_tip_link'
target: [0.55, 0.25, 0.2, 0, 3.1415, 0]"

