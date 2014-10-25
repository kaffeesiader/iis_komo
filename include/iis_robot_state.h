#ifndef IIS_ROBOT_STATE_H
#define IIS_ROBOT_STATE_H

struct IISRobotState {

	double left_arm[7];
	double right_arm[7];
	double left_sdh[7];
	double right_sdh[7];

};

#endif // IIS_ROBOT_STATE_H
