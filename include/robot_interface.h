#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <iis_robot.h>

using namespace std;
using namespace ros;

namespace iis_komo {


class RobotInterface {

public:

	RobotInterface(NodeHandle &nh);
	~RobotInterface() {}

	IISRobotState getState();
	void execute(IISRobot::PlanninGroup group, const IISRobot::Path &path);

private:

	Publisher _pub_left_arm_move;
	Publisher _pub_right_arm_move;

	Subscriber _sub_left_arm_state;
	Subscriber _sub_right_arm_state;
	Subscriber _sub_left_sdh_state;
	Subscriber _sub_right_sdh_state;

	IISRobotState _current_state;

	void CBLeftArmState(const sensor_msgs::JointStatePtr &msg);
	void CBRightArmState(const sensor_msgs::JointStatePtr &msg);
	void CBLeftSdhState(const sensor_msgs::JointStatePtr &msg);
	void CBRightSdhState(const sensor_msgs::JointStatePtr &msg);

};

}


#endif // ROBOT_INTERFACE_H
