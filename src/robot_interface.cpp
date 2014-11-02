#include <Core/array.h>
#include <robot_interface.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

namespace iis_komo {


RobotInterface::RobotInterface(NodeHandle &nh) {

	_pub_left_arm_move = nh.advertise<std_msgs::Float64MultiArray>("left_arm/joint_control/move", 1, true);
	_pub_right_arm_move = nh.advertise<std_msgs::Float64MultiArray>("right_arm/joint_control/move", 1, true);

	_sub_left_arm_state = nh.subscribe("left_arm/joint_control/get_state", 1, &RobotInterface::CBLeftArmState, this);
	_sub_right_arm_state = nh.subscribe("right_arm/joint_control/get_state", 1, &RobotInterface::CBRightArmState, this);
	_sub_left_sdh_state = nh.subscribe("left_sdh/joint_control/get_state", 1, &RobotInterface::CBLeftSdhState, this);
	_sub_right_sdh_state = nh.subscribe("right_sdh/joint_control/get_state", 1, &RobotInterface::CBRightSdhState, this);
}

IISRobotState RobotInterface::getState()
{
	return _current_state;
}

void RobotInterface::execute(IISRobot::PlanninGroup group, const IISRobot::Path &path)
{
	Publisher *pub = NULL;
	switch (group) {
	case IISRobot::LeftArm:
		pub = &_pub_left_arm_move;
		break;
	case IISRobot::RightArm:
		pub = &_pub_right_arm_move;
		break;
	default:
		ROS_ERROR("Hand trajectory execution is not implemented yet!");
		return;
	}

	std_msgs::Float64MultiArray msg;
	msg.layout.dim.resize(1);
	msg.layout.dim[0].size = 7;
	msg.data.resize(7);

	ros::Rate r(100);

	// iterate over trajectory points...
	for (int i = 1; i < path.d0; ++i) {

		arr current_wp = path[i-1];
		arr next_wp = path[i];
		arr diff = next_wp - current_wp;

		// how many interpolation steps ?
		double steps = 10;

		// ... calculate interpolation step based on loop rate ...
		arr step = diff / steps;

		// ... and move subsequently towards next position.
		for (int j = 0; j < steps; ++j) {

			for(int k = 0; k < 7; ++k) {
				msg.data[k] = current_wp(k);
			}

			pub->publish(msg);

			current_wp += step;

			r.sleep();
		}
	}
}

void RobotInterface::CBLeftArmState(const sensor_msgs::JointStatePtr &msg)
{
	for (int i = 0; i < 7; ++i) {
		_current_state.left_arm[i] = msg->position[i];
	}
//	ROS_INFO("Left joint state received!");
}

void RobotInterface::CBRightArmState(const sensor_msgs::JointStatePtr &msg)
{
	for (int i = 0; i < 7; ++i) {
		_current_state.right_arm[i] = msg->position[i];
	}
//	ROS_INFO("Right joint state received!");
}

void RobotInterface::CBLeftSdhState(const sensor_msgs::JointStatePtr &msg)
{
	for (int i = 0; i < 7; ++i) {
		_current_state.left_sdh[i] = msg->position[i];
	}
}

void RobotInterface::CBRightSdhState(const sensor_msgs::JointStatePtr &msg)
{
	for (int i = 0; i < 7; ++i) {
		_current_state.right_sdh[i] = msg->position[i];
	}
}

}
