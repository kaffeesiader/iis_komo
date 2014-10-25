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

void RobotInterface::execute(const vector<IISRobotState> &path)
{
	std_msgs::Float64MultiArray msg_left;
	msg_left.layout.dim.resize(1);
	msg_left.layout.dim[0].size = 7;
	msg_left.data.resize(7);

	std_msgs::Float64MultiArray msg_right;
	msg_right.layout.dim.resize(1);
	msg_right.layout.dim[0].size = 7;
	msg_right.data.resize(7);

	ros::Rate r(100);

	// iterate over trajectory points...
	for (int i = 1; i < path.size(); ++i) {

		const IISRobotState &current_wp = path[i-1];
		const IISRobotState &next_wp = path[i];

		arr c_state_left(&current_wp.left_arm[0], 7);
		arr n_state_left(&next_wp.left_arm[0], 7);
		arr diff_left = n_state_left - c_state_left;

		arr c_state_right(&current_wp.right_arm[0], 7);
		arr n_state_right(&next_wp.right_arm[0], 7);
		arr diff_right = n_state_right - c_state_right;

		// ... calculate interpolation step based on loop rate ...
		arr step_left = diff_left / 10.0;
		arr step_right = diff_right / 10.0;

		// ... and move subsequently towards next position.
		for (int j = 0; j < 10; ++j) {

			for(int k = 0; k < 7; ++k) {
				msg_left.data[k] = c_state_left(k);
				msg_right.data[k] = c_state_right(k);
			}

			_pub_left_arm_move.publish(msg_left);
			_pub_right_arm_move.publish(msg_right);

			c_state_left += step_left;
			c_state_right += step_right;

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
