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

	ROS_INFO("Connecting to arm controllers...");

	string topic = "left_arm/follow_joint_trajectory";
	_left_arm_client.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(topic, true));
	_left_arm_client->waitForServer();

	 topic = "right_arm/follow_joint_trajectory";
	_right_arm_client.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(topic, true));
	_right_arm_client->waitForServer();

	ROS_INFO("Connection established...");
}

IISRobotState RobotInterface::getState()
{
	return _current_state;
}

bool RobotInterface::execute(IISRobot::PlanninGroup group, trajectory_msgs::JointTrajectory &trajectory)
{
	ControllerClientPtr client;

	switch (group) {
	case IISRobot::LeftArm:
		client = _left_arm_client;
		break;
	case IISRobot::RightArm:
		client = _right_arm_client;
		break;
	default:
		ROS_ERROR("Can only execute trajectories for left or right robot arm!");
		return false;
	}

	if (!client->isServerConnected()) {
		ROS_ERROR("Controller action server not available");
		return false;
	}

	control_msgs::FollowJointTrajectoryActionGoal goal;

	trajectory.header.stamp = ros::Time::now();

	goal.goal.trajectory = trajectory;
	goal.header.stamp = ros::Time::now();

	client->sendGoal(goal.goal);

	if (!client->waitForResult()) {
		ROS_INFO_STREAM("Controller action returned early");
	}

	control_msgs::FollowJointTrajectoryResultConstPtr res = client->getResult();

	if (client->getState()	== actionlib::SimpleClientGoalState::SUCCEEDED) {

		if(res->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL) {
			ROS_INFO("Trajectory execution succeeded!");
			return true;
		} else {
			ROS_ERROR("Trajectory execution failed: ", res->error_code);
			return false;
		}

	} else {
		ROS_WARN_STREAM("Execution failed: " << client->getState().toString() << ": " << client->getState().getText());
		return false;
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
