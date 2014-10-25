#include <Ors/ors.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <komo_interface.h>
#include <robot_interface.h>


#define JOINT_STATES_TOPIC "/simulation/right_arm/joint_control/get_state"
#define JOINT_MOVE_TOPIC "/simulation/right_arm/joint_control/move"


using namespace ros;
using namespace std;


void runTest(iis_komo::KomoInterface &ki, iis_komo::RobotInterface &ri) {

	ROS_INFO("Starting test run...");

	ROS_INFO("Planning for first target (position only)...");

	arr traj;
	if(ki.plan("right_eef", "target1", traj)) {
		ROS_INFO("Trajectory planning request successful - executing...");
//		ri.executePath(traj, "right");
		ROS_INFO("Trajectory execution complete.");
	}

	if(ki.plan("right_eef", "target2", traj)) {
//		ROS_INFO("Trajectory planning request successful - executing...");
//		ri.executePath(traj, "right");
//		ROS_INFO("Trajectory execution complete.");
	}

	if(ki.plan("left_eef", "target3", traj)) {
//		ROS_INFO("Trajectory planning request successful - executing...");
//		ri.executePath(traj, "left");
//		ROS_INFO("Trajectory execution complete.");
	}

	ROS_INFO("Test run complete");

}

int main(int argc,char** argv) {
	MT::initCmdLine(argc,argv);

	ros::init(argc, argv, "iis_komo_test");
	ros::AsyncSpinner spinner(4);
	spinner.start();

	NodeHandle nh("simulation");

	ors::KinematicWorld world("iis_robot.kvg");
	iis_komo::KomoInterface ki(&world);
	iis_komo::RobotInterface ri(nh);

	runTest(ki, ri);

	spinner.stop();
	return EXIT_SUCCESS;
}


