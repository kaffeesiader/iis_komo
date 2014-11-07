#include <ros/ros.h>
#include <iis_komo/Move.h>
#include <iis_schunk_hardware/GripCmd.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "test_KOMO_pick");
	ros::NodeHandle nh;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Publisher pub_grasp = nh.advertise<iis_schunk_hardware::GripCmd>("right_sdh/joint_control/grip_hand", 1, true);

	ROS_INFO("Creating service client for KOMO...");

	// create a service client for the KOMO service...
	ros::ServiceClient client = nh.serviceClient<iis_komo::Move>("motion_control/move");
	// ... and wait for the service to come up if necessary
	while(!client.exists()) {
		ROS_WARN("Waiting for KOMO service to come up...");
		client.waitForExistence(ros::Duration(30));
	}

	ROS_INFO("Service client created!");

	ROS_INFO("Planning and moving to pregrasp position");

	// define the service request
	iis_komo::MoveRequest request;
	request.eef_link = "right_sdh_tip_link";
	request.plan_only = false;
	// specify target position
	request.target.push_back(0.3); // pos x
	request.target.push_back(0.3); // pos y
	request.target.push_back(0.4); // pos z
	request.target.push_back(0.0); // orient r
	request.target.push_back(3.14);// orient p
	request.target.push_back(0.0); // orient y

	// alternatively use quaternion ...
//	request.target.push_back(0.3); // pos x
//	request.target.push_back(0.3); // pos y
//	request.target.push_back(0.4); // pos z
//	request.target.push_back(0.0); // quat x
//	request.target.push_back(0.0); // quat y
//	request.target.push_back(0.0); // quat z
//	request.target.push_back(1.0); // quat w

	// ... or do position only planning
//	request.target.push_back(0.3); // pos x
//	request.target.push_back(0.3); // pos y
//	request.target.push_back(0.4); // pos z

	// create response object
	iis_komo::MoveResponse response;

	if(!client.call(request, response)) {
		ROS_ERROR("Error executing KOMO service request!");
		return EXIT_FAILURE;
	}

	// check if planning and execution was succesful
	if(!response.result) {
		ROS_ERROR("Trajectory planning/execution failed!");
		return EXIT_FAILURE;
	}

	ROS_INFO("Pregrasp position reached!");
	ROS_INFO("Preshaping hand");

	iis_schunk_hardware::GripCmd grasp;
	grasp.gripType = 1; // parallel
	grasp.closeRatio = 0.0;
	pub_grasp.publish(grasp);

	// allow the gripper to open
	sleep(2);
	ROS_INFO("Planning and moving to grasp position");

	request.target.clear();
	request.target.push_back(0.3); // pos x
	request.target.push_back(0.3); // pos y
	request.target.push_back(0.15); // pos z
	request.target.push_back(0.0); // orient r
	request.target.push_back(3.14);// orient p
	request.target.push_back(0.0); // orient y

	if(!client.call(request, response)) {
		ROS_ERROR("Error executing KOMO service request!");
		return EXIT_FAILURE;
	}

	// check if planning and execution was succesful
	if(!response.result) {
		ROS_ERROR("Trajectory planning/execution failed!");
		return EXIT_FAILURE;
	}

	ROS_INFO("Grasp position reached!");
	ROS_INFO("Grasping object");

	grasp.closeRatio = 0.9;
	pub_grasp.publish(grasp);

	sleep(2);

	ROS_INFO("Planning and moving to postgrasp position");

	request.target.clear();
	request.target.push_back(0.0); // pos x
	request.target.push_back(0.0); // pos y
	request.target.push_back(0.4); // pos z
	request.target.push_back(0.0); // orient r
	request.target.push_back(3.14);// orient p
	request.target.push_back(0.0); // orient y

	if(!client.call(request, response)) {
		ROS_ERROR("Error executing KOMO service request!");
		return EXIT_FAILURE;
	}

	// check if planning and execution was succesful
	if(!response.result) {
		ROS_ERROR("Trajectory planning/execution failed!");
		return EXIT_FAILURE;
	}

	ROS_INFO("Postgrasp position reached!");
	ROS_INFO("Grasp completed!");

	spinner.stop();

	return EXIT_SUCCESS;
}

