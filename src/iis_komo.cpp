#include <Ors/ors.h>
#include <ros/ros.h>

#include <iis_komo/ExecuteTrajectory.h>
#include <iis_komo/PlanTrajectory.h>
#include <robot_interface.h>
#include <komo_wrapper.h>
#include <time_parameterization.h>
#include <utils.h>

#define MODEL_FILE_NAME "../data/iis_robot.kvg"

using namespace ros;
using namespace std;

namespace iis_komo {

class KomoInterface {

public:
	KomoInterface(NodeHandle &nh, KomoWrapper *komo, RobotInterface *robot)
	{
		_komo = komo;
		_robot = robot;
//		_move_srv = nh.advertiseService("motion_control/move", &KomoInterface::callbackMove, this);
		_plan_srv = nh.advertiseService("motion_control/plan_trajectory", &KomoInterface::callbackPlan, this);
		_execute_srv = nh.advertiseService("motion_control/execute_trajectory", &KomoInterface::callbackExecute, this);

		// wait for initial state (TODO: improve this!)
		sleep(1);

		_komo->setState(_robot->getState());
		_komo->display();
	}

private:

	KomoWrapper *_komo;
	RobotInterface *_robot;
	TimeParameterization _time_param;

	ServiceServer _move_srv;
	ServiceServer _execute_srv;
	ServiceServer _plan_srv;

	bool callbackPlan(iis_komo::PlanTrajectory::Request &request, iis_komo::PlanTrajectory::Response &response) {
		string eef = request.eef_link;
		size_t sz = request.target.size();

		ROS_INFO("Received planning request for link '%s' on planning group '%s'", eef.c_str(), request.planning_group.c_str());
		IISRobot::PlanninGroup group;

		if(request.planning_group == "left_arm") {
			group = IISRobot::LeftArm;
		} else if(request.planning_group == "right_arm") {
			group = IISRobot::RightArm;
		} else {
			ROS_ERROR("Unknown planning group '%s'.", request.planning_group.c_str());
			response.result = false;
			return true;
		}

		bool success;
		IISRobot::Path path;
		IISRobotState state = _robot->getState();

		// define some default values to use if no tolerances provided...
		ors::Vector pos_tol = { 0.005, 0.005, 0.005 };
		ors::Vector ang_tol = { 0.1, 0.1, 0.1 };

		// set tolerance values if provided
		if((request.position_tolerance.x > 0.0) ||
			(request.position_tolerance.y > 0.0) ||
			(request.position_tolerance.z > 0.0))
		{
			pos_tol.x = request.position_tolerance.x;
			pos_tol.y = request.position_tolerance.y;
			pos_tol.z = request.position_tolerance.z;
		}

		if((request.angular_tolerance.x > 0.0) ||
			(request.angular_tolerance.y > 0.0) ||
			(request.angular_tolerance.z > 0.0))
		{
			ang_tol.x = request.angular_tolerance.x;
			ang_tol.y = request.angular_tolerance.y;
			ang_tol.z = request.angular_tolerance.z;
		}

		_komo->setPositionTolerance(pos_tol);
		_komo->setAngularTolerance(ang_tol);

		switch (sz) {
		case 3:
			// position only
			ROS_INFO("Specifying position only goal constraint.");
			success = _komo->plan(
						eef,
						request.target[0],
						request.target[1],
						request.target[2],
						state,
						path);
			break;
		case 6:
			// position and orientation based on roll, pitch, yaw
			ROS_INFO("Specifying position and orientation goal constraint (rpy).");
			success = _komo->plan(
						eef,
						request.target[0],
						request.target[1],
						request.target[2],
						request.target[3],
						request.target[4],
						request.target[5], state, path);
			break;
		case 7:
			// position and orientation based on quaternion
			ROS_INFO("Specifying position and orientation goal constraint (quat).");
			success = _komo->plan(
						eef,
						request.target[0],
						request.target[1],
						request.target[2],
						request.target[3],
						request.target[4],
						request.target[5],
						request.target[6],
						state,
						path);
			break;
		default:
			ROS_ERROR("Unable to plan - invalid target specification!");
			return true;
		}

		if(success) {
			ROS_INFO("Planning successful!");
			komo_path_to_joint_traj(group, path, response.trajectory);
			response.result = true;

			ROS_INFO("Planning request completed!");
		} else {
			ROS_WARN("Planning failed!");
			response.result = false;
		}

		return true;
//		_time_param.computeTimeStamps(response.trajectory);


//		if(!request.plan_only) {
//			ROS_INFO("Executing trajectory...");
//			success = _robot->execute(group, response.trajectory);
//			if(success) {
//				ROS_INFO("Trajectory execution complete.");
//			} else {
//				ROS_ERROR("Trajectory execution failed.");
//			}
//		}

	}

	bool callbackExecute(iis_komo::ExecuteTrajectory::Request &request, iis_komo::ExecuteTrajectory::Response &response) {

		ROS_INFO("Received trajectory execution request for planning group '%s'", request.planning_group.c_str());
		IISRobot::PlanninGroup group;

		if(request.planning_group == "left_arm") {
			group = IISRobot::LeftArm;
		} else if(request.planning_group == "right_arm") {
			group = IISRobot::RightArm;
		} else {
			ROS_ERROR("Unknown planning group '%s'.", request.planning_group.c_str());
			response.result = false;
			return true;
		}

		double vel_factor = 0.5;

		if(request.velocity_factor > 0) {
			vel_factor = request.velocity_factor;
		}
		ROS_INFO("Computing time parameterization (velocity factor: %.2f)", vel_factor);
		_time_param.clearTimeParams(request.trajectory);
		_time_param.set_velocity_factor(vel_factor);
		_time_param.computeTimeStamps(request.trajectory);

		ROS_INFO("Executing trajectory...");

		if(_robot->execute(group, request.trajectory)) {
			ROS_INFO("Trajectory execution complete.");
			response.result = true;
		} else {
			ROS_ERROR("Trajectory execution failed.");
			response.result = false;
		}

		return true;
	}

};

}

int main(int argc, char *argv[])
{
	MT::initCmdLine(argc, argv);
	ros::init(argc, argv, "iis_komo");
	AsyncSpinner spinner(2);
	spinner.start();

	NodeHandle nh;

	ROS_INFO("Starting IIS_KOMO node in namespace '%s'...", nh.getNamespace().c_str());

	iis_komo::KomoWrapper wrapper(MODEL_FILE_NAME);
	iis_komo::RobotInterface robot(nh);
	iis_komo::KomoInterface ki(nh, &wrapper, &robot);

	ros::waitForShutdown();

	spinner.stop();
	ROS_INFO("IIS_KOMO node shutdown completed.");

	return EXIT_SUCCESS;
}

