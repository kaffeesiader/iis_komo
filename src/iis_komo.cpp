#include <Ors/ors.h>
#include <ros/ros.h>

#include <iis_komo/Move.h>
#include <robot_interface.h>
#include <komo_wrapper.h>

using namespace ros;
using namespace std;

namespace iis_komo {

class KomoInterface {

public:
	KomoInterface(NodeHandle &nh, KomoWrapper *komo, RobotInterface *robot)
	{
		_komo = komo;
		_robot = robot;
		_move_srv = nh.advertiseService("motion_control/move", &KomoInterface::callbackMove, this);
		// wait for initial state
		sleep(1);

		_komo->setState(_robot->getState());
		_komo->display();
	}

private:

	KomoWrapper *_komo;
	RobotInterface *_robot;

	ServiceServer _move_srv;

	bool callbackMove(iis_komo::Move::Request &request, iis_komo::Move::Response &response) {
		string eef = request.eef_link;
		size_t sz = request.target.size();

		ROS_INFO("Received planning request for link '%s'", eef.c_str());

		vector<IISRobotState> path;
		IISRobotState state = _robot->getState();

		switch (sz) {
		case 3:
			// position only
			ROS_INFO("Specifying position only goal constraint.");
			_komo->plan(
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
			_komo->plan(
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
			_komo->plan(
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
			return false;
		}

		if(path.size() == 0) {
			ROS_WARN("Planning failed!");
			response.result = false;

			return false;
		}

		ROS_INFO("Planning successful!");

		if(!request.plan_only) {
			ROS_INFO("Executing trajectory...");
			_robot->execute(path);
			ROS_INFO("Trajectory execution complete.");
		}

		response.result = true;
		ROS_INFO("Planning request completed!");

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

	iis_komo::KomoWrapper wrapper("iis_robot.kvg");
	iis_komo::RobotInterface robot(nh);
	iis_komo::KomoInterface ki(nh, &wrapper, &robot);

	ros::waitForShutdown();

	spinner.stop();
	ROS_INFO("IIS_KOMO node shutdown completed.");

	return EXIT_SUCCESS;
}

