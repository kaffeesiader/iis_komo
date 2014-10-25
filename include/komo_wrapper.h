#ifndef KOMO_WRAPPER_H
#define KOMO_WRAPPER_H

#include <Ors/ors.h>
#include <Ors/ors_swift.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>

#include <iis_robot_state.h>

using namespace std;


namespace iis_komo {

class KomoWrapper {

public:

	KomoWrapper(const string &config_name);
	~KomoWrapper();

	void setState(const IISRobotState &state);

    // with start state
	bool plan(const string &eef, double x, double y, double z, const IISRobotState &start_state, vector<IISRobotState> &path);
	bool plan(const string &eef, double x, double y, double z, double roll, double pitch, double yaw, const IISRobotState &start_state, vector<IISRobotState> &path);
	bool plan(const string &eef, double x, double y, double z, double qx, double qy, double qz, double qw, const IISRobotState &start_state, vector<IISRobotState> &path);

//	bool plan(const vector<double> &start_state, const string &goal_name, arr &traj);

	bool plan(const string eef, const string goal_name, arr &traj);

    void display();

private:

    ors::KinematicWorld *_world;


	void setGripperState(const string &arm, const double *state);
	void getGripperState(const string &arm, const double *state);
	int getWorldJointIndex(const string &arm, int idx);
	void arrToPath(const arr &traj, vector<IISRobotState> &path);
	// validates the planning outcome
	// checks for correctness and if all constraints are met
	bool validateResult(const arr &traj, ors::Shape &eef, ors::Shape &target, bool position_only = false);

	/// Return a trajectory that moves the endeffector to a desired target position (taken from komo.h)
	bool planTo(ors::KinematicWorld& world,//in initial state
			   ors::Shape &endeff,         //endeffector to be moved
			   ors::Shape &target,         //target shape
			   arr &traj,                  //the resulting trajectory
			   byte whichAxesToAlign = 0,  //bit coded options to align axes
			   uint iterate = 1);          //usually the optimization methods may be called just once; multiple calls -> safety

};

}

#endif // KOMO_WRAPPER_H
