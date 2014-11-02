#ifndef KOMO_WRAPPER_H
#define KOMO_WRAPPER_H

#include <vector>

#include <Ors/ors.h>
#include <Ors/ors_swift.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>

#include <iis_robot.h>

using namespace std;


namespace iis_komo {

class KomoWrapper {

public:

	KomoWrapper(const string &config_name);
	~KomoWrapper();

	void setState(const IISRobotState &state);

    // with start state
	bool plan(const string &eef, IISRobot::PlanninGroup group, double x, double y, double z, IISRobot::Path &path);
	bool plan(const string &eef, IISRobot::PlanninGroup group, double x, double y, double z, double roll, double pitch, double yaw, IISRobot::Path &path);
	bool plan(const string &eef, IISRobot::PlanninGroup group, double x, double y, double z, double qx, double qy, double qz, double qw, IISRobot::Path &path);

//	bool plan(const vector<double> &start_state, const string &goal_name, arr &traj);

	bool plan(const string eef, const string goal_name, arr &traj);

	void display(bool block = false, const char *msg = "Ready...");
	void addShape();

private:

    ors::KinematicWorld *_world;
	vector<ors::Body *> _collision_objects;

	void setState(IISRobot::PlanninGroup group, const double values[]);
	void arrToPath(const arr &traj, IISRobot::Path &path);
	// validates the planning outcome
	// checks for correctness and if all constraints are met
	bool validateResult(const arr &traj, ors::Shape &eef, ors::Shape &target, bool position_only = false);

	/// Return a trajectory that moves the endeffector to a desired target position (taken from komo.h)
	bool planTo(ors::KinematicWorld& world,//in initial state
			   ors::Shape &endeff,         //endeffector to be moved
			   ors::Shape &target,         //target shape
			   uint agent,				   //the group we want to plan for
			   arr &traj,                  //the resulting trajectory
			   byte whichAxesToAlign = 0,  //bit coded options to align axes
			   uint iterate = 1);          //usually the optimization methods may be called just once; multiple calls -> safety

};

}

#endif // KOMO_WRAPPER_H
