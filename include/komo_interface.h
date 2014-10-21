#ifndef KOMO_INTERFACE_H
#define KOMO_INTERFACE_H

#include <Ors/ors.h>
#include <Ors/ors_swift.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/taskMap_transition.h>

using namespace std;


namespace iis_komo {

class KomoInterface {

public:

    KomoInterface(const string &config_name);
    ~KomoInterface();

    // with start state
    bool plan(const vector<double> &start_state, double x, double y, double z, arr &traj);
    bool plan(const vector<double> &start_state, double x, double y, double z, double roll, double pitch, double yaw, arr &traj);
	bool plan(const vector<double> &start_state, const string &goal_name, arr &traj);

	bool plan(const string eef, const string goal_name, arr &traj);

    void display();

private:

    ors::KinematicWorld *_world;

	/// Return a trajectory that moves the endeffector to a desired target position (taken from komo.h)
	bool planTo(ors::KinematicWorld& world,//in initial state
			   ors::Shape& endeff,         //endeffector to be moved
			   ors::Shape &target,         //target shape
			   arr &traj,                  //the resulting trajectory
			   byte whichAxesToAlign = 0,  //bit coded options to align axes
			   uint iterate = 1);          //usually the optimization methods may be called just once; multiple calls -> safety

};

}

#endif // KOMO_INTERFACE_H
