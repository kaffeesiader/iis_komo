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

	// set the robot configuration to given state
	void setState(const IISRobotState &state);

	// tolerance values used for trajectory validation
	void setPositionTolerance(const ors::Vector tolerance) { _pos_tolerance = tolerance; }
	void setAngularTolerance(const ors::Vector tolerance) { _ang_tolerance = tolerance; }

	void setPositionPrecision(double value) { _positionPrecision = value; }
	void setZeroVelocityPrecision(double value) { _zeroVelocityPrecision = value; }
	void setJointLimitPrecision(double value) { _jointLimitPrecision = value; }
	void setJointLimitMargin(double value) { _jointLimitMargin = value; }
	void setCollisionPrecision(double value) { _collisionPrecision = value; }
	void setCollisionMargin(double value) { _collisionMargin = value; }
	void setAlignmentPrecision(double value) { _alignmentPrecision = value; }
	void setIterations(double value) { _iterations = value; }

	bool plan(IISRobot::PlanninGroup group, const string &eef,
			  const ors::Vector &goal_pos, const ors::Quaternion &goal_orient,
			  bool position_only, IISRobot::Path &path,
			  ors::Vector &pos_error, ors::Vector &ang_error, string &error_msg);

	void display(bool block = false, const char *msg = "Ready...");

private:

    ors::KinematicWorld *_world;

	ors::Vector _pos_tolerance;
	ors::Vector _ang_tolerance;

	double _positionPrecision;
	double _zeroVelocityPrecision;
	double _jointLimitPrecision;
	double _jointLimitMargin;
	double _collisionPrecision;
	double _collisionMargin;
	double _alignmentPrecision;
	double _iterations;

	// validates the planning outcome
	// checks for correctness and if all constraints are met
	bool validateJointLimits(const arr &traj, string &error_msg);
	bool validateCollisions(const arr &traj, string &error_msg);
	void computePositionError(const ors::Shape &eef, const ors::Shape &target, ors::Vector &error);
	void computeAlignmentError(const ors::Shape &eef, const ors::Shape &target, ors::Vector &error);
	bool withinTolerance(const ors::Vector &error, const ors::Vector &tolerance);

	void setGripperJointPos(const string &joint, double pos);
	void setGripperState(const string &arm, const double *state);
	int getWorldJointIndex(const string &arm, int idx);
};

}

#endif // KOMO_WRAPPER_H
