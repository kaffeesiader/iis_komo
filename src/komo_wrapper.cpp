/**
 *
 *
 *
 *
 **/
#include <komo_wrapper.h>
#include <utils.h>
#include <ros/ros.h>


using namespace std;

namespace iis_komo {


KomoWrapper::KomoWrapper(const string &config_name)
{
	CHECK(!config_name.empty(), "Unable to create KinematicWorld - parameter 'config_name' cannot be empty!");
	_world = new ors::KinematicWorld(config_name.c_str());
	CHECK(_world->getJointStateDimension() >= 14, "Wrong joint state dimension! Provided ors description is invalid!");
	// enable collision checking for all shapes
	for(ors::Shape *s:_world->shapes) {
		s->cont = true;
	}

	double def_pos_tolerance = MT::getParameter<double>("KOMO/moveTo/defaultPositionTolerance", 0.005);
	double def_ang_tolerance = MT::getParameter<double>("KOMO/moveTo/defaultAngularTolerance", 0.1);

	_pos_tolerance = {def_pos_tolerance, def_pos_tolerance, def_pos_tolerance};
	_ang_tolerance = {def_ang_tolerance, def_ang_tolerance, def_ang_tolerance};

	//-- parameters
	_positionPrecision = MT::getParameter<double>("KOMO/moveTo/positionPrecision", 1e4); // original 1e3
	_collisionPrecision = MT::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
	_collisionMargin = MT::getParameter<double>("KOMO/moveTo/collisionMargin", .1);
	_jointLimitPrecision = MT::getParameter<double>("KOMO/moveTo/jointLimitPrecision", 0.1);
	_jointLimitMargin = MT::getParameter<double>("KOMO/moveTo/jointLimitMargin", 1e5);
	_zeroVelocityPrecision = MT::getParameter<double>("KOMO/moveTo/zeroVelocityPrecision", 1e1);
	_alignmentPrecision = MT::getParameter<double>("KOMO/moveTo/alignmentPrecision", 1e4); // original 1e3
	_iterations = MT::getParameter<double>("KOMO/moveTo/iterations", 1);
}

KomoWrapper::~KomoWrapper() {
    if(_world) {
        delete _world;
	}
}

int KomoWrapper::getWorldJointIndex(const string &arm, int idx)
{
	char name[20];
	sprintf(name, "%s_arm_%d_joint", arm.c_str(), idx);
	ors::Joint *jnt = _world->getJointByName(name);
	if(jnt) {
		return jnt->index;
	} else {
		return -1;
	}
}

void KomoWrapper::setState(const IISRobotState &state)
{
	arr new_state(_world->getJointStateDimension());
	// set the joint positions of the two arms
	for(int i = 0; i < 7; ++i) {
		int left_idx = getWorldJointIndex("left", i);
		int right_idx = getWorldJointIndex("right", i);
		new_state(left_idx) = state.left_arm[i];
		new_state(right_idx) = state.right_arm[i];
	}
	_world->setJointState(new_state);
	// set the states of the grippers
	// this has to be done in a different way because gripper joints are fixed...
	setGripperState("left", state.left_sdh);
	setGripperState("right", state.right_sdh);
	_world->calc_fwdPropagateFrames();
}

void KomoWrapper::setGripperState(const string &arm, const double *state)
{
	setGripperJointPos(arm + "_sdh_knuckle_joint", state[0]);
	setGripperJointPos(arm + "_sdh_finger_12_joint", state[1]);
	setGripperJointPos(arm + "_sdh_finger_13_joint", state[2]);
	setGripperJointPos(arm + "_sdh_thumb_2_joint", state[3]);
	setGripperJointPos(arm + "_sdh_thumb_3_joint", state[4]);
	setGripperJointPos(arm + "_sdh_finger_22_joint", state[5]);
	setGripperJointPos(arm + "_sdh_finger_23_joint", state[6]);
	// set state of mirrored joint
	setGripperJointPos(arm + "_sdh_finger_21_joint", -state[0]);
}

void KomoWrapper::setGripperJointPos(const string &joint, double pos)
{
	ors::Joint *jnt = _world->getJointByName(joint.c_str());
	if(!jnt) {
		cerr << "Unable to set gripper joint position - no joint with name '" << joint << "' found in model!";
	} else {
		jnt->Q.rot.setRad(pos, 1, 0, 0);
	}
}

bool KomoWrapper::plan(IISRobot::PlanninGroup group, const string &eef,
					   const ors::Vector &goal_pos, const ors::Quaternion &goal_orient,
					   bool position_only, IISRobot::Path &path,
					   ors::Vector &pos_error, ors::Vector &ang_error, string &error_msg)
{
	ors::Shape *target = _world->getShapeByName("target");
	if(!target) {
		error_msg = "Unable to find shape with name 'target' within model.";
		cerr << error_msg << endl;
		return false;
	}

	ors::Shape *endeff = _world->getShapeByName(eef.c_str());
	if(!endeff) {
		error_msg = "Unable to find link with name '" + eef + "' within model.";
		cerr << error_msg << endl;
		return false;
	}

	// set target position...
	target->rel.pos = goal_pos;
	// ...and orientation
	target->rel.rot = goal_orient;

	// make planning request and align all axes
	// return planTo(*_world, *endeff, *target, path, 7);

	_world->calc_fwdPropagateShapeFrames();
	display(false, "planning...");

	target->cont = false; // don't know if this is necessary...

	//-- set up the MotionProblem
	MotionProblem MP(*_world);
//	MP.loadTransitionParameters(); //->move transition costs to tasks!
	_world->swift().initActivations(*_world);

	TaskCost *c;
	// compute the range of time slices where the position constraint has to be enforced (last 10%)
	uint offset = (uint)(MP.T * 0.1);

	// TaskMap for end effector position
	c = MP.addTask("EEF_position", new DefaultTaskMap(posTMT, endeff->index, NoVector, target->index, NoVector));
	c->setCostSpecs(MP.T-offset, MP.T, {0.}, _positionPrecision);

	// TaskMap for zero velocity at goal
	c = MP.addTask("EEF_velocity", new DefaultTaskMap(posTMT, *_world, endeff->name)); //endeff.index));
	//  c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, world));
	c->setCostSpecs(MP.T, MP.T, {0.}, _zeroVelocityPrecision);
	c->map.order = 1; //make this a velocity variable!

	// TaskMap to enforce joint limits on all time slices
	c = MP.addTask("Joint_limits", new DefaultTaskMap(qLimitsTMT));
	c->setCostSpecs(0, MP.T, {0.}, _jointLimitPrecision);

	// enable collision checking
	if(_collisionPrecision < 0){ //interpreted as hard constraint
		c = MP.addTask("Collisions", new CollisionConstraint(_collisionMargin));
	}else{ //cost term
		c = MP.addTask("Collisions", new ProxyTaskMap(allPTMT, {0}, _collisionMargin));
	}
	c->setCostSpecs(0, MP.T, {0.}, _collisionPrecision);

	// TaskMap for transition costs
	c = MP.addTask("Transitions", new TransitionTaskMap(*_world));
	c->map.order=2;
	c->setCostSpecs(0, MP.T, {0.}, 1e0);

	// TaskMaps for orientation constraints
	if(!position_only) {
		for(uint i=0;i<3;i++) {
			ors::Vector axis;
			axis.setZero();
			axis(i) = 1.;
			c = MP.addTask(STRING("EEF_allign_" << i), new DefaultTaskMap(vecAlignTMT, endeff->index, axis, target->index, axis));
			c->setCostSpecs(MP.T-offset, MP.T, {1.}, _alignmentPrecision);
		}
	}

	//-- create the Optimization problem (of type kOrderMarkov)
	MotionProblemFunction MF(MP);
	arr x = replicate(MP.x0, MP.T+1);
	rndGauss(x, .01, true); //don't initialize at a singular config

	//-- optimize
	ors::KinematicWorld::setJointStateCount = 0;
	for(uint k = 0; k < _iterations; k++) {
		MT::timerStart();
		if(_collisionPrecision < 0){
			// verbose=2 shows gnuplot after optimization process
			// verbose=1 shows optimization steps
			optConstrained(x, NoArr, Convert(MF), OPT(verbose=1, stopIters=100, maxStep=.5, stepInc=2., allowOverstep=false));
			//verbose=2, stopIters=100, maxStep=.5, stepInc=2./*, nonStrictSteps=(!k?15:5)*/));
		} else {
			optNewton(x, Convert(MF), OPT(verbose=1, stopIters=100, maxStep=.5, stepInc=2., nonStrictSteps=(!k?15:5)));
		}
		double opt_time = MT::timerRead();
		uint set_jnt_state_cnt = ors::KinematicWorld::setJointStateCount;
		cout <<"** optimization time=" << opt_time
			 <<" setJointStateCount=" << set_jnt_state_cnt;
		//    checkJacobian(Convert(MF), x, 1e-5);
		MP.costReport(false);
	}

	CHECK(x.d0 > 0, "Trajectory is empty...");

	cout << "Optimization process finished" << endl;
	cout << "Optimization time:  " << MT::timerRead() << endl;
	cout << "SetJointStateCount: " << ors::KinematicWorld::setJointStateCount << endl;
	cout << "Validating planning result..." << endl;

	arr final_state = x[x.d0-1];
	_world->setJointState(final_state);
	_world->calc_fwdPropagateFrames();

	computePositionError(*endeff, *target, pos_error);
	computeAlignmentError(*endeff, *target, ang_error);

	cout << "EEF final pos:   " << endeff->X.pos << endl;
	cout << "EEF target pos:  " << target->X.pos << endl;
	cout << "Position error:  " << pos_error << endl;
	cout << "Alignment error: " << ang_error << endl;

	cout << "Displaying trajectory..." << endl;
	// displayTrajectory(x, -1, *_world, "Planning result", 0.05); // do not block after displaying...
	displayTrajectory(x, 1, *_world, "Planning result", 0.05);
	cout << "Validating trajectory..." << endl;

	// check end effector goal position
	if(!withinTolerance(pos_error, _pos_tolerance)) {
		error_msg = "Goal position not within tolerance values.";
		cerr << "Validation failed!" << endl;
		return false;
	}

	// check end effector goal alignment
	if(!withinTolerance(ang_error, _ang_tolerance)) {
		error_msg = "Goal alignment not within tolerance values.";
		cerr << "Validation failed!" << endl;
		return false;
	}

	// check joint limits
	if(!validateJointLimits(x, error_msg)) {
		cerr << "Validation failed!" << endl;
		return false;
	}

	// watch out for collisions
	if(!validateCollisions(x, error_msg)) {
		cerr << "Validation failed!" << endl;
		return false;
	}

	cout << "Trajectory validation ok!" << endl;
	path = x;

	return true;
}

void KomoWrapper::display(bool block, const char *msg)
{
	_world->watch(block, msg);
}

void KomoWrapper::computePositionError(const ors::Shape &eef, const ors::Shape &target, ors::Vector &error) {
	error = eef.X.pos - target.X.pos;
}

void KomoWrapper::computeAlignmentError(const ors::Shape &eef, const ors::Shape &target, ors::Vector &error) {
	cerr << "ALIGNMENT ERROR COMPUTATION NOT IMPLEMENTED YET!!!" << endl;
//	ors::Quaternion diff = eef.X.rot - target.X.rot;
	//	cout << "Distance between quats: " << diff << endl;
}

bool KomoWrapper::withinTolerance(const ors::Vector &error, const ors::Vector &tolerance)
{
	return (fabs(error.x) <= tolerance.x) &&
			(fabs(error.y) <= tolerance.y) &&
			(fabs(error.z) <= tolerance.z);
}

bool KomoWrapper::validateJointLimits(const arr &traj, string &error_msg)
{
	arr limits = IISRobot::get_arm_joint_limits();

	// validation steps neccessary for each single point
	for(int i = 0; i < traj.d0 - 1; ++i) {
		arr pt = traj[i];
		// validate joint limits
		for (int j = 0; j < 7; ++j) {
			int left_index = getWorldJointIndex("left", j);
			int right_index = getWorldJointIndex("right", j);
			double left_pos = pt(left_index);
			double right_pos = pt(right_index);
			double limit = limits(j);

			if(fabs(left_pos) > limit) {
				stringstream ss;
				ss << "Joint limit violated for joint " << j << " in left arm! Value: " << left_pos;
				error_msg = ss.str();
				cerr << error_msg << endl;
				return false;
			}
			if(fabs(right_pos) > limit) {
				stringstream ss;
				ss << "Joint limit violated for joint " << j << " in right arm! Value: " << left_pos;
				error_msg = ss.str();
				cerr << error_msg << endl;
				return false;
			}
		}
	}

	return true;
}

bool KomoWrapper::validateCollisions(const arr &traj, string &error_msg)
{
	cerr << "COLLISION VALIDATION NOT IMPLEMENTED YET!!!" << endl;
	return true;
}

}
