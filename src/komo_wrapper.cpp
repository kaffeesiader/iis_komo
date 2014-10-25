/**
 *
 *
 *
 *
 **/
#include <komo_wrapper.h>
#include <utils.h>

using namespace std;

namespace iis_komo {

KomoWrapper::KomoWrapper(const string &config_name)
{
	_world = new ors::KinematicWorld(config_name.c_str());
	cout << _world->getJointStateDimension() << endl;
	CHECK(_world->getJointStateDimension() >= 14, "Provided ors description does not contain two arms!");
	// enable collision checking for all shapes
//	for(ors::Shape *s:_world->shapes) {
//        s->cont = true;
//	}

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
	cout << "Setting gripper joint positions is not implemented yet!!!" << endl;
}

void KomoWrapper::getGripperState(const string &arm, const double *state)
{
	cout << "Getting gripper joint positions is not implemented yet!!!" << endl;
}

void KomoWrapper::arrToPath(const arr &traj, vector<IISRobotState> &path)
{
	// traverse through all trajectory points
	for (int i = 0; i < traj.d0; ++i) {
		arr pt = traj[i];
		IISRobotState state;
		// extract the joint positions for each arm
		for (int j = 0; j < 7; ++j) {
			state.left_arm[j] = pt(getWorldJointIndex("left", j));
			state.right_arm[j] = pt(getWorldJointIndex("right", j));
		}
	}
}

bool KomoWrapper::plan(const string &eef, double x, double y, double z, const IISRobotState &start_state, vector<IISRobotState> &path)
{
	// set initial position
	setState(start_state);

	// set target position
	ors::Shape *target = _world->getShapeByName("target");
	if(!target) {
		cerr << "Unable to find shape with name '" << "target" << "'" << endl;
		return false;
	}

	ors::Vector targetPos(x, y, z);
	target->rel.pos = targetPos;

	ors::Shape *endeff = _world->getShapeByName(eef.c_str());
	if(!target) {
		cerr << "Unable to find shape with name '" << eef << "'" << endl;
		return false;
	}

	// make planning request, using only position constraints.
	arr traj;
	bool success = planTo(*_world, *endeff, *target, traj);
	if(success) {
		arrToPath(traj, path);
	}
	return success;
}

bool KomoWrapper::plan(const string &eef, double x, double y, double z, double roll, double pitch, double yaw, const IISRobotState &start_state, vector<IISRobotState> &path)
{
	// set initial position
	setState(start_state);

	ors::Shape *target = _world->getShapeByName("target");
	if(!target) {
		cerr << "Unable to find shape with name '" << "target" << "'" << endl;
		return false;
	}

	// set target position...
	ors::Vector target_pos(x, y, z);
	target->rel.pos = target_pos;

	// ...and orientation
	ors::Quaternion q;
	q.setRpy(roll, pitch, yaw);

	target->rel.rot = q;

	ors::Shape *endeff = _world->getShapeByName(eef.c_str());
	if(!target) {
		cerr << "Unable to find shape with name '" << eef << "'" << endl;
		return false;
	}

	// make planning request and align all axes
	arr traj;
	bool success = planTo(*_world, *endeff, *target, traj, 7);
	if(success) {
		arrToPath(traj, path);
	}
	return success;
}

bool KomoWrapper::plan(const string &eef, double x, double y, double z, double qx, double qy, double qz, double qw, const IISRobotState &start_state, vector<IISRobotState> &path)
{
	// set initial position
	setState(start_state);

	ors::Shape *target = _world->getShapeByName("target");
	if(!target) {
		cerr << "Unable to find shape with name '" << "target" << "'" << endl;
		return false;
	}

	// set target position...
	ors::Vector target_pos(x, y, z);
	target->rel.pos = target_pos;

	// ...and orientation
	ors::Quaternion q;
	q.set(qw, qx, qy, qz);

	target->rel.rot = q;

	ors::Shape *endeff = _world->getShapeByName(eef.c_str());
	if(!target) {
		cerr << "Unable to find shape with name '" << eef << "'" << endl;
		return false;
	}

	// make planning request and align all axes
	arr traj;
	bool success = planTo(*_world, *endeff, *target, traj, 7);
	if(success) {
		arrToPath(traj, path);
	}
	return success;
}

void KomoWrapper::display()
{
	_world->watch(true);
}


bool KomoWrapper::validateResult(const arr &traj, ors::Shape &eef, ors::Shape &target, bool position_only)
{
	cout << "TRAJECTORY VALIDATION IS NOT IMPLEMENTED YET!!!" << endl;
	return true;
}

bool KomoWrapper::planTo(ors::KinematicWorld &world,
                           ors::Shape &endeff,
                           ors::Shape &target,
                           arr &traj,
                           byte whichAxesToAlign,
                           uint iterate)
{
	//-- parameters
	double posPrec = MT::getParameter<double>("KOMO/moveTo/precision", 1e4); // original 1e3
	double colPrec = MT::getParameter<double>("KOMO/moveTo/collisionPrecision", -1e0);
	double margin = MT::getParameter<double>("KOMO/moveTo/collisionMargin", .1);
	double zeroVelPrec = MT::getParameter<double>("KOMO/moveTo/finalVelocityZeroPrecision", 1e1);
	double alignPrec = MT::getParameter<double>("KOMO/moveTo/alignPrecision", 1e4); // original 1e3

	cout << "MARGIN: " << margin << endl;
	cout << "COLPRE: " << colPrec << endl;

	//-- set up the MotionProblem
	target.cont=false;

	MotionProblem MP(world);
	//  MP.loadTransitionParameters(); //->move transition costs to tasks!
	world.swift().initActivations(world);

	TaskCost *c;
	c = MP.addTask("endeff_pos", new DefaultTaskMap(posTMT, endeff.index, NoVector, target.index, NoVector));
	c->setCostSpecs(MP.T, MP.T, {0.}, posPrec);

	c = MP.addTask("endeff_vel", new DefaultTaskMap(posTMT, world, endeff.name)); //endeff.index));
	//  c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, world));
	c->setCostSpecs(MP.T, MP.T, {0.}, zeroVelPrec);
	c->map.order=1; //make this a velocity variable!

	if(colPrec<0){ //interpreted as hard constraint
		c = MP.addTask("collision", new CollisionConstraint(margin));
	}else{ //cost term
		c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, margin));
	}
	c->setCostSpecs(0, MP.T, {0.}, colPrec);

	c = MP.addTask("transitions", new TransitionTaskMap(world));
	c->map.order=2;
	c->setCostSpecs(0, MP.T, {0.}, 1e0);

	for(uint i=0;i<3;i++) if(whichAxesToAlign&(1<<i)){
		ors::Vector axis;
		axis.setZero();
		axis(i)=1.;
		c = MP.addTask(STRING("endeff_align_"<<i),
					   new DefaultTaskMap(vecAlignTMT, endeff.index, axis, target.index, axis));
		c->setCostSpecs(MP.T, MP.T, {1.}, alignPrec);
	}

	//-- create the Optimization problem (of type kOrderMarkov)
	MotionProblemFunction MF(MP);
	arr x = replicate(MP.x0, MP.T+1);
	rndGauss(x,.01,true); //don't initialize at a singular config

	//-- optimize
	ors::KinematicWorld::setJointStateCount=0;
	for(uint k=0;k<iterate;k++){
		MT::timerStart();
		if(colPrec<0){
			optConstrained(x, NoArr, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=.5, stepInc=2., allowOverstep=false));
			//verbose=1, stopIters=100, maxStep=.5, stepInc=2./*, nonStrictSteps=(!k?15:5)*/));
		}else{
			optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=.5, stepInc=2., nonStrictSteps=(!k?15:5)));
		}
		cout <<"** optimization time=" <<MT::timerRead()
			 <<" setJointStateCount=" <<ors::KinematicWorld::setJointStateCount <<endl;
		//    checkJacobian(Convert(MF), x, 1e-5);
		MP.costReport();
	}

//	arr final_state = x[x.d0-1];
//	world.setJointState(final_state);

//	cout << "Last point: " << final_state << endl;
	cout << "EEF final pos: " << endeff.X.pos << endl;
	cout << "Target pos:    " << target.X.pos << endl;
//	cout << "EEF final rot: " << endeff.X.rot << endl;

	if(validateResult(x, endeff, target)) {
		traj = x;
		displayTrajectory(x, 1, world, "Planning result", 0.05);
		cout << "Optimization process finished" << endl;
		return true;
	} else {
		return false;
	}

}

}
