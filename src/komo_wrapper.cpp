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
	// enable collision checking for all shapes
	for(ors::Shape *s:_world->shapes) {
		s->cont = true;
	}
}

KomoWrapper::~KomoWrapper() {
    if(_world) {
        delete _world;
    }
}

void KomoWrapper::setState(const IISRobotState &state)
{
	// set the state for each single agent
//	setState(IISRobot::LeftArm, state.left_arm);
//	setState(IISRobot::RightArm, state.right_arm);
//	setState(IISRobot::LeftSDH, state.left_sdh);
//	setState(IISRobot::RightSDH, state.right_sdh);
}

void KomoWrapper::setState(IISRobot::PlanninGroup group, const double values[])
{
	// switch to selected agent...
	_world->setAgent(group);
	uint jsDim = _world->getJointStateDimension();
	// get the joint names of specified agent...
	MT::Array<const char*> names = IISRobot::get_jointnames_from_group(group);
	CHECK(names.N == jsDim, "Unable to set joint states - wrong number of joints");
	arr state(jsDim);
	// ... and set each joint to desired position.
	for (int i = 0; i < jsDim; ++i) {
		const char *name = names(i);
		ors::Joint *jnt = _world->getJointByName(name);
		CHECK(jnt, "Unable to set joint position - joint '" << name << "' not found in model!");
		state(jnt->qIndex) = values[i];
	}

	_world->setJointState(state);
}

void KomoWrapper::arrToPath(const arr &traj, IISRobot::Path &path)
{
//	// traverse through all trajectory points
//	for (int i = 0; i < traj.d0; ++i) {
//		arr pt = traj[i];
//		IISRobot::ArmState state;
//		for (int j = 0; j < 7; ++j) {
//			state[j] = pt(j);
//		}
//		// add extracted state to path
//		path.push_back(state);
//	}
}

void KomoWrapper::display(bool block, const char *msg)
{
	_world->watch(block, msg);
}

void KomoWrapper::addShape()
{
	ors::Body *b = new ors::Body(*_world);
	ors::Shape *s = new ors::Shape(*_world, *b, NULL, false);

	s->type = ors::ShapeType::boxST;
	s->size[0] = 0.1;
	s->size[1] = 0.1;
	s->size[2] = 0.1;

	s->color[0] = 10.0;

	b->X.pos.x = 1.0;
	b->X.pos.y = 1.0;
	b->X.pos.z = 0.14;

//	ors::Shape *o = _world->getShapeByName("obstacle1");
//	printf("R: %.2f G: %.2f  B: %.2f\n", o->color[0], o->color[1], o->color[2]);

	_collision_objects.push_back(b);
	_world->calc_fwdPropagateFrames();
}

bool KomoWrapper::validateResult(const arr &traj, ors::Shape &eef, ors::Shape &target, bool position_only)
{
	arr final_state = traj[traj.d0-1];
	_world->setJointState(final_state);
	_world->calc_fwdPropagateFrames();

	cout << "EEF final pos:  " << eef.X.pos << endl;
	cout << "EEF target pos: " << target.X.pos << endl;

	// check end effector position
	double epsilon = 0.020;
	ors::Vector dist = target.X.pos - eef.X.pos;
	cout << "Position offset: " << dist.length() << endl;

	if(dist.length() > epsilon) {
		cerr << "Trajectory validation failed - position constraint not satisfied." << endl;
		return false;
	}
	// ...and orientation if required
	if(!position_only) {
		cout << "ORIENTATION CONSTRAINT VALIDATION NOT IMPLEMENTED YET!" << endl;
	}
	// arm joint limits
	double limits[] = {
		2.96705972839,
		2.09439510239,
		2.96705972839,
		2.09439510239,
		2.96705972839,
		2.09439510239,
		2.96705972839
					  };

	// validation steps neccessary for each single point
	for(int i = 0; i < traj.d0 - 1; ++i) {
		arr pt = traj[i];
		// validate joint limits
		for (int j = 0; j < 7; ++j) {
			if(pt(j) > limits[j] || pt(j) < -limits[j]) {
				cerr << "Joint limit violated for joint " << j << "! Value: " << pt(j) << endl;
				return false;
			}
		}
	}

	cout << "Trajectory validation OK." << endl;
	return true;
}

/* ---------------------------- planning related ----------------------------------*/

bool KomoWrapper::plan(const string &eef, const IISRobot::PlanninGroup group, double x, double y, double z, IISRobot::Path &path)
{
	// set target position
	ors::Shape *target = _world->getShapeByName("target");
	if(!target) {
		cerr << "Unable to find shape with name '" << "target" << "'" << endl;
		return false;
	}

	ors::Vector targetPos(x, y, z);
	target->rel.pos = targetPos;

	ors::Shape *endeff = _world->getShapeByName(eef.c_str());
	if(!endeff) {
		cerr << "Unable to find shape with name '" << eef << "'" << endl;
		return false;
	}

	// make planning request, using only position constraints.
	return planTo(*_world, *endeff, *target, (uint)group, path);
}

bool KomoWrapper::plan(const string &eef, const IISRobot::PlanninGroup group, double x, double y, double z, double roll, double pitch, double yaw, IISRobot::Path &path)
{
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
	if(!endeff) {
		cerr << "Unable to find shape with name '" << eef << "'" << endl;
		return false;
	}

	// make planning request and align all axes
	return planTo(*_world, *endeff, *target, (uint)group, path, 7);
}

bool KomoWrapper::plan(const string &eef, const IISRobot::PlanninGroup group, double x, double y, double z, double qx, double qy, double qz, double qw, IISRobot::Path &path)
{
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
	if(!endeff) {
		cerr << "Unable to find shape with name '" << eef << "'" << endl;
		return false;
	}

	// make planning request and align all axes
	return planTo(*_world, *endeff, *target, (uint)group, path, 7);
}

bool KomoWrapper::planTo(ors::KinematicWorld &world,
						   ors::Shape &endeff,
						   ors::Shape &target,
						   uint agent,
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
	double iterations = MT::getParameter<double>("KOMO/moveTo/iterations", 1);

	cout << "Position precision     : " << posPrec << endl;
	cout << "Align precision        : " << alignPrec << endl;
	cout << "Collision margin       : " << margin << endl;
	cout << "Zero velocity precision: " << zeroVelPrec << endl;
	cout << "Collision precision    : " << colPrec << endl;
	cout << "Iterations             : " << iterations << endl;

	// switch to selected planning group
	_world->setAgent(agent);
	// ensure that the specified end effector is part of selected planning group
	if(!_world->getShapesByAgent(agent).contains(&endeff)) {
		cerr << "The end effector '" << endeff.name << "' is not part of selected planning group." << endl;
		return false;
	}

	_world->calc_fwdPropagateShapeFrames();
	display(false, "planning...");
	//-- set up the MotionProblem
	target.cont=false;

	MotionProblem MP(world);
//	MP.loadTransitionParameters(); //->move transition costs to tasks!
	world.swift().initActivations(world);
//	arr W(world.getJointStateDimension());
//	for (int i = 0; i < world.getJointStateDimension(); ++i) {
//		W(i) = 1.0;
//	}
//	MP.H_rate_diag = W;

	TaskCost *c;
	c = MP.addTask("End effector pos", new DefaultTaskMap(posTMT, endeff.index, NoVector, target.index, NoVector));
	c->setCostSpecs(MP.T, MP.T, {0.}, posPrec);

	c = MP.addTask("End effector vel", new DefaultTaskMap(posTMT, world, endeff.name)); //endeff.index));
	//  c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, world));
	c->setCostSpecs(MP.T, MP.T, {0.}, zeroVelPrec);
	c->map.order=1; //make this a velocity variable!

	if(colPrec<0){ //interpreted as hard constraint
		c = MP.addTask("Collision const.", new CollisionConstraint(margin));
	}else{ //cost term
		c = MP.addTask("Collision cost", new ProxyTaskMap(allPTMT, {0}, margin));
	}
	c->setCostSpecs(0, MP.T, {0.}, colPrec);

	c = MP.addTask("Transition costs", new TransitionTaskMap(world));
	c->map.order=2;
	c->setCostSpecs(0, MP.T, {0.}, 1e0);

	for(uint i=0;i<3;i++) if(whichAxesToAlign&(1<<i)){
		ors::Vector axis;
		axis.setZero();
		axis(i)=1.;
		c = MP.addTask(STRING("End effector orient"<<i),
					   new DefaultTaskMap(vecAlignTMT, endeff.index, axis, target.index, axis));
		c->setCostSpecs(MP.T, MP.T, {1.}, alignPrec);
	}

	//-- create the Optimization problem (of type kOrderMarkov)
	MotionProblemFunction MF(MP);
	arr x = replicate(MP.x0, MP.T+1);
	rndGauss(x,.01,true); //don't initialize at a singular config

	//-- optimize
	ors::KinematicWorld::setJointStateCount=0;
	for(uint k=0;k<iterations;k++){
		MT::timerStart();
		if(colPrec<0){
			// verbose=2 shows gnuplot after optimization process
			// verbose=1 shows optimization steps
			optConstrained(x, NoArr, Convert(MF), OPT(verbose=1, stopIters=100, maxStep=.5, stepInc=2., allowOverstep=false));
			//verbose=2, stopIters=100, maxStep=.5, stepInc=2./*, nonStrictSteps=(!k?15:5)*/));
		}else{
			optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, maxStep=.5, stepInc=2., nonStrictSteps=(!k?15:5)));
		}
		double opt_time = MT::timerRead();
		uint set_jnt_state_cnt = ors::KinematicWorld::setJointStateCount;
		cout <<"** optimization time=" << opt_time
			 <<" setJointStateCount=" << set_jnt_state_cnt;
		//    checkJacobian(Convert(MF), x, 1e-5);
		MP.costReport(false);
	}

	cout << "Optimization process finished" << endl;
	cout << "Optimization time:  " << MT::timerRead() << endl;
	cout << "SetJointStateCount: " << ors::KinematicWorld::setJointStateCount << endl;
	cout << "Validating planning result..." << endl;

	bool pos_only = (whichAxesToAlign == 0);
	bool valid = validateResult(x, endeff, target, pos_only);

	if(valid) {
		traj = x;
	}

	cout << "Displaying trajectory..." << endl;
	displayTrajectory(x, 1, world, "Planning result", 0.05);

	world.watch(false, "Ready...");

}

}
