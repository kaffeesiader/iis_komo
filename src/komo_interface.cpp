/**
 *
 *
 *
 *
 **/
#include <komo_interface.h>


namespace iis_komo {

KomoInterface::KomoInterface(const string &config_name)
{
    _world = new ors::KinematicWorld(config_name.c_str());
    // used in example - don't know if really necessary...
//    makeConvexHulls(_world->shapes);
    // enable collision checking for all shapes
    for(ors::Shape *s:_world->shapes) {
        s->cont = true;
    }
}

KomoInterface::~KomoInterface() {
    if(_world) {
        delete _world;
    }
}

bool KomoInterface::plan(const vector<double> &start_state, double x, double y, double z, arr &traj)
{
    // set initial position
    arr init_pos(&start_state[0], 7);
    _world->setJointState(init_pos);
    // set target position
    ors::Shape *target = _world->getShapeByName("target");
    ors::Vector targetPos(x, y, z);
    target->rel.pos = targetPos;

    ors::Shape *eef = _world->getShapeByName("endeff");

    // make planning request, using only position constraints.
    return planTo(*_world, *eef, *target, traj);
}


bool KomoInterface::plan(const vector<double> &start_state, double x, double y, double z, double roll, double pitch, double yaw, arr &traj)
{
    // set initial position
    arr init_pos(&start_state[0], 7);
    _world->setJointState(init_pos);

    ors::Shape *target = _world->getShapeByName("target");

    // set target position...
    ors::Vector target_pos(x, y, z);
    target->rel.pos = target_pos;

    // ...and orientation
    ors::Quaternion q;
    q.setRpy(roll, pitch, yaw);

    target->rel.rot = q;

    ors::Shape *eef = _world->getShapeByName("endeff");

    // make planning request and align all axes
    return planTo(*_world, *eef, *target, traj, 7);

}

bool KomoInterface::plan(const vector<double> &start_state, const string &goal_name, arr &traj)
{
    // set initial position
    arr init_pos(&start_state[0], 7);
    _world->setJointState(init_pos);

    ors::Shape *target = _world->getShapeByName(goal_name.c_str());

    if(target) {
        ors::Shape *eef = _world->getShapeByName("endeff");
        // make planning request and align all axes
        return planTo(*_world, *eef, *target, traj, 7);
    } else {
        cerr << "Unable to find shape with name '" << goal_name << "'" << endl;
        return false;
	}
}

bool KomoInterface::plan(const string eef_name, const string goal_name, arr &traj)
{
	ors::Shape *eef = _world->getShapeByName(eef_name.c_str());
	ors::Shape *target = _world->getShapeByName(goal_name.c_str());
	if(!target || !eef) {
		cerr << "Unable to handle planning request! Either end effector or goal with given name does not exist!";
		return false;
	}
	return planTo(*_world, *eef, *target, traj, 7);
}

void KomoInterface::display()
{
    _world->watch(true);
}

bool KomoInterface::planTo(ors::KinematicWorld &world,
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

	//-- set up the MotionProblem
	target.cont=false;

	MotionProblem MP(world);
	//  MP.loadTransitionParameters(); //->move transition costs to tasks!
	world.swift().initActivations(world);
	MP.world.watch(true);

	TaskCost *c;
	c = MP.addTask("endeff_pos", new DefaultTaskMap(posTMT, endeff.index, NoVector, target.index, NoVector));
	c->setCostSpecs(MP.T, MP.T, {0.}, posPrec);

	c = MP.addTask("endeff_vel", new DefaultTaskMap(posTMT, world, "endeff")); //endeff.index));
	//  c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, world));
	c->setCostSpecs(MP.T, MP.T, {0.}, zeroVelPrec);
	c->map.order=1; //make this a velocity variable!

	if(colPrec<0){ //interpreted as hard constraint
		c = MP.addTask("collisionConstraints", new CollisionConstraint(margin));
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

	arr final_state = x[x.d0-1];
	world.setJointState(final_state);

//	cout << "Last point: " << final_state << endl;
	cout << "EEF final pos: " << endeff.X.pos << endl;
	cout << "Target pos:    " << target.X.pos << endl;
//	cout << "EEF final rot: " << endeff.X.rot << endl;

	traj = x;

	//    world.watch(true);
	displayTrajectory(x, 1, world, "trajectory", 0.05);

	return true;
}

}

