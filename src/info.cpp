#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_constrained.h>
#include <string>

using namespace std;

int main(int argc, char *argv[])
{
	ors::KinematicWorld w("iis_robot.kvg");

	for (int i = 0; i < 3; ++i) {
		ors::Vector axis;
		axis.setZero();
		axis(i) = 1.0;
		DefaultTaskMap dtm(vecAlignTMT, w, "obstacle1", axis, "obstacle2", axis);
		arr y;
		dtm.phi(y,NoArr,w);
		cout << "Test" << i << ": " << y << ", axis: " << axis << endl;
	}

	arr state(w.getJointStateDimension());
	state(0) = 4.5;
	state(1) = 4.5;
	w.setJointState(state);
//	w.calc_Q_from_q();

	arr y;
	LimitsConstraint lc;
	lc.margin = 0.1;
	lc.phi(y, NoArr, w);

	cout << "Limits: " << y << endl;

	DefaultTaskMap ltm(qLimitsTMT);
	ltm.phi(y,NoArr,w);
	cout << "Limits2:" << y << endl;

	ors::Joint *jnt = w.getJointByName("right_sdh_finger_12_joint");
	if(jnt) {
		jnt->Q.rot.setRad(1.57,1, 0, 0);
		cout << "right_sdh_finger_12_joint qIndex: " << jnt->qIndex << endl;
		w.calc_fwdPropagateFrames();
//		w.watch(true, "Joint state set!");
	}

	w.setAgent(1);
	cout << "joint_states_dimension: " << w.getJointStateDimension() << endl;
	cout << "right_sdh_finger_12_joint qIndex: " << jnt->qIndex << endl;


	ors::Shape *rgl = w.getShapeByName("left_sdh_grasp_link");
	if(!rgl) {
		cout << "Link not found!" << endl;
	} else {
		cout << "Link found!" << endl;
	}

	ors::Body *b = new ors::Body(w);

	ors::Shape *s = new ors::Shape(w, *b, NULL, false);
	s->type = ors::ShapeType::boxST;
	s->size[0] = 0.1;
	s->size[1] = 0.1;
	s->size[2] = 0.1;

	s->color[0] = 10.0;

	b->X.pos.x = 1.0;
	b->X.pos.y = 1.0;
	b->X.pos.z = 0.14;

	ors::Shape *o = w.getShapeByName("obstacle1");
	printf("R: %.2f G: %.2f  B: %.2f\n", o->color[0], o->color[1], o->color[2]);

	w.calc_fwdPropagateFrames();
//	w.watch(true);

	delete b;
//	delete s;

//	w.watch(true);

	return 0;
}

