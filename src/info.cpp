#include <Ors/ors.h>
#include <string>

using namespace std;

int main(int argc, char *argv[])
{
	ors::KinematicWorld w("iis_robot.kvg");

	ors::Joint *jnt = w.getJointByName("right_sdh_finger_22_joint");
	if(jnt) {
		jnt->Q.rot.setRad(1.57,1, 0, 0);
		w.calc_fwdPropagateFrames();
		w.watch(true, "Joint state set!");
	}

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
	w.watch(true);

	delete b;
//	delete s;

	w.watch(true);

	return 0;
}

