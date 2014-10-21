#include <Ors/ors.h>

int main(int argc, char *argv[])
{
	ors::KinematicWorld w("iis_robot.kvg");

	uint dim = w.getJointStateDimension();

	cout << "JointStateDimension: " << dim << endl;
	ors::Joint *jnt = w.getJointByName("right_arm_0_joint");
	cout << "Limits: " << jnt->limits << endl;

	return 0;
}

