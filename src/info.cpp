#include <Ors/ors.h>
#include <string>

using namespace std;

int main(int argc, char *argv[])
{
	ors::KinematicWorld w("iis_robot.kvg");

	cout << "joint_states_dimension (agent 0): " << w.getJointStateDimension() << endl;
	ors::Joint *jnt = w.getJointByName("left_arm_6_joint");
	cout << "left_arm_6_joint: qIndex=" << jnt->qIndex << ", index=" << jnt->index << endl;

	w.setAgent(1);
	cout << "joint_states_dimension (agent 1): " << w.getJointStateDimension() << endl;
	cout << "left_arm_6_joint: qIndex=" << jnt->qIndex << ", index=" << jnt->index << endl;

	jnt = w.getJointByName("right_arm_6_joint");
	cout << "right_arm_6_joint: qIndex=" << jnt->qIndex << ", index=" << jnt->index << endl;

	w.setAgent(2);
	cout << "joint_states_dimension (agent 2): " << w.getJointStateDimension() << endl;

	w.watch(true);

	return 0;
}

