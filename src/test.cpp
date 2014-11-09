#include <Ors/ors.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <robot_interface.h>
#include <iis_robot.h>

using namespace ros;
using namespace std;


int main(int argc,char** argv) {

	double d[] = {0.0,1.0,2.0,3.0,4.0,5.0,6.0};
	arr test(&d[0], 7);

	cout << "Test: " << test << endl;

	vector<double> v;
	v.push_back(0.0);
	v.push_back(1.0);
	v.push_back(2.0);
	v.push_back(3.0);
	v.push_back(4.0);
	v.push_back(5.0);
	v.push_back(6.0);

	arr test2(&v[0], 7);
	cout << "Test2: " << test2 << endl;

	MT::Array<const char*> names = iis_komo::IISRobot::left_arm_get_jointnames();
	cout << "JointNames: " << names << endl;

	ors::Quaternion q1;
	q1.setRpy(1.5708, 1.5708, 0);

	ors::Vector vec = {0,0,1};
	double dbl;
	q1.getRad(dbl, vec);

	cout << "Quaternion: " << q1 << endl;
	cout << "vec\t" << vec << endl;
	cout << "dbl\t" << dbl << endl;
	vec.setZero();
//	vec(0) = 1.;
	cout << q1.getVec(vec) << endl;
	cout << q1.getMatrix() << endl;

	return EXIT_SUCCESS;
}


