#include <Ors/ors.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <robot_interface.h>

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

	return EXIT_SUCCESS;
}


