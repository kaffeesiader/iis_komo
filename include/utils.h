#ifndef UTILS_H
#define UTILS_H

#include <iis_robot.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace std;
using namespace iis_komo;

#define JOINT(idx,arm) arm "_arm_" #idx "_joint"

void komo_path_to_joint_traj(const IISRobot::PlanninGroup &group,
							 const IISRobot::Path &path,
							 trajectory_msgs::JointTrajectory &trajectory) {

	// set joint names
	MT::Array<const char *> joint_names = IISRobot::get_jointnames_from_group(group);
	for (int i = 0; i < 7; ++i) {
		trajectory.joint_names.push_back(joint_names(i));
	}

	// create waypoints
	int num_points = path.d0;
	trajectory.points.resize(num_points);
	for (int i = 0; i < num_points; ++i) {
		const arr &wp = path[i];
		trajectory_msgs::JointTrajectoryPoint &point = trajectory.points[i];
		for (int j = 0; j < 7; ++j) {
			point.positions.push_back(wp(2*j+(int)group));
		}
	}

}

#endif // UTILS_H
