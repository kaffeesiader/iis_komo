#include <Ors/ors.h>
#include <Motion/komo.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <komo_interface.h>


#define JOINT_STATES_TOPIC "/simulation/right_arm/joint_control/get_state"
#define JOINT_MOVE_TOPIC "/simulation/right_arm/joint_control/move"


using namespace ros;
using namespace std;

ros::Publisher _pub_move;
bool _execution_allowed;

void executeTrajectory(arr trajectory)
{

    if(!_execution_allowed) {
        return;
    }

    std_msgs::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].size = 7;
    msg.data.resize(7);

    double hz = 100;
    ros::Rate loop_rate(hz);

    int size = trajectory.d0 - 1;

    // iterate over trajectory points...
    for (int i = 1; i < size; ++i) {
        arr current_state = trajectory[i-1];
        arr next_state = trajectory[i];
        arr diff = next_state - current_state;
        // ... calculate interpolation step based on loop rate ...
        arr step = diff / 10.0;
        // ... and move subsequently towards next position.
        for (int j = 0; j < 10; ++j) {
            current_state += step;
            for(int k = 0; k < 7; ++k) {
                msg.data[k] = current_state(k);
            }
            _pub_move.publish(msg);
            loop_rate.sleep();
        }
    }
}

bool getState(vector<double> &state) {
    ROS_DEBUG("Waiting for current state...");
    sensor_msgs::JointStateConstPtr current_state =  ros::topic::waitForMessage<sensor_msgs::JointState>(JOINT_STATES_TOPIC, ros::Duration(3.0));
    if(!current_state) {
        ROS_WARN("Unable to retrieve initial joint state!");
        state.clear();
        for (int i = 0; i < 7; ++i) {
            state.push_back(0.0);
        }
        return false;
    }
    state = current_state->position;
    ROS_DEBUG("Initial state received.");
    return true;
}

void runTest(iis_komo::KomoInterface &ki) {

    ROS_INFO("Starting test run...");

    ROS_INFO("Planning for first target (position only)...");

    vector<double> jnt_state;
    getState(jnt_state);

    arr traj;
    if(ki.plan(jnt_state, 0.5, 0.0, 0.8, traj)) {
        ROS_INFO("Trajectory planning request successful - executing...");
        executeTrajectory(traj);
        ROS_INFO("Trajectory execution complete.");
    }

    ROS_INFO("Planning for named goals...");

    int i = 1;
    while(true) {
        // iterate through named goals...
        stringstream ss;
        ss << "goal" << i;
        string goal_name = ss.str();
        cout << goal_name << " " << i << endl;

        getState(jnt_state);
        if(ki.plan(jnt_state, goal_name, traj)) {
            ROS_INFO_STREAM("Trajectory planning for " << goal_name << " successful - executing...");
            executeTrajectory(traj);
            ROS_INFO("Trajectory execution complete.");
            i++;
        } else {
            break;
        }
    }

    ROS_INFO("Test run complete");

}

int main(int argc,char** argv) {
    MT::initCmdLine(argc,argv);

    ros::init(argc, argv, "iis_komo_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    NodeHandle nh;
    _pub_move = nh.advertise<std_msgs::Float64MultiArray>(JOINT_MOVE_TOPIC, 1, false);
    _execution_allowed = true;

    iis_komo::KomoInterface ki("kuka.kvg");
    runTest(ki);

    spinner.stop();
    return EXIT_SUCCESS;
}


