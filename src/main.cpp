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

void executeTrajectory(arr trajectory)
{
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

    ROS_INFO("Planning for first target...");

    vector<double> init;
    for (int i = 0; i < 7; ++i) {
        init.push_back(0.0);
    }

    arr traj;
    if(ki.plan(init, 0.3, 0.3, 0.5, 1.5, 0.0, 0.0, traj)) {
        ROS_INFO("Trajectory planning request successful - executing...");
//        ki.execute(traj);
        ROS_INFO("Trajectory execution complete.");
    }

    ROS_INFO("Planning for second target...");
    if(ki.plan(init, -0.3, -0.3, 0.7, traj)) {
        ROS_INFO("Trajectory planning request successful - executing...");
//        ki.execute(traj);
        ROS_INFO("Trajectory execution complete.");
    }

    ROS_INFO("Test run complete");

}

//void test2() {

//    ors::KinematicWorld world("kuka.kvg");

//    ors::Shape *target = world.getShapeByName("target");
////    target->X.addRelativeTranslation(0.0, 0.0, 1.6);

//    cout << "Target body.x.pos: " << target->body->X.pos << endl;
//    cout << "Target rel.pos: " << target->rel.pos << endl;
//    cout << "Target X.pos: " << target->X.pos << endl;
//    world.watch(true);

//    target->X.addRelativeTranslation(0.1, 0.1, -0.1);

//    cout << "Target body.x.pos: " << target->body->X.pos << endl;
//    cout << "Target rel.pos: " << target->rel.pos << endl;
//    cout << "Target X.pos: " << target->X.pos << endl;

//    world.watch(true);

//    target->reset();
//    target->X.addRelativeTranslation(0.0, 0.0, 0.4);

//    cout << "Target body.x.pos: " << target->body->X.pos << endl;
//    cout << "Target rel.pos: " << target->rel.pos << endl;
//    cout << "Target X.pos: " << target->X.pos << endl;

//    world.watch(true);

//    target->X.pos = ors::Vector(0.0, 0.0, 1.3);

//    cout << "Target body.x.pos: " << target->body->X.pos << endl;
//    cout << "Target rel.pos: " << target->rel.pos << endl;
//    cout << "Target X.pos: " << target->X.pos << endl;

//    world.watch(true);

//}

int main(int argc,char** argv) {
    MT::initCmdLine(argc,argv);

    ros::init(argc, argv, "iis_komo_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    NodeHandle nh;
    _pub_move = nh.advertise<std_msgs::Float64MultiArray>(JOINT_MOVE_TOPIC, 1, false);

    iis_komo::KomoInterface ki("kuka.kvg");
    runTest(ki);

    spinner.stop();
    return EXIT_SUCCESS;
}


