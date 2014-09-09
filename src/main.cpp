#include <Ors/ors.h>
#include <Motion/komo.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

using namespace ros;
using namespace std;

arr plan(const vector<double> &start_state) {

    ors::KinematicWorld G("kuka.kvg");
    ors::Shape *t = G.getShapeByName("target");
  //  makeConvexHulls(G.shapes);

    for(ors::Shape *s:G.shapes) s->cont=true;

    // set initial position and display
    arr init_pos(&start_state[0], 7);

    G.setJointState(init_pos);
    ors::Shape *eef = G.getShapeByName("endeff");

    arr x = moveTo(G, *eef, *t, 7);
    return x;

  //  s->rel.addRelativeTranslation(-0.2,0.2,-0.2);
  //  s->rel.addRelativeRotationDeg(90,0,0,1);
  //  x = moveTo(G, *G.getShapeByName("endeff"), *s, 7);
  //  for(uint i=0;i<1;i++) displayTrajectory(x, 1, G, "planned trajectory", 0.01);

  //  s->rel.addRelativeRotationDeg(90,1,0,0);
  //  x = moveTo(G, *G.getShapeByName("endeff"), *s, 7);
  //  for(uint i=0;i<1;i++) displayTrajectory(x, 1, G, "planned trajectory", 0.01);
}

bool execute(NodeHandle &nh, arr &trajectory) {

    Publisher move_pub = nh.advertise<std_msgs::Float64MultiArray>("/simulation/right_arm/joint_control/move", 1, false);

    std_msgs::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].size = 7;
    msg.data.resize(7);

    double hz = 100;
    ros::Rate loop_rate(hz);

    int size = trajectory.d0 - 1;

    for (int i = 1; i < size; ++i) {
        arr current_state = trajectory[i-1];
        arr next_state = trajectory[i];
        arr diff = next_state - current_state;
        arr step = diff / 10.0;

        for (int j = 0; j < 10; ++j) {
            current_state += step;
            for(int k = 0; k < 7; ++k) {
                msg.data[k] = current_state(k);
            }
            move_pub.publish(msg);
            loop_rate.sleep();
        }
    }
}

void runTest(NodeHandle &nh) {

    ROS_INFO("Waiting for initial state...");
    sensor_msgs::JointStateConstPtr state =  ros::topic::waitForMessage<sensor_msgs::JointState>("/simulation/right_arm/joint_control/get_state", ros::Duration(3.0));
    if(!state) {
        ROS_ERROR("Unable to retrieve initial joint state - shutting down...");
        exit(EXIT_FAILURE);
    }
    ROS_INFO("Initial state received.");

    ors::KinematicWorld G("kuka.kvg");
    arr traj = plan(state->position);
    displayTrajectory(traj, 1, G, "planned trajectory", 0.01);
    execute(nh, traj);
}

int main(int argc,char** argv) {
    MT::initCmdLine(argc,argv);

    ros::init(argc, argv, "iis_komo_test");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    NodeHandle nh;

    runTest(nh);

    spinner.stop();
    return EXIT_SUCCESS;
}


