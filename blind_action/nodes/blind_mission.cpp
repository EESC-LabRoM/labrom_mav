// take_off libraries
#include "blind_action/take_off_server.h"
#include "blind_action/take_off_client.h"
#include "blind_action/blind.h"

// controller libriares
#include <labrom_control/pid_simple.h>

int main(int argc, char **argv){
  // Initialize ROS within this node
  ros::init(argc,argv,"nodeBlindMission");
  ros::NodeHandle nodeBlindMission;
  ros::Rate ros_loop_rate(60);
  
  // Controller for take off action
  controllers::pid::Simple pid("Blind");
  double kp=0.2, ki=1, kd=10*0.02, windup_thresh=10;
  pid.SetParams(kp,ki,kd,windup_thresh);

  // Action server
  blind::take_off::TakeOffServer server(pid);
  int feedforward=40, max_thrust=50, loop_rate=20;
  server.SetParams(feedforward, max_thrust, loop_rate);

  // Action client
  blind::take_off::TakeOffClient client;
  double take_off_accel = 10.0;
  client.SetGoal(take_off_accel);

  // Perform take off attempt
  double timeout = 30;
  client.SendGoal(timeout);
  
  while(ros::ok()) {
    ros::spinOnce();
    ros_loop_rate.sleep();
  }

  return 0;
}
