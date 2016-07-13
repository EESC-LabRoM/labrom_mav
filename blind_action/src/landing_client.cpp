// blind_action libraries
#include "blind_action/landing_client.h"

namespace blind{
namespace landing{
/*
* Empty constructor
*/
LandingClient::LandingClient(void){
  SetGoal();
};

/*
* Empty destructor
*/
LandingClient::~LandingClient(void){};


/*
* Set Goal
* @param landing_accel landing acceleration [DEFAULT 10 m/s/s]
*/
void LandingClient::SetGoal(double landing_accel){
  // Update goal
  goal_.landing_accel = landing_accel;
  // Log
  ROS_INFO("Blind Landing CLIENT: Updating goal parameters. [landing_accel]=[%.2f]", goal_.landing_accel);
}

/*
* Send Goal
* This function sends the goal to landing blind action server.
* It returns true - succeed or false - failed
* @param timeout time for cancelling landing attempt  [DEFAULT = 2.0 s]
*/
bool LandingClient::SendGoal(double timeout){
  // Starting action client
  actionlib::SimpleActionClient<blind_action::LandingAction> ac("blindlanding", true);

  // Log
  ROS_INFO("Blind Landing CLIENT: Waiting foraction server to start.");

  // Wait for the action server to start
  ac.waitForServer(); 
  ROS_INFO("Blind Landing CLIENT: Action server started, sending goal. [landing_accel]=[%.2f]. Timeout = %.2f secs", goal_.landing_accel, timeout);

  // Send goal
  ac.sendGoalAndWait(goal_,ros::Duration(timeout),ros::Duration(0.1));
 
  // Check result
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Blind Landing CLIENT: Action finished - %s", ac.getState().toString().c_str());
  }else{
    ROS_INFO("Blind Landing CLIENT: Action did not finish before the time out." );
  }
  return 0;
}

} // landing namespace
} // blind namespace


