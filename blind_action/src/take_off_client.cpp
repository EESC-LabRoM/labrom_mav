/*************************************************************************
*   Blind::Client implementation
*   This file is part of blind_action
*
*   blind_action is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   blind_action is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with blind_action.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

// blind_action libraries
#include "blind_action/take_off_client.h"

namespace blind{
namespace take_off{
/**
* Empty constructor
*/
Client::Client(void){
  SetGoal();
};

/**
* Empty destructor
*/
Client::~Client(void){};


/**
* Set Goal
* @param take_off_accel take off acceleration [DEFAULT 10 m/s/s]
*/
void Client::SetGoal(double take_off_accel, double climb_time){
  // Update goal
  goal_.take_off_accel = take_off_accel;
  goal_.climb_time     = climb_time;
  // Log
  ROS_INFO("Blind Take off CLIENT: Updating goal parameters. [take_off_accel, climb_time]=[%.2f, %.2f]", 
              goal_.take_off_accel,
              goal_.climb_time);
}

/**
* Send Goal
* This function sends the goal to take off blind action server.
* It returns true - succeed or false - failed
* @param timeout time for cancelling take off attempt  [DEFAULT = 2.0 s]
*/
bool Client::SendGoal(double timeout){
  // Starting action client
  actionlib::SimpleActionClient<blind_action::TakeOffAction> ac("blindtakeoff", true);

  // Log
  ROS_INFO("Blind Take Off CLIENT: Waiting foraction server to start.");

  // Wait for the action server to start
  ac.waitForServer(); 
  ROS_INFO("Blind Take Off CLIENT: Action server ready, sending goal. [take_off_accel]=[%.2f]. Timeout = %.2f secs", goal_.take_off_accel, timeout);

  // Send goal
  ac.sendGoalAndWait(goal_,ros::Duration(timeout),ros::Duration(0.1));
 
  // Check result
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Blind Take Off CLIENT: Action finished - %s", ac.getState().toString().c_str());
    result_.thrust       = ac.getResult()->thrust;
    return true;
  }else{
    ROS_INFO("Blind Take Off CLIENT: Action did not finish before the time out." );
    return false;
  }
  
}

/**
* Get result thrust
* This function returns the thrust achieved in the last successful take off
*/
int Client::getResultThrust(void){
  return result_.thrust;
}

} // take_off namespace
} // blind namespace


