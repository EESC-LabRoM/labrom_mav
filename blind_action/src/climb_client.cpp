/*************************************************************************
*   Blind::Climb implementation
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
#include "blind_action/climb_client.h"

namespace blind{
namespace climb{
/*
* Empty constructor
*/
ClimbClient::ClimbClient(void){
  SetGoal();
};

/*
* Empty destructor
*/
ClimbClient::~ClimbClient(void){};


/*
* Set Goal
* @param climb_accel climb acceleration [DEFAULT 10 m/s/s]
* @param timeout climb action duration in seconds [DEFAULT 1 sec]
*/
void ClimbClient::SetGoal(double climb_accel, double timeout){
  // Update goal
  goal_.climb_accel = climb_accel;
  goal_.timeout = timeout;
  // Log
  ROS_INFO("Blind Climb CLIENT: Updating goal parameters. [climb_accel]=[%.2f]", goal_.climb_accel);
}

/*
* Send Goal
* This function sends the goal to climb blind action server.
* It returns true - succeed or false - failed
* @param timeout time for cancelling climb attempt  [DEFAULT = 2.0 s]
*/
bool ClimbClient::SendGoal(double timeout){
  // Starting action client
  actionlib::SimpleActionClient<blind_action::ClimbAction> ac("blindclimb", true);

  // Log
  ROS_INFO("Blind Climb CLIENT: Waiting foraction server to start.");

  // Wait for the action server to start
  ac.waitForServer(); 
  ROS_INFO("Blind Climb CLIENT: Action server started, sending goal. [climb_accel]=[%.2f]. Timeout = %.2f secs", goal_.climb_accel, timeout);

  // Send goal
  ac.sendGoalAndWait(goal_,ros::Duration(timeout),ros::Duration(0.1));
 
  // Check result
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Blind Climb CLIENT: Action finished - %s", ac.getState().toString().c_str());
  }else{
    ROS_INFO("Blind Climb CLIENT: Action did not finish before the time out." );
  }
  return 0;
}

} // climb namespace
} // blind namespace


