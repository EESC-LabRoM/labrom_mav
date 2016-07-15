/*************************************************************************
*   Blind Node implementation for testinf purpose
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

// landing libraries
#include "blind_action/landing_server.h"
#include "blind_action/landing_client.h"
#include "blind_action/blind.h"

// controller libriares
#include <labrom_control/pid_simple.h>

namespace blind{
void Spin(void){
	ros::spin();
}
}

int main(int argc, char **argv){
  // Initialize ROS within this node
  ros::init(argc,argv,"LandingBlindDebug");
  // Initialize spin thread
  boost::thread spin_thread(&blind::Spin);
  // Controller for landing action
  controllers::pid::Simple pid("Blind");
  double kp=0.2, ki=1, kd=10*0.02, windup_thresh=10;
  pid.SetParams(kp,ki,kd,windup_thresh);

  // Action server
  blind::landing::LandingServer server(pid);
  int feedforward=40, max_thrust=50, loop_rate=20;
  server.SetParams(feedforward, max_thrust, loop_rate);

  // Action client
  blind::landing::LandingClient client;
  double landing_accel = 9.6;
  client.SetGoal(landing_accel);

  // Perform landing attempt
  double timeout = 30;
  client.SendGoal(timeout);
  
  // Shutdown ros and spin thread
  ros::shutdown();
  spin_thread.join();

  return 0;
}
