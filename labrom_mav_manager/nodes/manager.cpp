/*************************************************************************
*   Manager implementation 
*   This file is part of labrom_mav_manager
*
*   labrom_mav_manager is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_mav_manager is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_mav_manager.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

// take_off libraries
#include "blind_action/take_off_server.h"
#include "blind_action/take_off_client.h"
#include "blind_action/blind.h"

// controller libriares
#include "labrom_control/pid_simple.h"

int main(int argc, char **argv){
  // Initialize ROS within this node  ros::init(argc,argv,"TakeOffBlindDebug");
  // Initialize spin thread
  boost::thread spin_thread(&blind::Spin);
  // Controller for take off action
  controllers::pid::Simple pid("Blind"); 
  double kp=0.2, ki=1, kd=10*0.02, windup_thresh=1;
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
  
  // Shutdown ros and spin thread
  ros::shutdown();
  spin_thread.join();

  return 0;
}