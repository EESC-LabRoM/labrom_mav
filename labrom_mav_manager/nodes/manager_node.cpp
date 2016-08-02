/*************************************************************************
*   Manager Node implementation 
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

// labrom_mav_manager libraries
#include "labrom_mav_manager/manager.h"
// labrom_mav_blind_action libraries
#include "labrom_mav_blind_action/blind.h"

// ROS message libraries
#include "std_msgs/Int32.h"
#include "geometry_msgs/Vector3.h"

// ROS messages
std_msgs::Int32 thrust;
geometry_msgs::Vector3 attitude;

void TakeOffFeedback(const labrom_mav_blind_action::TakeOffFeedback::ConstPtr& feedback);

int main(int argc, char **argv){
  // Initialize ROS within this node 
  ros::init(argc,argv,"MAVManager");
  ros::NodeHandle nh;

  // Publishers
  ros::Publisher attitude_pub = nh.advertise<geometry_msgs::Vector3>("/cmd_attitude",1);
  ros::Publisher thrust_pub   = nh.advertise<std_msgs::Int32>("/cmd_thrust",1);

  blind::take_off::Client takeoff("/blind/takeoff");

  // @todo is ROS Spin thread necessary? REMEMBER TO CLOSE THE THREAD AT THE VERY END!!!!!
  boost::thread spin_thread(&manager::Spin);

  // Action clients
  actionlib::SimpleActionClient<labrom_mav_blind_action::TakeOffAction> ac("/blind/takeoff", true);
  ac.waitForServer();

  // Manager state machine
  manager::ManagerState state= manager::TAKE_OFF;

  while(ros::ok()){
    // Spin thread
    switch (state){
      case (manager::IDLE):{
        //! @todo Switching condition (turn motors on?)
       
        break;
      }

      case (manager::TURN_MOTORS_ON):{
        //! @todo turn motors on

        break;     
      }

      // Call take off server
      case (manager::TAKE_OFF):{
        // Assemble blind take off goal
        takeoff.SendGoal(10.5,1);
        /*labrom_mav_blind_action::TakeOffGoal goal;
        goal.take_off_accel = 10.5;
        goal.climb_time = 1;
        // Send goal
        ac.sendGoal(goal, NULL, NULL, &TakeOffFeedback);
        */
        // Change for supervisionary state
        state = manager::WAIT_TAKE_OFF;
        
        break;
      }

      // Blind take off supervisionary state
      case (manager::WAIT_TAKE_OFF):{
        thrust.data = 0;
        std::cout << "Waiting take off: " << takeoff.GetThrust() << std::endl;
        break;
      }

      case (manager::FREE_MODE):
        //! @todo call user action server  
        
        break;
      case (manager::LAND):
        //! @todo call land server

        break;
      case (manager::TURN_MOTORS_OFF):
        //! @todo turn motors off

        break;

      default:
        ROS_INFO("[unknown state]");

        break;
    
    } // switch

  }
   spin_thread.join(); 

  // Shutdown ros and spin thread
  ros::shutdown();
  
  return 0;
}

// Called every time feedback is received for the goal
void TakeOffFeedback(const labrom_mav_blind_action::TakeOffFeedback::ConstPtr& feedback)
{
  thrust.data = (int) feedback->thrust;
  std::cout << "thrust: " << thrust.data << std::endl;
}
