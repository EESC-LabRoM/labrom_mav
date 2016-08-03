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


void TakeOffFeedback(const labrom_mav_blind_action::TakeOffFeedback::ConstPtr& feedback);

int main(int argc, char **argv){
  // Initialize ROS within this node 
  ros::init(argc,argv,"MAVManager");
  ros::NodeHandle nh;

  // Publishers
  ros::Publisher attitude_pub = nh.advertise<geometry_msgs::Vector3>("/cmd_attitude",1);
  ros::Publisher thrust_pub   = nh.advertise<std_msgs::Int32>("/cmd_thrust",1);

  // ROS messages
  std_msgs::Int32 thrust;
  geometry_msgs::Vector3 attitude;

  // Action clients
  blind::take_off::Client blind_take_off("/blind/takeoff");
  blind::landing::Client blind_landing("/blind/landing");

  // @todo is ROS Spin thread necessary? REMEMBER TO CLOSE THE THREAD AT THE VERY END!!!!!
  // boost::thread spin_thread(&manager::Spin);
  // spin_thread.join(); 
  ros::Rate rate(20);

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
        //! @todo these goals parameters should be input from variables that users can modify (e.g. dyn_reconfigure)
        double take_off_accel = 10.5;
        int climb_time = 1;
        blind_take_off.SendGoal(take_off_accel, climb_time);
        state = manager::WAIT_TAKE_OFF;
        
        break;
      }

      // Blind take off supervisionary state
      case (manager::WAIT_TAKE_OFF):{
        thrust.data = blind_take_off.GetThrust();
        thrust_pub.publish(thrust);
        if (blind_take_off.IsDone())
          state = manager::FREE_MODE;
        break;
      }

      // Receive order from extern machine
      case (manager::FREE_MODE):{
        //! @todo call user action server  
        state = manager::LAND;
        break;
      }
      
      // Call land server (blind)
      case (manager::LAND):{
        // Assemble landing goal
        //! @todo these goals parameters should be input from variables that users can modify (e.g. dyn_reconfigure)
        double descend_accel = 9.7;
        double hit_ground_accel = 11.5;
        blind_landing.SendGoal(descend_accel, hit_ground_accel);
        state = manager::WAIT_LANDING;
        break;
      }

      // Landing supervisionary state
      case (manager::WAIT_LANDING):{
        thrust.data = blind_landing.GetThrust();
        thrust_pub.publish(thrust);
        if (blind_landing.IsDone())
          state = manager::TURN_MOTORS_OFF;
        break;
      }

      case (manager::TURN_MOTORS_OFF):
        //! @todo turn motors off

        break;

      default:
        ROS_INFO("[unknown state]");

        break;
    
    } // switch

    // Loop delay
    rate.sleep();
  } // while

  // Shutdown ros and spin thread
  ros::shutdown();
  
  return 0;
}

