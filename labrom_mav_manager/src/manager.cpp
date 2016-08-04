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

// labrom libraries
#include "labrom_mav_manager/manager.h"
// labrom_mav_blind_action libraries
#include "labrom_mav_blind_action/blind.h"

namespace manager{

/**
* Empty constructor
*/
Manager::Manager(void){
  // Adversite and subscribe topics
  attitude_pub_ = nh_.advertise<geometry_msgs::Vector3>("/cmd_attitude",1);
  thrust_pub_   = nh_.advertise<std_msgs::Int32>("/cmd_thrust",1);
  odom_sub_    = nh_.subscribe("/odometry",1,&Manager::OdomCallback,this);

};

/**
* Empty destructor
*/
Manager::~Manager(void){};

/**
*  Odometry callback   
* @param msg last odometry message received
*/
void Manager::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
  odom_ = *msg;
}

/**
* Manager state machine
*/
void Manager::Loop(void){
  // Action clients
  blind::take_off::Client blind_take_off("/blind/takeoff");
  blind::landing::Client blind_landing("/blind/landing");

  // Setting state machine parameters
  ros::Rate rate(20);
  manager::ManagerState state= manager::TAKE_OFF;

  // ROS messages
  std_msgs::Int32 thrust;
  geometry_msgs::Vector3 attitude;

  // Here comes the manager state machine (core)
  while(ros::ok()){

    // Nothing to do  
    switch (state){
      case (manager::IDLE):{
        //! @todo Switching condition (turn motors on?)
       thrust.data = 0;
        break;
      }

      case (manager::TURN_MOTORS_ON):{
        //! @todo turn motors on
        break;     
      }

      // Call take off server
      case (manager::TAKE_OFF):{
        // Assemble take off goal
        double take_off_accel;
        int  climb_time;
        nh_.param("take_off_accel", take_off_accel, 10.0);
        nh_.param("climb_time", climb_time, 0);
        // Send take off goal
        blind_take_off.SendGoal(take_off_accel, climb_time);
        // Log and change state 
        ROS_INFO("[Manager] Trying to take off.");
        state = manager::WAIT_TAKE_OFF;
        
        break;
      }

      // Blind take off supervisionary state
      case (manager::WAIT_TAKE_OFF):{
        thrust.data = blind_take_off.GetThrust();
        if (blind_take_off.IsDone()){
          // Change feedforward value for take off thrust value;
          nh_.setParam("feedforward", thrust.data);
          // Log and change state
          ROS_INFO("[Manager] Take off succeeded.");
          state = manager::FREE_MODE;
        }
        break;
      }

      // Receive order from extern machine
      case (manager::FREE_MODE):{
        //! @todo call user action server 
        // Check if disconnected from higher level controller 
        if(  (ros::Time::now() - odom_.header.stamp).toSec() > 1.0)
            state = manager::LAND;
        
        break;
      }
      
      // Call land server (blind)
      case (manager::LAND):{
        // Assemble landing goal
        double descend_accel, hit_ground_accel;
        nh_.param("descend_accel", descend_accel, 9.0);
        nh_.param("hit_ground_accel", hit_ground_accel, 11.5);
        // Send goal
        blind_landing.SendGoal(descend_accel, hit_ground_accel);     
        // Log and change state 
        ROS_INFO("[Manager] Trying to land.");
        state = manager::WAIT_LANDING;
        break;
      }

      // Landing supervisionary state
      case (manager::WAIT_LANDING):{
        thrust.data = blind_landing.GetThrust();
        if (blind_landing.IsDone()){
          // Send low motor speed, log and change state
          thrust.data = 0; 
          ROS_INFO("[Manager] Landing detected.");
          state = manager::TURN_MOTORS_OFF;
        }
        break;
      }

      case (manager::TURN_MOTORS_OFF):
        //! @todo turn motors off

        break;

      default:
        ROS_INFO("[unknown state]");

        break;
    
    } // switch

    // Publish thrust and attitude messages
    thrust_pub_.publish(thrust);
    // Loop delay
    rate.sleep();

  } // while
}
} // manager namespace

int main(int argc, char **argv){
  // Initialize ROS within this node 
  ros::init(argc,argv,"MAVManager");

  manager::Manager manager;      

  manager.Loop();
}