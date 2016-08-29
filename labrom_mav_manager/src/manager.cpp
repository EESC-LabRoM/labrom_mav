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

namespace manager{

/**
* Empty constructor
*/
Manager::Manager(void): nh_("~"){
  // Adversite and subscribe topics
  attitude_pub_ = nh_.advertise<geometry_msgs::Vector3>("cmd_attitude",1);
  thrust_pub_   = nh_.advertise<std_msgs::Int32>("cmd_thrust",1);

  imu_sub_      = nh_.subscribe("/imu",1,&Manager::ImuCallback,this);


};

/**
* Empty destructor
*/
Manager::~Manager(void){};

/**
* IMU callback   
* @param[in] msg last odometry message received
*/
void Manager::ImuCallback(const sensor_msgs::Imu::ConstPtr &msg){
  imu_ = *msg;
  // Save ~max upward acceleration
  if (imu_.linear_acceleration.z >  max_detected_accel_){
    max_detected_accel_ = imu_.linear_acceleration.z ;
  // save ~min upward acceleration
  }else if (imu_.linear_acceleration.z <  min_detected_accel_){
     min_detected_accel_ = imu_.linear_acceleration.z ;
  }
}


/**
* Manager state machine
*/
void Manager::Loop(void){
  // Setting state machine parameters
  ros::Rate rate(20);
  manager::ManagerState state= manager::IDLE;

  // ROS messages
  std_msgs::Int32 thrust;
  geometry_msgs::Vector3 attitude;

  // Here comes the manager state machine (core)
  while(ros::ok()){

    // Nothing to do  
    switch (state){
      case (manager::IDLE):
        //! @todo Switching condition (turn motors on?)
        thrust.data = 0;
        attitude.x = 0;
        attitude.y = 0;
        attitude.z = 0;
        break;
      
      // Getting ready to take off
      case (manager::TAKE_OFF):
        // Setting variables to take off detection
        max_detected_accel_ = 0;
        break;

      // Blind take off supervisionary state
      case (manager::WAIT_TAKE_OFF):{
        // Increase thrust
        thrust.data = thrust.data + 1;
        // Check if take off accel reached
        if (max_detected_accel_ > _take_off_accel){
          // Log and change state
          ROS_INFO("[Manager] Take off succeeded.");
          state = manager::CLIMB;
        }
        break;
      }

      // Getting ready to climb
      case (manager::CLIMB):{
        // Setting time to current time
        time_ = ros::Time::now().toSec();
        state = manager::CLIMB;
      }

      // Blind climb supervisor
      case (manager::WAIT_CLIMB):{
        if( (ros::Time::now().toSec() - time_) > _climb_time ){
          state = manager::HOVER;
        }
      }

      // Hover (velocity control)
      case (manager::HOVER):{

      }

      // Receive order from extern machine
      case (manager::FREE_MODE):{
        //! @todo call user action server 
        // Check if disconnected from higher level controller 
       // if(  (ros::Time::now() - odom_.header.stamp).toSec() > 1) //_free_mode_estimation_timeout
         //   state = manager::LAND;
        
        break;
      }
      
      // Call land server (blind)
      case (manager::LAND):{
        // Assemble landing goal

        state = manager::WAIT_LANDING;
        break;
      }

      // Landing supervisionary state
      case (manager::WAIT_LANDING):{
      /*  if (blind_landing.IsDone()){
          // Send low motor speed, log and change state
          thrust.data = 0; 
          ROS_INFO("[Manager] Landing detected.");
          state = manager::TURN_MOTORS_OFF;
        }*/
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
