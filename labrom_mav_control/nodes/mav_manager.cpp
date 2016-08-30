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
#include "labrom_mav_control/mav_manager.h"

namespace mav_control{
namespace manager{
/**
* Empty constructor
*/
Manager::Manager(void): nh_("~"){
  // Adversite and subscribe topics
  attitude_pub_ = nh_.advertise<geometry_msgs::Vector3>("cmd_attitude",1);
  thrust_pub_   = nh_.advertise<std_msgs::Float32>("cmd_thrust",1);

  imu_sub_      = nh_.subscribe("/imu",1,&Manager::ImuCallback,this);
  odom_sub_     = nh_.subscribe("/odometry",1,&Manager::OdometryCallback,this);
  traj_sub_     = nh_.subscribe("/trajectory",1,&Manager::TrajectoryCallback,this);  
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
     max_detected_accel_ = imu_.linear_acceleration.z;
  // save ~min upward acceleration
  }else if (imu_.linear_acceleration.z <  min_detected_accel_){
     min_detected_accel_ = imu_.linear_acceleration.z ;
  }
}

/**
* Update odometry
* param[in] Last odometry msg received
*/
void Manager::OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg){
    odom_ = *msg;
}

/**
* Update trajectories
* param[in] msg last trajectory received
*/
void Manager::TrajectoryCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg){
    traj_ = *msg;
}

/**
* Manager state machine
*/
void Manager::Spin(void){
  // ROS messages
  std_msgs::Float32 thrust;
  geometry_msgs::Vector3 attitude;
  trajectory_msgs::JointTrajectoryPoint local_traj;

  // Setting state machine parameters
  double mass, rate;
  double take_off_accel, land_accel, climb_time;

  nh_.param<double>("mass",mass, 0);
  nh_.param<double>("rate",rate, 20);
  ros::Rate loop_rate(rate);

  // Controllers
  mav_control::velocity::linear::Controller vel_controller(mass);

  // Here comes the manager state machine (core)
  ManagerState state = manager::TAKE_OFF;

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
        nh_.param("take_off_accel", take_off_accel,0.0);
        max_detected_accel_ = 0;
        state = manager::WAIT_TAKE_OFF;
        ROS_INFO("[Manager] Trying to take off..");
        break;

      // Blind take off supervisionary state
      case (manager::WAIT_TAKE_OFF):
        // Increase thrust
        thrust.data = thrust.data + 1*1/rate;
        // Check if take off accel reached
        if (max_detected_accel_ > take_off_accel){
          // Log and change state
          state = manager::CLIMB;
          ROS_INFO("[Manager] Take off succeed!");
        }
        break;
      

      // Getting ready to climb
      case (manager::CLIMB):
        // Setting time to current time
        nh_.param("climb_time", climb_time,1.0);
        time_ = ros::Time::now().toSec();
        state = manager::WAIT_CLIMB;
        ROS_INFO("[Manager] Climbing!");
        break;

      // Blind climb supervisor
      case (manager::WAIT_CLIMB):
        if( (ros::Time::now().toSec() - time_) > climb_time ){
          state = manager::LAND;
        }
        break;

      // Setting hover (velocity control)
      case (manager::HOVER):
        local_traj.velocities.clear();
        for(int i=0; i < 3; ++i)
            local_traj.velocities.push_back(0);
        state = manager::HOVERING;
        ROS_INFO("[Manager] Hovering!");
        break;
    
      // Hovering
      case (manager::HOVERING):
        vel_controller.LoopOnce(local_traj, odom_, thrust, attitude);
        break;

      // Receive order from extern machine
      case (manager::FREE_MODE):{
        break;
      }
      
      // Setting land parameters
      case (manager::LAND):{
        /// Descend with constant downward velocity
        local_traj.velocities.clear();
        for(int i=0; i < 3; ++i)
            local_traj.velocities.push_back(0);
        local_traj.velocities[2] = -0.1;
        /// Landing parameters
        nh_.param("land_accel", land_accel,0.0);
        min_detected_accel_ = 100;
        state = manager::WAIT_LANDING;
        ROS_INFO("[Manager] Trying to land!");
        break;
      }

      // Landing supervisionary state
      case (manager::WAIT_LANDING):{
        vel_controller.LoopOnce(local_traj, odom_, thrust, attitude);
        // Check if take off accel reached
        if (min_detected_accel_ < land_accel){
          // Log and change state
          state = manager::IDLE;
          ROS_INFO("[Manager] Land detected!");
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
    attitude_pub_.publish(attitude);
    // Loop delay
    ros::spinOnce();
    loop_rate.sleep();
    
  } // while
}

} // manager namespace
} // control namespace

int main(int argc, char **argv){
  // Initialize ROS within this node 
  ros::init(argc,argv,"MAVManager");

  mav_control::manager::Manager manager;      
  manager.Spin();

}