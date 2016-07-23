/*************************************************************************
*   blind::landing::Server implementation
*   This file is part of blind_action
*
*   labrom_mav_blind_action is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_mav_blind_action is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_mav_blind_action.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/
// landing libraries
#include "labrom_mav_blind_action/landing_server.h"

namespace blind{
namespace landing{
/*
* Constructor
*/
Server::Server(void) :as_(nh_,"blind_landing", boost::bind(&Server::GoalCallback, this, _1), false){
  // Action server callbacks 
  as_.registerPreemptCallback(boost::bind(&Server::PreemptCallback, this));

  // Setting parameter with default values
  this->SetParams();

  // Start action server
  as_.start();

  // Log
  ROS_INFO("Blind Landing SERVER: Action server ready!");
  
}

/*
* Empty destructor
*/
Server::~Server(void){};

/*
* Imu Callback.
* Based on the current z-axis acceleration and the goal acceleration checks if 
* landing has succeeded.
* @param imu ros message
*/
void Server::ImuCallback(const sensor_msgs::Imu::ConstPtr &imu){
  // Read z-axis acceleration
  z_accel_ = imu->linear_acceleration.z;  
}

/*
* Goal callback. 
* This function is called upon acceptance of a new goal.
* @param goal current goal specifying landing acceleration (m/s/s) 
*/
void Server::GoalCallback(const labrom_mav_blind_action::LandingGoalConstPtr &goal){
  // Saving goal parameters
  goal_.descend_accel = goal->descend_accel;
  goal_.hit_ground_accel = goal->hit_ground_accel;

  // ROS subscribers and publishers
  imu_sub_      = nh_.subscribe("/imu", 1, &Server::ImuCallback, this);

  // Ros sleep time
  ros::Rate ros_rate(loop_rate_);

  // Define and set inital values for take off attemp
  double dt = 1.0/loop_rate_;
  double decrement = 0;
  feedback_.thrust  = feedforward_;
  z_accel_ = 0;
  state_ = IDLE;

  // Log and ROS Loop 
  ROS_INFO("Blind Landing SERVER: Trying to landing. [descend_accel, hit_ground_accel]=[%.2f %.2f]m/s/s ", goal_.descend_accel, goal_.hit_ground_accel);
  while( ros::ok() && as_.isActive()){
    /* State table list
      IDLE: Although action has been called by a client, wait until imu callback has received a message.
      ACTIVE: Increment thrust value until a take off has been detected.
      DESCEND: Keep thrust until ground hit detection. Once detected, send thrus to 0 (minimal value).
      FINISHED: Set action succeed, shut down imu subscriber. Then, nothing to do.
      default: set thrust to 0 (should be safe!!!)
    */  
    switch (state_){
      case (IDLE):  
        // Wait for imu messages
        if (z_accel_ > 0)
          state_ = ACTIVE;
        break;

      case (ACTIVE):
        // Decrease thrust until descend accel detected
        if (z_accel_ > goal_.descend_accel){
          // Decrease thrust
          decrement -= 1*dt;
          feedback_.thrust  = std::min(feedback_.thrust + ((int) decrement), max_thrust_);
        } else{
          // Save take off thrust    
          result_.thrust = feedback_.thrust;
          state_ = DESCEND;
        }
        break; 
                             
      case(DESCEND): 
        // Remain in this state until ground hit detected
        if (z_accel_ > goal_.hit_ground_accel){
          feedback_.thrust = 0;
          state_ = FINISHED;
        }
        break;

      case (FINISHED): 
        // Shut down imu subscriber  
        imu_sub_.shutdown();
        // nothing to do..
        as_.setSucceeded(result_);
        break;

      default:
        // code should not enter here. just in case.. 
        feedback_.thrust = 0;
        ROS_WARN("Landing SERVER: Bug detected");
        break;
    }
    
    // Publish feedback  
    as_.publishFeedback(feedback_);

    // Loop rate
    ros_rate.sleep();
  }
}

/*
* Preemption callback
* Modify action server to aborted
*/
void Server::PreemptCallback(void){
  // Modify server status to aborted
  as_.setAborted();
  // Log
  ROS_INFO("Landing Blind SERVER: Aborted");
}
 
/*
* Set actuation parameters.
* @param feedforward feedforward thrust value (0 to 100) [DEFAULT = 0] 
* @param max_thrust maximium thrust value (0 to 100) [DEFAULT = 10]
* @param loop_rate actuation loop rate [DEFAULT = 10 Hz]
*/
void Server::SetParams(int feedforward, int max_thrust, int loop_rate){
  // Update values..
  feedforward_ = std::min(feedforward, 100);
  max_thrust_  = std::min(max_thrust,100);
  loop_rate_   = loop_rate;
  // Log        
  ROS_INFO("Blind Landing SERVER: Actuation parameters updated. [feedforward, max_thrust, loop_rate] := [%d, %d, %d]", feedforward_, max_thrust_, loop_rate_);

}

} //landing namespace
} //blind namespace

