/*************************************************************************
*   Blind::ClimbServer implementation
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
#include "blind_action/climb_server.h"

// ROS message libraries
#include "std_msgs/Int32.h"
#include "geometry_msgs/Vector3.h"

namespace blind{
namespace climb{
/*
* Constructor
* @param loop_rate actuation publishing frequency.
*/
ClimbServer::ClimbServer(controllers::Controller &controller) :as_(nh_,"blindclimb", boost::bind(&ClimbServer::GoalCallback, this, _1), false){
  // Action server callbacks
//  as_.registerGoalCallback(boost::bind(&ClimbServer::GoalCallback, this));   
  as_.registerPreemptCallback(boost::bind(&ClimbServer::PreemptCallback, this));
  // ROS subscribers and publishers
  imu_sub_      = nh_.subscribe("/imu", 1, &ClimbServer::ImuCallback, this);
  thrust_pub_   = nh_.advertise<std_msgs::Int32>("/cmd_thrust",1);
  attitude_pub_ = nh_.advertise<geometry_msgs::Vector3>("/cmd_attitude",1);

  // Copying controller
  controller_ = &controller;

  // Setting parameter with default values
  this->SetParams();
  is_reading_imu_ = false;
  // Start action server
  as_.start();

  // Log
  ROS_INFO("Blind Climb SERVER: Action server ready!");
  
}

/*
* Empty destructor
*/
ClimbServer::~ClimbServer(void){};

/*
* Imu Callback.
* Update z-accel measurments
* @param imu ros message
*/
void ClimbServer::ImuCallback(const sensor_msgs::Imu::ConstPtr &imu){
  // Make sure action is active
  if (!as_.isActive())
    return;
 
  // Action is ready to be executed
  if (!is_reading_imu_)
    is_reading_imu_ = true;
  
  // Update z-axis acceleration value
  feedback_.z_accel = imu->linear_acceleration.z;

}

/*
* Goal callback. 
* This function is called upon acceptance of a new goal.
* @param goal current goal specifying take off acceleration (m/s/s) 
*/
void ClimbServer::GoalCallback(const blind_action::ClimbGoalConstPtr &goal){
  // Take off starting time
  start_time_ = ros::Time::now().toSec();
  // Saving goal parameters
  //goal_.climbaccel = as_.acceptNewGoal()->climbaccel;
  goal_.climb_accel = goal->climb_accel;
  goal_.timeout = goal->timeout;
  // Ros sleep time
  ros::Rate ros_rate(loop_rate_);

  // Messages
  std_msgs::Int32 thrust_msg;
  geometry_msgs::Vector3 attitude_msg;
  // Define initial values for messages
  thrust_msg.data  = 0;
  attitude_msg.x   = 0;
  attitude_msg.y   = 0;
  attitude_msg.z   = 0;
  // Sampling time
  double dt = 1.0/loop_rate_;
  // Log
  ROS_INFO("Blind Take Off SERVER: Climb actiove active. [climb_accel]=[%f m/s/s] ", goal_.climb_accel);

  // ROS Loop
  while( ros::ok() && as_.isActive()){   
    // Wait until imu is receiving messages
    if (!is_reading_imu_)
      continue;
    
    // Compute controller iteration
    double u =  controller_->LoopOnce(goal_.climb_accel, feedback_.z_accel, dt);
    // Assemble message
    thrust_msg.data = std::min((int) (u + 0.5 + feedforward_) , max_thrust_);
    // Publish
    thrust_pub_.publish(thrust_msg); 
    attitude_pub_.publish(attitude_msg); 
    // Feedback
    feedback_.thrust  = thrust_msg.data; 
    feedback_.elapsed_time = ros::Time::now().toSec() - start_time_;

    // Check if climb time reached
    if (feedback_.elapsed_time >= goal_.timeout ){
      // Climb time reached!
      result_.thrust  = feedback_.thrust;
      result_.z_accel = feedback_.z_accel;
      // Set the action state to succeeded
      as_.setSucceeded(result_);
      // Log
      ROS_INFO("Blind Climb SERVER: Succeeded!");
    }

    // Publish feedback and wait until next loop
    as_.publishFeedback(feedback_);
    ros_rate.sleep();
  }
}

/*
* Preemption callback
* Modify action server to aborted
*/
void ClimbServer::PreemptCallback(void){
  // Modify server status to aborted
  as_.setAborted();
  // Log
  ROS_INFO("Climb Blind SERVER: Aborted");
}
 
/*  
* Set actuation parameters.
* @param feedforward feedforward thrust value (0 to 100) [DEFAULT = 0] 
* @param max_thrust maximium thrust value (0 to 100) [DEFAULT = 10]
* @param loop_rate actuation loop rate [DEFAULT = 10 Hz]
*/
void ClimbServer::SetParams(int feedforward, int max_thrust, int loop_rate){
  // Update values..
  feedforward_ = std::min(feedforward, 100);
  max_thrust_  = std::min(max_thrust,100);
  loop_rate_   = loop_rate;
  // Log        
  ROS_INFO("Blind  Take Off SERVER: Actuation parameters updated. [feedforward, max_thrust, loop_rate] := [%d, %d, %d]", feedforward_, max_thrust_, loop_rate_);

}

} //climb namespace
} //blind namespace

