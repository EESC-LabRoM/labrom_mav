/*************************************************************************
*   Blind::TakeOffServer implementation
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
// take_off libraries
#include "blind_action/take_off_server.h"

// ROS message libraries
#include "std_msgs/Int32.h"
#include "geometry_msgs/Vector3.h"

namespace blind{
namespace take_off{
/*
* Constructor
* @param loop_rate actuation publishing frequency.
*/
TakeOffServer::TakeOffServer(controllers::Controller &controller) :as_(nh_,"blindtakeoff", boost::bind(&TakeOffServer::GoalCallback, this, _1), false){
  // Action server callbacks
//  as_.registerGoalCallback(boost::bind(&TakeOffServer::GoalCallback, this));   
  as_.registerPreemptCallback(boost::bind(&TakeOffServer::PreemptCallback, this));
  // ROS subscribers and publishers
  imu_sub_      = nh_.subscribe("/imu", 1, &TakeOffServer::ImuCallback, this);
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
  ROS_INFO("Blind Take Off SERVER: Action server ready!");
  
}

/*
* Empty destructor
*/
TakeOffServer::~TakeOffServer(void){};

/*
* Imu Callback.
* Based on the current z-axis acceleration and the goal acceleration checks if 
* take off has succeeded.
* @param imu ros message
*/
void TakeOffServer::ImuCallback(const sensor_msgs::Imu::ConstPtr &imu){
  // Make sure action is active
  if (!as_.isActive())
    return;
 
  // Action is ready to be executed
  if (!is_reading_imu_)
    is_reading_imu_ = true;
  
  // Check if take off succeed
  if( imu->linear_acceleration.z >= goal_.take_off_accel){
    // Success!
    result_.thrust       = feedback_.thrust;
    result_.elapsed_time = ros::Time::now().toSec() - start_time_;
    // Set the action state to succeeded
    as_.setSucceeded(result_);
    // Log
   
    ROS_INFO("Blind Take Off SERVER: Succeeded!");
  }

  // Update value
  feedback_.z_accel = imu->linear_acceleration.z;

}

/*
* Goal callback. 
* This function is called upon acceptance of a new goal.
* @param goal current goal specifying take off acceleration (m/s/s) 
*/
void TakeOffServer::GoalCallback(const blind_action::TakeOffGoalConstPtr &goal){
  // Take off starting time
  start_time_ = ros::Time::now().toSec();
  // Saving goal parameters
  //goal_.take_off_accel = as_.acceptNewGoal()->take_off_accel;
  goal_.take_off_accel = goal->take_off_accel;
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
  ROS_INFO("Blind Take Off SERVER: Trying to take off. [take_off_accel]=[%f m/s/s] ", goal_.take_off_accel);

  // ROS Loop
  while( ros::ok() && as_.isActive()){
    // Wait until imu is receiving messages
    if (!is_reading_imu_)
      continue;
    // Compute controller iteration
    double u =  controller_->LoopOnce(goal_.take_off_accel, feedback_.z_accel, dt);
    // Assemble message
    thrust_msg.data = std::min((int) (u + 0.5 + feedforward_) , max_thrust_);
    // Publish
    thrust_pub_.publish(thrust_msg); 
    attitude_pub_.publish(attitude_msg); 
    // Feedback
    feedback_.thrust  = thrust_msg.data; 
    as_.publishFeedback(feedback_);
 
    // Loop rate
    ros_rate.sleep();
  }
}

/*
* Preemption callback
* Modify action server to aborted
*/
void TakeOffServer::PreemptCallback(void){
  // Modify server status to aborted
  as_.setAborted();
  // Log
  ROS_INFO("Take Off Blind SERVER: Aborted");
}
 
/*
* Set actuation parameters.
* @param feedforward feedforward thrust value (0 to 100) [DEFAULT = 0] 
* @param max_thrust maximium thrust value (0 to 100) [DEFAULT = 10]
* @param loop_rate actuation loop rate [DEFAULT = 10 Hz]
*/
void TakeOffServer::SetParams(int feedforward, int max_thrust, int loop_rate){
  // Update values..
  feedforward_ = std::min(feedforward, 100);
  max_thrust_  = std::min(max_thrust,100);
  loop_rate_   = loop_rate;
  // Log        
  ROS_INFO("Blind  Take Off SERVER: Actuation parameters updated. [feedforward, max_thrust, loop_rate] := [%d, %d, %d]", feedforward_, max_thrust_, loop_rate_);

}

} //take_off namespace
} //blind namespace

