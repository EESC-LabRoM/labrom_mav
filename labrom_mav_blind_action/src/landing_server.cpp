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
/**
* Constructor
*/
Server::Server(std::string name) :as_(nh_,name, boost::bind(&Server::GoalCallback, this, _1), false), nh_("/mav"){
  // Action server callbacks 
  as_.registerPreemptCallback(boost::bind(&Server::PreemptCallback, this));

  // Setting parameter with default values
  nh_.param<int>("feedforward",  feedforward_, 0);
  nh_.param<int>("max_thrust", max_thrust_, 0);
  nh_.param<int>("loop_rate", loop_rate_, 20);

  // Start action server
  as_.start();

  // Log
  ROS_INFO("Blind Landing SERVER: Action server ready!");
  
}

/**
* Empty destructor
*/
Server::~Server(void){};

/**
* Imu Callback.
* Based on the current z-axis acceleration and the goal acceleration checks if 
* landing has succeeded.
* @param imu ros message
*/
void Server::ImuCallback(const sensor_msgs::Imu::ConstPtr &imu){
  double time = ros::Time::now().toSec();

  /* State table list
    IDLE: Although action has been called by a client, wait until imu callback has received a message.
    DESCEND: Decrease thrust until detecting descend_accel.
    WAIT_HIT_GROUND: Keep thrust until ground hit detection. Once detected, send thrust to 0 (minimal value).
    FINISHED: Set action succeed, shut down imu subscriber. Then, nothing to do.
    default: set thrust to 0 (should be safe!!!)
  */  

  switch (state_){
    case (IDLE):  
      previous_time_ = time;
      state_ = DESCEND;
      break;
  
    case (DESCEND):
      // Check if landing acceleration has been detected 
      if ( imu->linear_acceleration.z >  goal_.descend_accel ){
        // No.. then decrease thrust
        feedback_.thrust  = std::min(feedback_.thrust - 1*(time - previous_time_), (double) max_thrust_);
        previous_time_ = time;
      // yes.. save current thust and time
      } else {  
        result_.thrust = feedback_.thrust;
        previous_time_ = time;
        state_ = WAIT_HIT_GROUND;
      }
      break;

    case(WAIT_HIT_GROUND): 
      // Remain in this state until ground hit detected
      if (imu->linear_acceleration.z > goal_.hit_ground_accel){
        ROS_INFO("Blind Landing SERVER: Landing detected! %f", result_.thrust);
        state_ = FINISHED;
      }
      break;

    case (FINISHED): 
      // Set success
      as_.setSucceeded(result_);
      break;

    default:
      // code should not enter here. just in case.. 
      feedback_.thrust =  feedforward_;
      ROS_WARN("Landing SERVER: Bug detected [unknown state]");
      break;
  }

}

/**
* Goal callback. 
* This function is called upon acceptance of a new goal.
* @param goal current goal specifying landing acceleration (m/s/s) 
*/
void Server::GoalCallback(const labrom_mav_blind_action::LandingGoalConstPtr &goal){
  // Saving goal parameters
  goal_.descend_accel = goal->descend_accel;
  goal_.hit_ground_accel = goal->hit_ground_accel;

  // Retrieving values from parameters server
  nh_.getParam("feedforward", feedforward_);
  nh_.getParam("max_thrust", max_thrust_);
  nh_.getParam("loop_rate", loop_rate_);

  // Ros sleep time
  ros::Rate ros_rate(loop_rate_);

  // Define and set inital values for landing
  feedback_.thrust  = feedforward_;
  state_ = IDLE;

  // Turn imu ROS subscriber on
  imu_sub_      = nh_.subscribe("/imu", 1, &Server::ImuCallback, this);

  // Log and ROS Loop 
  ROS_INFO("Blind Landing SERVER: Trying to landing. [descend_accel, hit_ground_accel]=[%.2f %.2f]m/s/s ", goal_.descend_accel, goal_.hit_ground_accel);
  while( ros::ok() && as_.isActive()){
    // Publish feedback  
    as_.publishFeedback(feedback_);
    // Loop rate
    ros_rate.sleep();
  }

  // Shut down imu subscriber  
  imu_sub_.shutdown();
}

/**
* Preemption callback
* Modify action server to aborted
*/
void Server::PreemptCallback(void){
  // Modify server status to aborted
  as_.setAborted();
  // Log
  ROS_INFO("Blind Landing SERVER: Aborted");
}
 
/**
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

int main(int argc, char **argv){
  // Initialize ROS within this node
  ros::init(argc,argv,"BlindLanding");

  // Call landing server
  blind::landing::Server server(ros::this_node::getName()); 

  // Spin
  ros::spin();
}