/*************************************************************************
*   Blind::take_off::Server implementation
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
#include "labrom_mav_manager/take_off_server.h"

namespace blind{
namespace take_off{

/**
* Constructor
*/
Server::Server(std::string name) :as_(nh_,name, boost::bind(&Server::GoalCallback, this, _1), false), nh_("/mav"){
  // Action server callback
  as_.registerPreemptCallback(boost::bind(&Server::PreemptCallback, this));

  // Get parameters from launch file
  nh_.param("feedforward", feedforward_, 0);
  nh_.param("max_thrust", max_thrust_, 0);
  nh_.param("loop_rate", loop_rate_, 20);

  // Start action server
  as_.start();

  // Log
  ROS_INFO("Blind Take Off SERVER: %s action server ready!", name.c_str());
  
}

/**
* Empty destructor
*/
Server::~Server(void){};

/**
* Imu Callback.
* Based on the current z-axis acceleration and the goal acceleration checks if 
* take off has succeeded.
* @param imu ros message
*/
void Server::ImuCallback(const sensor_msgs::Imu::ConstPtr &imu){
  double time = ros::Time::now().toSec();
  /* State table list
    IDLE: Although action has been called by a client, wait until imu callback has received a message.
    TRYING_TO_TAKE_OFF: Increment thrust value until a take off has been detected.
    CLIMB: Climb for time interval defined in goal.
    FINISHED: Set action succeed. Then, nothing to do.
    default: set thrust to feedfoward value (should be safe!!!)
  */   
  switch (state_){
    case (IDLE):
      feedback_.thrust = feedforward_;
      previous_time_ = time;
      state_ = TRYING_TO_TAKE_OFF;
      break;
    
    case (TRYING_TO_TAKE_OFF):
      // Check if take off acceleration has been detected 
      if ( imu->linear_acceleration.z < goal_.take_off_accel){
        // No.. then increase thrust
        feedback_.thrust  = std::min(feedback_.thrust + 2*(time - previous_time_), (double) max_thrust_);
        previous_time_ = time;
        
      // yes.. save current thust and time
      } else {  
        result_.thrust = feedback_.thrust;
        ROS_INFO("Blind Take Off SERVER: Take off detected! %f", result_.thrust);
        previous_time_ = time;
        state_ = CLIMB;
      }
      break;
  
    case (CLIMB): 
      // Remain in this state until climb_time reached 
      if ( (ros::Time::now().toSec() - previous_time_) >= goal_.climb_time) {
        state_ = FINISHED;
      }
      break;
    
    case (FINISHED):
      as_.setSucceeded(result_);
      break;

    default:
     // code should not enter here. just in case.. 
     feedback_.thrust = feedforward_;
     ROS_WARN("Take off SERVER: Bug detected [unknown state]");
     break;    
  }

}

/**
* Goal callback. 
* This function is called upon acceptance of a new goal.
* @param goal current goal specifying take off acceleration (m/s/s) 
*/
void Server::GoalCallback(const labrom_mav_manager::TakeOffGoalConstPtr &goal){
  // Saving goal parameters
  goal_.take_off_accel = goal->take_off_accel;
  goal_.climb_time     = goal->climb_time;

  // Retrieving values from parameters server
  nh_.getParam("feedforward", feedforward_);
  nh_.getParam("max_thrust", max_thrust_);
  nh_.getParam("loop_rate", loop_rate_);

  // Ros sleep time
  ros::Rate ros_rate(loop_rate_);

  // Define and set inital values for take off attemp
  feedback_.thrust  = 0;
  state_ = IDLE;

  // Turn imu ROS subscriber on
  imu_sub_  = nh_.subscribe("/imu", 1, &Server::ImuCallback, this);

  // Log and ROS Loop 
  ROS_INFO("Blind Take Off SERVER: Trying to take off. [take_off_accel, climb_time]:=[%.2f, %d]", goal_.take_off_accel, goal_.climb_time);
  while( ros::ok() && as_.isActive()){
    // Publish feedback  
    as_.publishFeedback(feedback_);
    // Loop rate
    ros_rate.sleep();
  }

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
  ROS_INFO("Take Off Blind SERVER: Aborted");
}
 
/**
* Set actuation parameters.
* @param feedforward feedforward thrust value (0 to 100) [DEFAULT = 0] 
* @param max_thrust maximium thrust value (0 to 100) [DEFAULT = 10]
* @param loop_rate actuation loop rate [DEFAULT = 10 Hz]
*/
void Server::SetParams(int feedforward, int max_thrust, int loop_rate){
  // Update values..
  feedforward_ = feedforward;
  max_thrust_  = max_thrust;
  loop_rate_   = loop_rate;
  // Log        
  ROS_INFO("Blind  Take Off SERVER: Actuation parameters updated. [feedforward, max_thrust, loop_rate] := [%d, %d, %d]", feedforward_, max_thrust_, loop_rate_);

}

} //take_off namespace
} //blind namespace

int main(int argc, char **argv){
  // Initialize ROS within this node
  ros::init(argc,argv,"BlindTakeOff");

  // Call take off server
  blind::take_off::Server server(ros::this_node::getName()); 

  // Spin
  ros::spin();
}