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

/**
* Constructor
* @param loop_rate actuation publishing frequency.
*/
Server::Server(void) :as_(nh_,"blindtakeoff", boost::bind(&Server::GoalCallback, this, _1), false){
  // Action server callback
  as_.registerPreemptCallback(boost::bind(&Server::PreemptCallback, this));

  // Setting parameters with default values
  this->SetParams();

  // Start action server
  as_.start();

  // Log
  ROS_INFO("Blind Take Off SERVER: Action server ready!");
  
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
  // Make sure action is active
  if (!as_.isActive())
    return;
  
  if (state_ == IDLE)
    state_ = ACTIVE;
    
  // Check if take off succeed
  if( imu->linear_acceleration.z >= goal_.take_off_accel){
    // Update action state_
    state_ = TAKE_OFF_DETECTED;
    // Log
    ROS_INFO("Blind Take Off SERVER: Take off detected! %d", result_.thrust);
  }

}

/**
* Goal callback. 
* This function is called upon acceptance of a new goal.
* @param goal current goal specifying take off acceleration (m/s/s) 
*/
void Server::GoalCallback(const blind_action::TakeOffGoalConstPtr &goal){
  // Saving goal parameters
  goal_.take_off_accel = goal->take_off_accel;
  goal_.climb_time     = goal->climb_time;

  // ROS subscribers and publishers
  imu_sub_      = nh_.subscribe("/fcu/imu", 1, &Server::ImuCallback, this);

  // Ros sleep time
  ros::Rate ros_rate(loop_rate_);

  // Define and set inital values for take off attemp
  double time;
  double dt = 1.0/loop_rate_;
  double increment = 0;
  feedback_.thrust  = feedforward_;
  state_ = IDLE;

  // Log and ROS Loop 
  ROS_INFO("Blind Take Off SERVER: Trying to take off. [take_off_accel]=[%f m/s/s] ", goal_.take_off_accel);
  while( ros::ok() && as_.isActive()){
    /* State table list
      IDLE: Although action has been called by a client, wait until imu callback has received a message.
      ACTIVE: Increment thrust value until a take off has been detected.
      TAKE_OFF_DETECTED: Transition state. Save take off thrust, time take off is detected.
      CLIMB: Climb for time interval defined in goal.
      FINISHED: Set action succeed. Then, nothing to do.
      default: set thrust to feedfoward value (should be safe!!!)
    */  
    switch (state_){
      case (IDLE):  
        // Wait for imu messages
        break;

      case (ACTIVE):  
        // Increase thrust
        increment += 1*dt;
        feedback_.thrust  = std::min(feedback_.thrust + ((int) increment), max_thrust_);
        break; 

      case (TAKE_OFF_DETECTED): 
        // Save take off thrust
        result_.thrust = feedback_.thrust;
        // Shut down imu subscriber
        imu_sub_.shutdown(); 
        // Save time and change state
        time = ros::Time::now().toSec();
        state_ = CLIMB;
        break;
                              
      case(CLIMB): 
        // Remain in this state until climb_time reached 
        if ( (ros::Time::now().toSec() - time) >= goal_.climb_time) {
          state_ = FINISHED;
        }
        break;

      case (FINISHED): 
        // nothing to do..
        as_.setSucceeded(result_);
        break;

      default:
        // code should not enter here. just in case.. 
        feedback_.thrust = feedforward_;
        ROS_WARN("Take off SERVER: Bug detected");
        break;
    }
    
    // Publish feedback  
    as_.publishFeedback(feedback_);

    // Loop rate
    ros_rate.sleep();
  }
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
  feedforward_ = std::min(feedforward, 100);
  max_thrust_  = std::min(max_thrust,100);
  loop_rate_   = loop_rate;
  // Log        
  ROS_INFO("Blind  Take Off SERVER: Actuation parameters updated. [feedforward, max_thrust, loop_rate] := [%d, %d, %d]", feedforward_, max_thrust_, loop_rate_);

}

} //take_off namespace
} //blind namespace

