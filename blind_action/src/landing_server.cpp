// landing libraries
#include "blind_action/landing_server.h"

// ROS message libraries
#include "std_msgs/Int32.h"
#include "geometry_msgs/Vector3.h"

namespace blind{
namespace landing{
/*
* Constructor
* @param loop_rate actuation publishing frequency.
*/
LandingServer::LandingServer(controllers::Controller &controller) :as_(nh_,"blindlanding", boost::bind(&LandingServer::GoalCallback, this, _1), false){
  // Action server callbacks
//  as_.registerGoalCallback(boost::bind(&LandingServer::GoalCallback, this));   
  as_.registerPreemptCallback(boost::bind(&LandingServer::PreemptCallback, this));
  // ROS subscribers and publishers
  imu_sub_      = nh_.subscribe("/fcu/imu", 1, &LandingServer::ImuCallback, this);
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
  ROS_INFO("Blind Landing SERVER: Action server ready!");
  
}

/*
* Empty destructor
*/
LandingServer::~LandingServer(void){};

/*
* Imu Callback.
* Based on the current z-axis acceleration and the goal acceleration checks if 
* landing has succeeded.
* @param imu ros message
*/
void LandingServer::ImuCallback(const sensor_msgs::Imu::ConstPtr &imu){
  // Make sure action is active
  if (!as_.isActive())
    return;
 
  // Action is ready to be executed
  if (!is_reading_imu_)
    is_reading_imu_ = true;

  // Check if landing succeed
  if( imu->linear_acceleration.z >= goal_.hit_ground_accel ){
    // Success!
    result_.thrust       = feedback_.thrust;
    result_.elapsed_time = ros::Time::now().toSec() - start_time_;
    // Set the action state to succeeded
    as_.setSucceeded(result_);
    // Log
   
    ROS_INFO("Blind Landing SERVER: Succeeded!");
  }

  // Update value
  feedback_.z_accel = imu->linear_acceleration.z;

}

/*
* Goal callback. 
* This function is called upon acceptance of a new goal.
* @param goal current goal specifying landing acceleration (m/s/s) 
*/
void LandingServer::GoalCallback(const blind_action::LandingGoalConstPtr &goal){
  // Landing starting time
  start_time_ = ros::Time::now().toSec();
  // Saving goal parameters
  //goal_.landing_accel = as_.acceptNewGoal()->landing_accel;
  goal_.landing_accel = goal->landing_accel;
  goal_.hit_ground_accel = goal->hit_ground_accel;
  // Ros sleep time
  ros::Rate ros_rate(loop_rate_);

  // Messages
  std_msgs::Int32 thrust_msg;
  geometry_msgs::Vector3 attitude_msg;
  // Define initial values for messages
  thrust_msg.data  = std::min(feedforward_ , max_thrust_);
  attitude_msg.x   = 0;
  attitude_msg.y   = 0;
  attitude_msg.z   = 0;
  // Sampling time
  double dt = 1.0/loop_rate_;
  // Log
  ROS_INFO("Blind Landing SERVER: Trying to landing. [landing_accel]=[%f m/s/s] ", goal_.landing_accel);

  // ROS Loop
  while( ros::ok() && as_.isActive()){
    // Wait until imu is receiving messages
    if (!is_reading_imu_)
      continue;

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
void LandingServer::PreemptCallback(void){
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
void LandingServer::SetParams(int feedforward, int max_thrust, int loop_rate){
  // Update values..
  feedforward_ = std::min(feedforward, 100);
  max_thrust_  = std::min(max_thrust,100);
  loop_rate_   = loop_rate;
  // Log        
  ROS_INFO("Blind Landing SERVER: Actuation parameters updated. [feedforward, max_thrust, loop_rate] := [%d, %d, %d]", feedforward_, max_thrust_, loop_rate_);

}

} //landing namespace
} //blind namespace

