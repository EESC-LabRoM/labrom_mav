/*************************************************************************
*   Linear plant associate with a PID control law for pose control (Implemenation)
*   This file is part of labrom_mav_control
*
*   labrom_mav_control is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_mav_control is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_mav_control.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

// labrom_mav_control libraries
#include <labrom_mav_control/pose_linear.h>
#include <vector>

namespace mav_control{
namespace pose{
namespace linear{
/**
* Empty constructor
*/
PID::PID(void): pnh_("~"){
  // Adversite and subscribe topics
  attitude_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("cmd_attitude",1);
  thrust_pub_   = nh_.advertise<std_msgs::Float32>("cmd_thrust",1);

  odom_sub_     = nh_.subscribe("odometry",1,&PID::OdometryCallback,this);
  pose_sub_     = nh_.subscribe("pose",1,&PID::PoseCallback,this);
  goal_sub_     = nh_.subscribe("goal",1,&PID::GoalCallback,this);  

  // Controllers parameters from configuration file
  std::vector<double> params;
  std::string coordinate[] = {"/x","/y","/z"}, gains[]={"/kp","/ki","/kd","/anti_windup"};
  for(int i=0;i<3;++i){
    for(int j=0;j<4;++j){
      params.push_back(0);
      pnh_.getParam("pose"+coordinate[i]+gains[j], params[4*i+j]);
    }
  }

  pnh_.param<std::string>("world_frame", world_frame_, "/world");
  pnh_.param<std::string>("body_frame" , body_frame_, "/body_link");
  pnh_.param<int>("loop_rate", loop_rate_, 20);
  pnh_.param<bool>("use_tf", use_tf_, false);

  // Loading PID controllers with configuration
  pid_ddx_ = controllers::pid::Simple("PID_ddx", params[0], params[1], params[2],  params[3]);
  pid_ddy_ = controllers::pid::Simple("PID_ddy", params[4], params[5], params[6],  params[7]);
  pid_ddz_ = controllers::pid::Simple("PID_ddz", params[8], params[9], params[10], params[11]);

  // Retrieving vehicle parameters
  pnh_.getParam("mass", vehicle_.mass);

  // Miscellaneous
  has_goal_ = false;
};

/**
* Empty destructor
*/
PID::~PID(void){};

/** Goal callback. Receives desired pose.
* @param[in] msg last goal pose message receive
*/
void PID::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
  desired_pose_ = msg->pose;
  if(!has_goal_)
    has_goal_= true;
}

/**
* Odometry message callback
* @param[in] odom current estimated odometry
*/
void PID::OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg){
  geometry_msgs::PoseStamped pose;
  geometry_msgs::TwistStamped twist;
  // Copying values
  pose.pose   = msg->pose.pose;
  pose.header = msg->header;
  
  twist.twist = msg->twist.twist;
  twist.header = msg->header;
  twist.header.frame_id = msg->child_frame_id;
  // Call method that actually computes/publishes control signals
  ComputeActuation(pose, twist);
}

/**
* TFCallback. Listen for TF to compute actuation inputs [DOES NOT WORK PROPERLY]
*/
void PID::TFCallback(void){
  try{
    geometry_msgs::PoseStamped pose;
    
    // Pose
    tf::StampedTransform transform;
    listener_.lookupTransform(	world_frame_,
                              	body_frame_,
                                ros::Time(0),
                                transform); 
    tf::poseTFToMsg(tf::Pose(transform), pose.pose);

    // Twist
    geometry_msgs::TwistStamped twist; 
    listener_.lookupTwist(body_frame_, 
                          world_frame_,  
                          ros::Time(0), 
                          ros::Duration(0.001),
                          twist.twist );

    // Call method that actually computes/publishes control signals.
    ComputeActuation(pose, twist);

  }catch (tf::TransformException ex){
    ROS_ERROR("[Pose Controller]%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

/**
* Odometry message callback
* @param[in] odom current estimated odometry
*/
void PID::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
  geometry_msgs::PoseStamped pose;
  geometry_msgs::TwistStamped twist;
  // Copying values
  pose.pose   = msg->pose;
  pose.header = msg->header;
  
  ComputeActuation(pose, twist);
}
  
/**
* Compute vehicle actuation input (collective thrust and attitude angles)
* @param[in] pose contains the current vehicle pose
* @param[in] twist contains the current vehicle twist
*/
void PID::ComputeActuation(const geometry_msgs::PoseStamped &pose,const geometry_msgs::TwistStamped &twist){
  if (has_goal_ == false){
    ROS_WARN("[PID Pose Controller] No goal has been set. Setting goal as current pose!");
    desired_pose_ = pose.pose;
    has_goal_ = true;
  }

  // Command accelerations 
  double ddx_c = pid_ddx_.LoopOnce(desired_pose_.position.x, pose.pose.position.x);
  double ddy_c = pid_ddy_.LoopOnce(desired_pose_.position.y, pose.pose.position.y);
  double ddz_c = pid_ddz_.LoopOnce(desired_pose_.position.z, pose.pose.position.z);

  // yaw controller
  double yaw = tf::getYaw(pose.pose.orientation);
  double yaw_error = (tf::getYaw (desired_pose_.orientation) - yaw);
  if (yaw_error > M_PI)
    yaw_error -= 2*M_PI;
  else if(yaw_error < -M_PI)
    yaw_error += 2*M_PI;

  double yaw_rate = 0.5*yaw_error;
  // Quadrotor input commands
  double T_d     = (9.81 - ddz_c)*vehicle_.mass;
  double pitch_d =  - 1/9.81  * (  ddx_c*cos(yaw) + ddy_c*sin(yaw) );
  double roll_d  =    1/9.81  * ( -ddx_c*sin(yaw) + ddy_c*cos(yaw) );

  // Assemble command message                 
  std_msgs::Float32 thrust;
  geometry_msgs::Vector3Stamped attitude;

  thrust.data = T_d;            
  attitude.vector.x = roll_d ;                    
  attitude.vector.y = pitch_d;
  attitude.vector.z = yaw_rate;
  attitude.header.frame_id = body_frame_;
  
  thrust_pub_.publish(thrust);
  attitude_pub_.publish(attitude);  
}

/**
* ROS loop
*/

void PID::Spin(void){
  // Odometry based
  if (!use_tf_)
    ros::spin();
  // TF based (not working properly) 
  else{
    //! @todo TFCallback is not working properly. Problem computing the twist with lookUpTwist
    /*
    ros::Rate rate(loop_rate_);
    while(ros::ok()){
      // Try to grab tf
      TFCallback();
      // Sleep until next iteration
      ros::spinOnce();
      rate.sleep();
    }
    */
  }


};

} // linear namespace
} // pose namespace
} // mav_control namespace

int main(int argc, char** argv){
  ros::init(argc, argv,"pose_controller");

  mav_control::pose::linear::PID pose_control;

  pose_control.Spin();

}