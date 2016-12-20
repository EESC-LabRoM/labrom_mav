/*************************************************************************
*   Linear plant associate with a PID control law for velocity control (Implemenation)
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
#include <labrom_mav_control/velocity_linear.h>
#include <vector>
namespace mav_control{
namespace velocity{
namespace linear{
/**
* Empty constructor
*/
Controller::Controller(void): pnh_("~"){
  // Adversite and subscribe topics
  attitude_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("cmd_attitude",1);
  thrust_pub_   = nh_.advertise<std_msgs::Float32>("cmd_thrust",1);
  cmdvel_sub_   = nh_.subscribe("cmd_vel",1,&Controller::CmdVelCallback,this);

  // Controllers parameters from configuration file
  std::vector<double> params;
  std::string coordinate[] = {"/x","/y","/z"}, gains[]={"/kp","/ki","/kd","/anti_windup"};
  for(int i=0;i<3;++i){
    for(int j=0;j<4;++j){
      params.push_back(0);
      pnh_.getParam("velocity"+coordinate[i]+gains[j], params[4*i+j]);
    }
  }

  pnh_.param<std::string>("world_frame", world_frame_, "/world");
  pnh_.param<std::string>("body_frame", body_frame_, "/base_link");
  pnh_.param<int>("loop_rate", loop_rate_, 50);
  pnh_.param<bool>("use_tf", use_tf_, false);

  // Loading PID controllers with configuration
  pid_ddx_ = controllers::pid::Simple("PID_ddx", params[0], params[1], params[2],  params[3]);
  pid_ddy_ = controllers::pid::Simple("PID_ddy", params[4], params[5], params[6],  params[7]);
  pid_ddz_ = controllers::pid::Simple("PID_ddz", params[8], params[9], params[10], params[11]);

  // Retrieving vehicle parameters
  pnh_.getParam("mass", params_.mass);

  // vehicle parameters
  params_.gravity = 9.81;

};

/**
* Empty destructor
*/
Controller::~Controller(void){};

/** Twist callback. Receives desired twist.
* @param[in] msg last twist message receivedist
*/
void Controller::CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg){
   desired_twist_ = *msg;
}

/**
* Odometry message callback
* @param[in] odom current estimated odometry
*/
void Controller::OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg){
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
void Controller::TFCallback(void){
  try{
    geometry_msgs::PoseStamped pose;
    
    // Pose
    tf::StampedTransform transform;
    listener_.lookupTransform	(	world_frame_,   
                              	body_frame_,
                                ros::Time(0),
                                transform); 
    tf::poseTFToMsg(tf::Pose(transform), pose.pose);

    // Twist
    geometry_msgs::TwistStamped twist; 
    listener_.lookupTwist(body_frame_,
                          world_frame_,
                          body_frame_,
                          tf::Point(0,0,0),
                          body_frame_,
                          ros::Time(0), 
                          ros::Duration(0.001),
                          twist.twist );

    // Applying rotation to expresse twist w.r.t. body_frame
    transform.setOrigin(tf::Vector3(0,0,0));
    transform.setRotation(transform.inverse().getRotation());
    tf::vector3TFToMsg(
            transform*tf::Vector3(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z),
            twist.twist.linear);
    tf::vector3TFToMsg(
            transform*tf::Vector3(twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z),
            twist.twist.angular);

    // Call method that actually computes/publishes control signals.
    ComputeActuation(pose, twist);

  }catch (tf::TransformException ex){
    ROS_ERROR("[Velocity Controller]%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

/**
* Compute vehicle actuation input (collective thrust and attitude angles)
* @param[in] pose contains the current vehicle pose
* @param[in] twist contains the current vehicle twist
*/
void Controller::ComputeActuation(const geometry_msgs::PoseStamped &pose,const geometry_msgs::TwistStamped &twist){
  // Roll, pitch and yaw angles
  double roll, pitch, yaw;
  tf::Quaternion qt( pose.pose.orientation.x, 
                     pose.pose.orientation.y, 
                     pose.pose.orientation.z, 
                     pose.pose.orientation.w);
  tf::Matrix3x3 R(qt);
  R.getRPY(roll, pitch, yaw);

  // Transform velocities from body-fixed frame to speed frame
  double vx = cos(pitch) * twist.twist.linear.x + sin(roll)*sin(pitch)* twist.twist.linear.y  + cos(roll)*sin(pitch)* twist.twist.linear.z;
  double vy = cos(roll)  * twist.twist.linear.y - sin(roll)*  twist.twist.linear.z;
  double vz = -sin(pitch)* twist.twist.linear.x + cos(pitch)*sin(roll)* twist.twist.linear.y  + cos(roll)*cos(pitch)* twist.twist.linear.z;

  // Command accelerations 
  double ddx_c = pid_ddx_.LoopOnce(desired_twist_.linear.x, vx);
  double ddy_c = pid_ddy_.LoopOnce(desired_twist_.linear.y, vy);
  double ddz_c = pid_ddz_.LoopOnce(desired_twist_.linear.z, vz);
  double yaw_rate = desired_twist_.angular.z;

  // Quadrotor input commands
  double T_d     = (params_.gravity - ddz_c)*params_.mass;
  double roll_d  =  1/params_.gravity  * ddy_c; 
  double pitch_d = -1/params_.gravity  * ddx_c; 

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

void Controller::Spin(void){
  // Odometry based
  if (!use_tf_) {
    odom_sub_     = nh_.subscribe("odometry",1,&Controller::OdometryCallback,this);
    ros::spin();
    // TF based (not working properly)
  }else{
    ros::Rate rate(loop_rate_);
    while(ros::ok()) {
      // Try to grab tf
      TFCallback();
      // Sleep until next iteration
      ros::spinOnce();
      rate.sleep();
    }
  }


};

} // linear namespace
} // velocity namespace
} // mav_control namespace

int main(int argc, char** argv){
  ros::init(argc, argv,"velocity_controller");

  mav_control::velocity::linear::Controller vel_control;

 vel_control.Spin();

}