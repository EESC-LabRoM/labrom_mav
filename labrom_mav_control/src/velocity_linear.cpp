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

  odom_sub_     = nh_.subscribe("odometry",1,&Controller::OdometryCallback,this);
  twist_sub_    = nh_.subscribe("cmd_vel",1,&Controller::TwistCallback,this);  

  // Controllers parameters from configuration file
  std::vector<double> params;
  std::string coordinate[] = {"/x","/y","/z"}, gains[]={"/kp","/ki","/kd","/anti_windup"};
  for(int i=0;i<3;++i){
    for(int j=0;j<4;++j){
      params.push_back(0);
      pnh_.getParam("velocity"+coordinate[i]+gains[j], params[4*i+j]);
    }
  }

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

/**
* Odometry message callback
* @param[in] odom current estimated odometry
*/
void Controller::OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg){
  // Roll, pitch and yaw angles
  double roll, pitch, yaw;
  tf::Quaternion qt( msg->pose.pose.orientation.x, 
                     msg->pose.pose.orientation.y, 
                     msg->pose.pose.orientation.z, 
                     msg->pose.pose.orientation.w);
  tf::Matrix3x3 R(qt);
  R.getRPY(roll, pitch, yaw);

  // Transform velocities from body-fixed frame to speed frame
  double vx = cos(pitch) * msg->twist.twist.linear.x + sin(roll)*sin(pitch)* msg->twist.twist.linear.y  + cos(roll)*sin(pitch)* msg->twist.twist.linear.z;
  double vy = cos(roll)  * msg->twist.twist.linear.y - sin(roll)*  msg->twist.twist.linear.z;
  double vz = -sin(pitch)* msg->twist.twist.linear.x + cos(pitch)*sin(roll)* msg->twist.twist.linear.y  + cos(roll)*cos(pitch)* msg->twist.twist.linear.z;

  // Command accelerations 
  double ddx_c = pid_ddx_.LoopOnce(twist_.linear.x, vx);
  double ddy_c = pid_ddy_.LoopOnce(twist_.linear.y, vy);
  double ddz_c = pid_ddz_.LoopOnce(twist_.linear.z, vz);
  double yaw_rate = twist_.angular.z;
  
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

  thrust_pub_.publish(thrust);
  attitude_pub_.publish(attitude);

}

//! Twist callback
void Controller::TwistCallback(const geometry_msgs::Twist::ConstPtr &msg){
   twist_ = *msg;
}

} // linear namespace
} // velocity namespace
} // mav_control namespace

int main(int argc, char** argv){
  ros::init(argc, argv,"velocity_controller");

  mav_control::velocity::linear::Controller vel_control;

  ros::spin();

}