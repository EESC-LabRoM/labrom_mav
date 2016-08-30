/*************************************************************************
*   Linear plant associate with a PID control law for hovering (Implemenation)
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
Controller::Controller(double mass, double gravity){
  // Controllers
  std::vector<double> kp, ki, kd, anti_windup;
  for(int i=0;i<3;++i){
    kp.push_back(0);
    ki.push_back(0);
    kd.push_back(0);
    anti_windup.push_back(0);
    traj_des_.velocities.push_back(0);
  }

  pid_ddx_ = controllers::pid::Simple("PID_ddx", kp[0], ki[0],  kd[0],  anti_windup[0]);
  pid_ddy_ = controllers::pid::Simple("PID_ddy", kp[1], ki[1],  kd[1],  anti_windup[1]);
  pid_ddz_ = controllers::pid::Simple("PID_ddz", kp[2], ki[2],  kd[2],  anti_windup[2]);

  // parameters
  params_.mass = mass;
  params_.gravity = gravity;
};

/**
* Empty destructor
*/
Controller::~Controller(void){};

/**
*  Trajectory message callback
* @param msg last trajectory message received
*/
void Controller::UpdateTrajectory(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg){
  // Save trajectory
  traj_des_ = *msg;
}

/**
* Odometry message callback
*/
void Controller::Iterate(const nav_msgs::Odometry::ConstPtr &msg, std_msgs::Float32 &thrust, geometry_msgs::Vector3 &attitude){
  // Roll, pitch and yaw angles
  double roll, pitch, yaw;
  tf::Quaternion qt(msg->pose.pose.orientation.x, 
                    msg->pose.pose.orientation.y, 
                    msg->pose.pose.orientation.z, 
                    msg->pose.pose.orientation.w);
  tf::Matrix3x3 R(qt);
  R.getRPY(roll, pitch, yaw);

  // Transform velocities from body-fixed frame to speed frame
  double vx = cos(pitch) *msg->twist.twist.linear.x + sin(roll)*sin(pitch)*msg->twist.twist.linear.y  + cos(roll)*sin(pitch)*msg->twist.twist.linear.z;
  double vy = cos(roll)  *msg->twist.twist.linear.y - sin(roll)* msg->twist.twist.linear.z;
  double vz = -sin(pitch)*msg->twist.twist.linear.x + cos(pitch)*sin(roll)*msg->twist.twist.linear.y  + cos(roll)*cos(pitch)*msg->twist.twist.linear.z;

  // Command accelerations 
  double ddx_c = pid_ddx_.LoopOnce(traj_des_.velocities[0], vx);
  double ddy_c = pid_ddy_.LoopOnce(traj_des_.velocities[1], vy);
  double ddz_c = pid_ddz_.LoopOnce(traj_des_.velocities[2], vz);


  // Quadrotor input commands
  double T_d     = (params_.gravity - ddz_c)*params_.mass;
  double roll_d  = (params_.mass)/T_d  * ddy_c;
  double pitch_d = -(params_.mass)/T_d * ddx_c;

  // Assemble command message
  thrust.data = T_d;                                
  attitude.x = roll_d ;                    
  attitude.y = pitch_d;
  attitude.z = 0;

}

} // linear namespace
} // velocity namespace
} // mav_control namespace

