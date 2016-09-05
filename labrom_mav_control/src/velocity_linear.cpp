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
Controller::Controller(double mass){
  // Controllers parameters from configuration file
  std::vector<double> params;
  std::string coordinate[] = {"/x","/y","/z"}, gains[]={"/kp","/ki","/kd","/anti_windup"};
  for(int i=0;i<3;++i){
    for(int j=0;j<4;++j){
      params.push_back(0);
      ros::param::get("velocity"+coordinate[i]+gains[j], params[4*i+j]);
    }

  }

  // Loading PID controllers with configuration
  pid_ddx_ = controllers::pid::Simple("PID_ddx", params[0], params[1], params[2],  params[3]);
  pid_ddy_ = controllers::pid::Simple("PID_ddy", params[4], params[5], params[6],  params[7]);
  pid_ddz_ = controllers::pid::Simple("PID_ddz", params[8], params[9], params[10], params[11]);

  // Retrieving global parameters
  ros::param::get("/gravity", params_.gravity);

  // vehicle parameters
  params_.mass = mass;
};

/**
* Empty destructor
*/
Controller::~Controller(void){};

/**
* Odometry message callback
* @param[in] traj desired trejacteroy
* @param[in] odom current estimated odometry
* @param[out] thrust value required to follow trajectory 
* @param[out] attitude value required to follow trajectory (roll, pitch, yaw)
*/
void Controller::LoopOnce(const trajectory_msgs::JointTrajectory &traj, const nav_msgs::Odometry &odom, std_msgs::Float32 &thrust, geometry_msgs::Vector3Stamped &attitude){
  // Roll, pitch and yaw angles
  double roll, pitch, yaw;
  tf::Quaternion qt(odom.pose.pose.orientation.x, 
                    odom.pose.pose.orientation.y, 
                    odom.pose.pose.orientation.z, 
                    odom.pose.pose.orientation.w);
  tf::Matrix3x3 R(qt);
  R.getRPY(roll, pitch, yaw);

  // Transform velocities from body-fixed frame to speed frame
  double vx = cos(pitch) *odom.twist.twist.linear.x + sin(roll)*sin(pitch)*odom.twist.twist.linear.y  + cos(roll)*sin(pitch)*odom.twist.twist.linear.z;
  double vy = cos(roll)  *odom.twist.twist.linear.y - sin(roll)* odom.twist.twist.linear.z;
  double vz = -sin(pitch)*odom.twist.twist.linear.x + cos(pitch)*sin(roll)*odom.twist.twist.linear.y  + cos(roll)*cos(pitch)*odom.twist.twist.linear.z;

  // Command accelerations 
  double ddx_c = pid_ddx_.LoopOnce(traj.points[0].velocities[0], vx);
  double ddy_c = pid_ddy_.LoopOnce(traj.points[0].velocities[1], vy);
  double ddz_c = pid_ddz_.LoopOnce(traj.points[0].velocities[2], vz);

  // Quadrotor input commands
  double T_d     = (params_.gravity + ddz_c)*params_.mass;
  double roll_d  = -(params_.mass)/T_d  * ddy_c;
  double pitch_d = (params_.mass)/T_d * ddx_c;

  // Assemble command message
  thrust.data = T_d;        
  attitude.header.frame_id = "fcu";                        
  attitude.vector.x = roll_d ;                    
  attitude.vector.y = pitch_d;
  attitude.vector.z = 0;

}

} // linear namespace
} // velocity namespace
} // mav_control namespace

