/*************************************************************************
*   Linearized plant associate with a PID control law for pose control (Header files)
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

#include <ros/ros.h>
// labrom_mav_control libraries
#include <labrom_mav_control/PoseLinear.h>
#include <Eigen/Geometry> 

//! Top level namespace
namespace labrom_mav_control{
/* Constructor */
PoseLinear::PoseLinear(void){}

/* Constructor */
PoseLinear::PoseLinear(labrom_mav_common::MavParams mav_params, std::string filename, std::string name): m_name(name)
{
  // Initialize PIDs
  if (name.size()>0)
    name += ".";
  m_pidX = labrom_mav_common::controller::PID(filename, m_name + "pose.x");
  m_pidY = labrom_mav_common::controller::PID(filename, m_name + "pose.y");
  m_pidZ = labrom_mav_common::controller::PID(filename, m_name + "pose.z");
  
  // Mav parameters (please, give the mass!! Otherwise mass = 0 ad control output will be zero)
  m_mav = mav_params;
};

/* Destructor */
PoseLinear::~PoseLinear(){};

/* Reset */
void PoseLinear::Reset()
{
  m_feedforward = labrom_mav_common::MavCommand();
  m_pidX.Reset();
  m_pidY.Reset();
  m_pidZ.Reset();    
}

/* Set FeedForward */
void PoseLinear::SetFeedforward(labrom_mav_common::MavCommand feedforward)
{
  m_feedforward = feedforward;
}

/* Run */
labrom_mav_common::MavCommand PoseLinear::Run(const labrom_mav_common::State &curr_state, const labrom_mav_common::State &goal_state, double dt)
{
  Eigen::Vector3d curr_pos = Labrom2Eigen(curr_state.pose.position);
  Eigen::Vector3d goal_pos =  Labrom2Eigen(goal_state.pose.position);
  labrom_mav_common::Euler curr_euler = curr_state.pose.euler;
  labrom_mav_common::Euler des_euler = goal_state.pose.euler;
  
  // Distance to goal
  double distance = (goal_pos - curr_pos).norm();
  
  // Saturate if distance is to far away from current position
  Eigen::Vector3d target_pos;
  if (distance > 1)
    target_pos = curr_pos + 1*(goal_pos - curr_pos)/distance;
  else 
    target_pos = goal_pos;
  
  target_pos(2) = goal_pos(2);
  
  // Compute desired acceleration  
  double ddx = m_pidX.Run(curr_pos(0), target_pos(0), dt);
  double ddy = m_pidY.Run(curr_pos(1), target_pos(1), dt);
  double ddz = m_pidZ.Run(curr_pos(2), target_pos(2), dt);

  // Mapping to quadrotor command
  labrom_mav_common::MavCommand mav_command;
  mav_command.thrust = m_mav.mass*(ddz + 9.81);
  mav_command.euler.pitch =  m_mav.mass*(ddx*cos(curr_euler.yaw) + ddy*sin(curr_euler.yaw) )/ mav_command.thrust;  
  mav_command.euler.roll  = -m_mav.mass*(-ddx*sin(curr_euler.yaw) + ddy*cos(curr_euler.yaw) )/ mav_command.thrust;  

  double delta_yaw = (0 - curr_euler.yaw);
  
  if(delta_yaw > M_PI )
    delta_yaw -= 2*M_PI;
  else if (delta_yaw < -M_PI)
    delta_yaw += 2*M_PI;
    
    

    
  mav_command.angular_vel.z = 0.2*delta_yaw; 
  return (mav_command); 
};

} // labrom_mav_control
