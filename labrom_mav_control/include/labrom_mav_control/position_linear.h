/*************************************************************************
*   Linear plant associate with a PID control law for position control (Header file)
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

#ifndef POSITION_LINEAR_H_
#define POSITION_LINEAR_H_

//labrom_control libraries
#include<labrom_control/pid_simple.h>

// ROS libraries
#include "ros/ros.h"
#include "tf/tf.h"
// ROS message libraries
#include "nav_msgs/Odometry.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3Stamped.h"
// Libraries
#include <Eigen/Geometry>

#include <vector>

//! top level namespace
namespace mav_control{
//! position controllers
namespace position{
//! Linearized plants
namespace linear{
//! Linearized quadrotor controllers for position commands.
class Controller{
  public:
    //! Empty constructor
    Controller(double mass);
    //! Empty destructor
    ~Controller(void);
    //! Odometry message callback
    void LoopOnce(const trajectory_msgs::JointTrajectory &traj, const nav_msgs::Odometry &odom, std_msgs::Float32 &thrust, geometry_msgs::Vector3Stamped &attitude);

  private:     
    controllers::pid::Simple pid_ddx_;
    controllers::pid::Simple pid_ddy_;
    controllers::pid::Simple pid_ddz_;

    struct{
      double mass;
      double gravity;
    } params_;

};
} // pid namespace
} // linear namespace
} // mav_control namespace
#endif