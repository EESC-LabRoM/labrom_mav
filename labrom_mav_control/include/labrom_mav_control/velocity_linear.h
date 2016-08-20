/*************************************************************************
*   Linear plant associate with a PID control law for hovering (Header file)
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

#ifndef VELOCITY_LINEAR_H_
#define VELOCITY_LINEAR_H_

//labrom_control libraries
#include<labrom_control/pid_simple.h>

// ROS libraries
#include "ros/ros.h"
#include "tf/tf.h"
// ROS message libraries
#include "nav_msgs/Odometry.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"

#include <vector>

//! top level namespace
namespace mav_control{
//! velocity controllers
namespace velocity{
//! Linearized plants
namespace linear{
//! Linearized quadrotor controllers for velocity commands.
class Controller{
  public:
    //! Empty constructor
    Controller(std::string name);
    //! Empty destructor
    ~Controller(void);
    //! Trajectory message callback
    void TrajCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &msg);
    //! Odometry message callback
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
    //! Shutdown controller
    void Shutdown(void);
    //! Turn controller on
    void TurnControllerOn(void);
    //! Loop
    void Loop(void);

  private:     
    ros::NodeHandle nh_;                                    //!< ROS NodeHandle
    ros::Subscriber traj_sub_;                              //!< trajectory subscriber
    ros::Subscriber odom_sub_;                              //!< odometry subscriber
    ros::Publisher thrust_pub_;                             //!< Thrust publisher
    ros::Publisher attitude_pub_;                           //!< Attitude publisher

    trajectory_msgs::JointTrajectoryPoint traj_des_;        //!< Desired trajectory message

    controllers::pid::Simple pid_ddx_;
    controllers::pid::Simple pid_ddy_;
    controllers::pid::Simple pid_ddz_;
    std::string name_;
    struct{
      double mass;
      double gravity;
    } params_;

};
} // pid namespace
} // linear namespace
} // mav_control namespace
#endif