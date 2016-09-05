/*************************************************************************
*   Manager header files
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

#ifndef MAV_MANAGER_H_
#define MAV_MANAGER_H_

// Control laws
#include <labrom_mav_control/velocity_linear.h>

// ROS libraries
#include "ros/ros.h"

// ROS message libraries
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "trajectory_msgs/JointTrajectory.h"

// top-level namespace
namespace mav_control{
namespace manager{
enum ManagerState{IDLE=0, TURN_MOTORS_ON, TAKE_OFF, WAIT_TAKE_OFF, CLIMB, WAIT_CLIMB=5, HOVER, HOVERING, FREE_MODE_VELOCITY, LAND, WAIT_LANDING, TURN_MOTORS_OFF};

class Manager{
  public:
    //! Constructor
    Manager();
    //! Destructor
    ~Manager();
    //! IMU callback
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    //! Odometry callback
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
    //! Trajectory callback
    void TrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
    //! State machine loop
    void Spin(void);

  private:
    ros::NodeHandle nh_;                //!< ROS nodehandle
    ros::NodeHandle pnh_;                //!< ROS nodehandle (private)

    ros::Publisher attitude_pub_;       //!< ROS attitude publisher
    ros::Publisher thrust_pub_;         //!< ROS thrust publisher

    ros::Subscriber imu_sub_;           //!< ROS IMU subscriber
    ros::Subscriber odom_sub_;          //!< ROS odometry subscriber
    ros::Subscriber traj_sub_;          //!< ROS trajectory subscriber

    sensor_msgs::Imu imu_;                   //!< imu message
    nav_msgs::Odometry odom_;                //!< odometry message
    trajectory_msgs::JointTrajectory traj_;  //!< trajectory message

    bool is_odom_active_;

    double max_detected_accel_;
    double min_detected_accel_;
    double time_;
  
};

} // manager namespace
} // control namespace
#endif // MAV_MANAGER_H_