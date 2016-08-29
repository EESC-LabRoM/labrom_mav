/*************************************************************************
*   Manager header files
*   This file is part of labrom_mav_manager
*
*   labrom_mav_manager is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_mav_manager is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_mav_manager.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

#ifndef LABROM_MAV_MANAGER_H_
#define LABROM_MAV_MANAGER_H_

// ROS libraries
#include "ros/ros.h"

// ROS message libraries
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Vector3.h"
// top-level namespace
namespace manager{
enum ManagerState{IDLE=0, TURN_MOTORS_ON, TAKE_OFF, WAIT_TAKE_OFF, CLIMB, WAIT_CLIMB, HOVER, FREE_MODE, LAND, WAIT_LANDING, TURN_MOTORS_OFF};

class Manager{
  public:
    //! Constructor
    Manager();
    //! Destructor
    ~Manager();
    //! Odometry callback
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    //! State machine loop
    void Loop(void);

  private:
    ros::NodeHandle nh_;                //!< ROS nodehandle

    ros::Publisher attitude_pub_;       //!< ROS attitude publisher
    ros::Publisher thrust_pub_;         //!< ROS thrust publisher

    ros::Subscriber imu_sub_;           //!< ROS IMU subscriber

    sensor_msgs::Imu imu_;              //!< imu message

    double max_detected_accel_;
    double min_detected_accel_;
    double time_;

    double _take_off_accel;
    double _land_accel;
    double _climb_time;
};


} // manager namespace
#endif // LABROM_MAV_MANAGER_H_