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
#include "std_msgs/Int32.h"
#include "geometry_msgs/Vector3.h"
// top-level namespace
namespace manager{
enum ManagerState{IDLE=0, TURN_MOTORS_ON, TAKE_OFF, WAIT_TAKE_OFF, FREE_MODE, LAND, WAIT_LANDING, TURN_MOTORS_OFF};

class Manager{
  public:
    //! Constructor
    Manager();
    //! Destructor
    ~Manager();
    //! Odometry callback
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    //! State machine loop
    void Loop(void);

  private:
    ros::NodeHandle nh_;                //!< ROS nodehandle
    ros::Publisher attitude_pub_;       //!< ROS attitude publisher
    ros::Publisher thrust_pub_;         //!< ROS thrust publisher
    ros::Subscriber odom_sub_;          //!< ROS odometry subscriber

    nav_msgs::Odometry odom_;           //!< odometry message

};


} // manager namespace
#endif // LABROM_MAV_MANAGER_H_