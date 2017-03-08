/*************************************************************************
*   Transform commands from labrom_mav_common to crazyflie ROS format (Header file)
*   This file is part of labrom_asctec_interface
*
*   labrom_mav_interface is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_asctec_interface is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_asctec_interface.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/


#ifndef LABROM_MAV_INTERFACE_CRAZYFLIE_H_
#define LABROM_MAV_INTERFACE_CRAZYFLIE_H_

// labrom_mav_common libraries
#include <labrom_mav_common/typedef.h>

// ROS libraries
#include <ros/ros.h>

// ROS message libraries
#include <geometry_msgs/Twist.h>

// top level namespace
namespace labrom_mav_interface{
// Interface to labrom_mav_interface controls
class Crazyflie{
  public:
    //! Constructor
    Crazyflie(void);
    //! Destructor
    ~Crazyflie(void);
    //! mav_cmd subscriber
    void MavCmdSubscriber(const labrom_mav_common::MavCmd::ConstPtr &msg); 
    
  private: 
    ros::NodeHandle m_nh;                      //!< ROS node handle
    ros::NodeHandle m_pnh;                     //!< ROS node handle   
    ros::Publisher m_cmd_vel_pub;              //!< CmdVel Publisher
    ros::Subscriber m_mav_cmd_sub;             //!< Mav ctrl subscriber
    
    double m_thrust_gain;
};

} // labrom_mav_interface namespace
#endif
