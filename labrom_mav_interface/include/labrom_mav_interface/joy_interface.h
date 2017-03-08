/*************************************************************************
*   Transform commands from joy to labrom_mav_common ROS format (Header file)
*   This file is part of labrom_asctec_interface
*
*   labrom_mav_interface is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_mav_interface is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_mav_interface.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/


#ifndef LABROM_MAV_INTERFACE_JOY_H_
#define LABROM_MAV_INTERFACE_JOY_H_

// labrom_mav_common libraries
#include <labrom_mav_common/TeleOpCmd.h>

// ROS libraries
#include <ros/ros.h>

// ROS message libraries
#include <sensor_msgs/Joy.h>

// top level namespace
namespace labrom_mav_interface{
// Interface to joy controls
class Joy{
  public:
    //! Constructor
    Joy(void);
    //! Destructor
    ~Joy(void);
    //! mav_cmd subscriber
    void JoySubscriber(const sensor_msgs::Joy::ConstPtr &msg); 
    
  private: 
    ros::NodeHandle m_nh;                      //!< ROS node handle
    ros::NodeHandle m_pnh;                     //!< ROS node handle   
    ros::Publisher m_teleop_pub;               //!< LAbrom TeleopCmd Publisher
    ros::Subscriber m_joy_sub;                 //!< ROS joy subscriber
    
};

} // labrom_mav_interface namespace
#endif
