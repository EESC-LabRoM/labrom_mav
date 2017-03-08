/*************************************************************************
*   Transform VRPN ground truth to odometry ROS format (Header file)
*   This file is part of labrom_asctec_interface
*
*   labrom_mav_interface is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_mav_interfaceis distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should  have received a copy of the GNU General Public License
*   along with labrom_mav_interface.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/


#ifndef LABROM_MAV_INTERFACE_VRPN_H_
#define LABROM_MAV_INTERFACE_VRPN_H_

// labrom_mav_common libraries
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// ROS libraries
#include <ros/ros.h>

// top level namespace
namespace labrom_mav_interface{
// Interface to labrom_mav_interface controls
class VRPN{
  public:
    //! Constructor
    VRPN(void);
    //! Destructor
    ~VRPN(void);
    //! mav_cmd subscriber
    void PoseSubscriber(const geometry_msgs::PoseStamped::ConstPtr &msg); 
    
  private: 
    ros::NodeHandle m_nh;                      //!< ROS node handle
    ros::NodeHandle m_pnh;                     //!< ROS node handle   
    ros::Publisher m_odometry_pub;             //!< Subscriber Publisher
    ros::Subscriber m_pose_sub;                //!< Pose subscriber
 
};

} // labrom_mav_interface namespace
#endif
