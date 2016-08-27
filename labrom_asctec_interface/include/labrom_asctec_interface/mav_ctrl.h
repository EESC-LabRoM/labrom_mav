/*************************************************************************
*   Transform commands from labrom to asctec_hl_interface format (Header file)
*   This file is part of labrom_asctec_interface
*
*   labrom_asctec_interface is free software: you can redistribute it and/or modify
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


#ifndef MAV_CTRL_H_
#define MAV_CTRL_H_

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <asctec_hl_comm/mav_ctrl.h>

namespace labrom_asctec_interface{
class CtrlNode{
  public:
    //! Constructor
    void CtrlNode(void);
    //! Destructor
    void ~CtrlNode(void);
    //! Thrust callback
    void ThrustCallback(const std_msgs::Int32::ConstPtr &msg);
    //! Attitude callback
    void AttitudeCallback(const geometry_msgs::Vector3::ConstPtr &msg);
    //! Pubish mav_ctrl message
    void PublishMavCtrl(void);
  
  private: 
    ros::NodeHandle nh_;                      //!< ROS node handle
    ros::Rate loop_rate_;                     //!< Loop rate
    ros::Subscriber sub_thrust_;              //!< Thrust subscriber
    ros::Subscriber sub_attitude_;            //!< Attitude subscriber
    ros::Publisher pub_mav_ctrl_;             //!< Mav ctrl publisher
	
    geometry_msgs::Vector3 attitude_;		//!< received attitude message
    std_msgs::Int32 thrust_;		        //!< received thrust message
    asctec_hl_comm::mav_ctrl mav_ctrl_;		//!< mav_ctrl message to be sent 
};
} // labrom_asctec_interface namespace
#endif
