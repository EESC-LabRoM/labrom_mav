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

// ROS libraries
#include <ros/ros.h>
#include <tf/transform_listener.h>

// ROS message libraries
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>

// Asctec libraries
#include <asctec_hl_comm/mav_ctrl.h>

// top level namespace
namespace labrom_asctec_interface{
// Interface to asctec_hl_interface controls
class CtrlNode{
  public:
    //! Constructor
    CtrlNode(void);
    //! Destructor
    ~CtrlNode(void);
    //! Thrust callback
    void ThrustCallback(const std_msgs::Float32::ConstPtr &msg);
    //! Attitude callback
    void AttitudeCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
    //! Pubish mav_ctrl message
    void PublishMavCtrl(void);
    //! Sping
    void Spin(void);
  
  private: 
    ros::NodeHandle nh_;                      //!< ROS node handle
    ros::NodeHandle pnh_;                      //!< ROS node handle   
    ros::Subscriber sub_thrust_;              //!< Thrust subscriber
    ros::Subscriber sub_attitude_;            //!< Attitude subscriber
    ros::Publisher pub_mav_ctrl_;             //!< Mav ctrl publisher
	
    tf::TransformListener tf_listener_;        //!< tf listener

    geometry_msgs::Vector3Stamped attitude_;		//!< received attitude message
    std_msgs::Float32 thrust_;		                //!< received thrust message
    asctec_hl_comm::mav_ctrl mav_ctrl_;	      	//!< mav_ctrl message to be sent
    
    // params
    std::string _control_frame_id;      //!< control frame
    double _max_thrust;				          //!< maximum quad thrust in Newtons
    int _loop_rate;			     	          //!< ROS loop rate
};
} // labrom_asctec_interface namespace
#endif
