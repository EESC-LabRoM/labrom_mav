/*************************************************************************
*   Transform commands from joy to labrom_mav_common ROS format (node implementation)
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

#include "labrom_mav_interface/vrpn_interface.h"

namespace labrom_mav_interface{

/**
 * Empty constructor.
 */
VRPN::VRPN(void): m_pnh("~"){
  // publishers
  m_odometry_pub   = m_nh.advertise<nav_msgs::Odometry>("odometry", 1);
  
  // subscribers
  m_pose_sub = m_nh.subscribe("pose", 1, & VRPN::PoseSubscriber, this);

};

/**
 * Empty destructor. 
 */
VRPN::~VRPN(void){};

/**
 * Subscribe to pose message from motioncap 

 */
void VRPN::PoseSubscriber(const geometry_msgs::PoseStamped::ConstPtr &msg){ 
  nav_msgs::Odometry odometry;
  
  // Copy message but stamp current time
  odometry.header = msg->header;
  odometry.header.stamp = ros::Time::now();
  odometry.pose.pose = msg->pose;
  
  m_odometry_pub.publish(odometry);
}

} // labrom_asctec_interface namespace

int main(int argc, char **argv){
  // Initialize ROS within this node
  ros::init(argc, argv, "labrom_joy_interface");
   // Ctrl interface node
  labrom_mav_interface::VRPN node;
  // Loop
  ros::spin();  
 
}
