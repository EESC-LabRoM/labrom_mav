/*************************************************************************
*   Transform commands from labrom to labrom_mav_interface format (node implementation)
*   This file is part of labrom_mav_interface
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

#include "labrom_mav_interface/vrep_interface.h"

namespace labrom_mav_interface{

/**
 * Empty constructor.
 */
VREP::VREP(void): m_pnh("~"){
  // subscribers
  m_thrust_pub   = m_nh.advertise<std_msgs::Float32>("cmd_thrust"  , 1);
  m_attitude_pub = m_nh.advertise<geometry_msgs::Vector3>("cmd_attitude", 1);
  
  // publishers
  m_mav_cmd_sub = m_nh.subscribe("mav_cmd", 1, &VREP::MavCmdSubscriber, this);

};

/**
 * Empty destructor. 
 */
VREP::~VREP(void){};

/**
 * Subscribe to MavCmd message 

 */
void VREP::MavCmdSubscriber(const labrom_mav_common::MavCmd::ConstPtr &msg){ 
  std_msgs::Float32 thrust;
  geometry_msgs::Vector3 attitude;
  
  // Assemble message
  thrust.data = msg->thrust;
  attitude.x = msg->roll;
  attitude.y = msg->pitch;    
  attitude.z = msg->wz;
  
  m_thrust_pub.publish(thrust);
  m_attitude_pub.publish(attitude);  
}

} // labrom_asctec_interface namespace

int main(int argc, char **argv){
  // Initialize ROS within this node
  ros::init(argc, argv, "labrom_vrep_interface");
   // Ctrl interface node
  labrom_mav_interface::VREP node;
  // Loop
  ros::spin();  
 
}
