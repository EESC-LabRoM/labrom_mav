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

#include "labrom_mav_interface/crazyflie_interface.h"

namespace labrom_mav_interface{

/**
 * Empty constructor.
 */
Crazyflie::Crazyflie(void): m_pnh("~"){
  // subscribers
  m_cmd_vel_pub   = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
  // publishers
  m_mav_cmd_sub = m_nh.subscribe("mav_cmd", 1, &Crazyflie::MavCmdSubscriber, this);

  // Parameters
  m_pnh.param("thrust_gain", m_thrust_gain, 125000.0);

};

/**
 * Empty destructor. 
 */
Crazyflie::~Crazyflie(void){};

/**
 * Subscribe to MavCmd message 

 */
void Crazyflie::MavCmdSubscriber(const labrom_mav_common::MavCmd::ConstPtr &msg){ 
  geometry_msgs::Twist cmd_vel;
     
  // Assemble message
  cmd_vel.linear.x = std::min(std::max(180/M_PI*msg->pitch, -20.0), 20.0);
  cmd_vel.linear.y = std::min(std::max(180/M_PI*msg->roll, -20.0), 20.0);
  cmd_vel.linear.z = std::min(std::max(m_thrust_gain*msg->thrust, 0.0), 50000.0) + 10000;  
  cmd_vel.angular.z = -180/M_PI*msg->wz;
  
  // 
  if (cmd_vel.linear.z < 10001)
    cmd_vel.linear.z = 0;
    
  m_cmd_vel_pub.publish(cmd_vel);
}

} // labrom_asctec_interface namespace

int main(int argc, char **argv){
  // Initialize ROS within this node
  ros::init(argc, argv, "labrom_crazyflie_interface");
   // Ctrl interface node
  labrom_mav_interface::Crazyflie node;
  // Loop
  ros::spin();  
 
}
