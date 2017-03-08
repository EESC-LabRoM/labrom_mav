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

#include "labrom_mav_interface/joy_interface.h"

namespace labrom_mav_interface{

/**
 * Empty constructor.
 */
Joy::Joy(void): m_pnh("~"){
  // publishers
  m_teleop_pub   = m_nh.advertise<labrom_mav_common::TeleOpCmd>("teleop_cmd", 1);
  
  // subscribers
  m_joy_sub = m_nh.subscribe("joy", 1, &Joy::JoySubscriber, this);

};

/**
 * Empty destructor. 
 */
Joy::~Joy(void){};

/**
 * Subscribe to MavCmd message 

 */
void Joy::JoySubscriber(const sensor_msgs::Joy::ConstPtr &msg){ 
  labrom_mav_common::TeleOpCmd teleop;
  
  teleop.emergency = msg->buttons[1] > 0;
  teleop.hard_land = msg->buttons[0] > 0;
  teleop.take_off  = msg->buttons[2] > 0;
  teleop.soft_land = msg->buttons[3] > 0;
  teleop.hover     = msg->buttons[5] > 0;        
  teleop.go2point  = msg->buttons[4] > 0;
  teleop.track     = msg->buttons[6] > 0;;
  teleop.restart   = msg->buttons[8] > 0;
  
  m_teleop_pub.publish(teleop);
}

} // labrom_asctec_interface namespace

int main(int argc, char **argv){
  // Initialize ROS within this node
  ros::init(argc, argv, "labrom_joy_interface");
   // Ctrl interface node
  labrom_mav_interface::Joy node;
  // Loop
  ros::spin();  
 
}
