/*************************************************************************
*   Transform commands from labrom to asctec_hl_interface format (node implementation)
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

#include "labrom_asctec_interface.h/mav_ctrl.h"

namespace labrom_asctec_interface{

/**
 * Empty constructor.
 */
CtrlNode::CtrlNode(void): nh_("~"){
  // Paramas
  nh_.param("max_thrust", _max_thrust, 17.1675);
  nh_.param("loop_rate", _loop_rate, 20);

  // Start subscribers
  sub_thrust_   = node.subscribe("/cmd_thrust"  , 1, thrustCallback);
  sub_attitude_ = node.subscribe("/cmd_attitude", 1, attitudeCallback);
  
  // Start publishers
  pub_mav_ctrl_ = node.advertise<asctec_hl_comm::mav_ctrl>("control", 1);

};

/**
 * Empty destructor. 
 */
CtrlNode::~CtrlNode(void){};

/**
 * Thrust callback. Save last thrust command received 
 */
void CtrlNode::ThrustCallback(const std_msgs::Int32::ConstPtr &msg){
  thrust_ = *msg;
}

/**
 * Attitude callback. Save last attitude command received 
 */
void CtrlNode::AttitudeCallback(const geometry_msgs::Vector3::ConstPtr &msg){
  attitude_ = *msg;
}

/**
 * Publish control message in asctec_hl_interface format
 */
void CtrlNode::PublishMavCtrl(void){
    // Control method
    switch (msg_mav_ctrl.type){
      case 1: // x~pitch, y~roll, z~thrust, units in rad and rad/s for yaw
              msg_mav_ctrl.x = attitude.y;                  // pitch
              msg_mav_ctrl.y = attitude.x;                  // roll
              msg_mav_ctrl.z = thrust.data / _max_thrust;   // yaw
              msg_mav_ctrl.yaw = attitude.z;    
              break; 
      case 2: //  x, y, z, yaw correspond to x_dot, y_dot, ...
              msg_mav_ctrl.x = attitude.y;                  // pitch
              msg_mav_ctrl.y = attitude.x;                  // roll
              msg_mav_ctrl.z = thrust.data / _max_thrust;   // yaw
              msg_mav_ctrl.yaw = attitude.z;    
              break;      
      default:msg_mav_ctrl.x = 0;   
              msg_mav_ctrl.y = 0;   
              msg_mav_ctrl.z = 0;   
              msg_mav_ctrl.yaw = 0;
              break;
    }
    // Publish message
    pub_mav_ctrl.publish(msg_mav_ctrl);
}

/**
 * ROS spin loop
 */
void CtrlNode::Spin(void){
  ros::Rate rate(_loop_rate);
  
  while(ros::ok()){
    // Publish control message
    PublishMavCtrl();
    // Sleep
    rate.sleep();
  }
}

} // labrom_asctec_interface namespace

int main(int argc, char **argv){
  // Initialize ROS within this node
  ros::init(argc, argv, "labrom_asctec_interface_ctrl");
  // Ctrl interface node
  labrom_asctec_interface::CtrlNode node;
  // Loop
  node.Spin();
  
}
