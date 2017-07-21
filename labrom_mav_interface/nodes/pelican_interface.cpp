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

#include "labrom_mav_interface/pelican_interface.h"

namespace labrom_mav_interface{

/**
 * Empty constructor.
 */
Pelican::Pelican(void): m_pnh("~"){
  // subscribers
  m_mav_ctrl_pub = m_nh.advertise<asctec_hl_comm::mav_ctrl>("control", 1);

  // publishers
  m_mav_cmd_sub = m_nh.subscribe("mav_cmd", 1, &Pelican::MavCmdSubscriber, this);

  // Parameters
  float t0 = 0.545;
  float m0 = 1.63;
  float g = 9.81;
  m_thrust_gain = (t0 * (m0 * g));

};

/**
 * Empty destructor.
 */
Pelican::~Pelican(void){};

/**
 * Subscribe to MavCmd message

 */
void Pelican::MavCmdSubscriber(const labrom_mav_common::MavCmd::ConstPtr &msg){
  asctec_hl_comm::mav_ctrl mav_ctrl;

  // Assemble message
  double max_pitch_roll = (M_PI/180)*20.0;
  double max_yaw = (2*M_PI)/5;
  mav_ctrl.x = std::min(std::max(msg->pitch, -max_pitch_roll), max_pitch_roll);
  mav_ctrl.y = std::min(std::max(msg->roll, -max_pitch_roll), max_pitch_roll);
  mav_ctrl.yaw = std::min(std::max(msg->yaw, -max_yaw), max_yaw);
  mav_ctrl.z = std::min(std::max((m_thrust_gain * msg->thrust), 0.3), 0.6);

  m_mav_ctrl_pub.publish(mav_ctrl);
}

} // labrom_asctec_interface namespace

int main(int argc, char **argv){
  // Initialize ROS within this node
  ros::init(argc, argv, "labrom_pelican_interface");
   // Ctrl interface node
  labrom_mav_interface::Pelican node;
  // Loop
  ros::spin();

}
