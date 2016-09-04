/*************************************************************************
*   Keyboard tele operation implementation
*   This file is part of labrom_mav_planner
*
*   labrom_mav_planner is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_mav_planner is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_mav_planner.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

// labrom_mav_planner library
#include "labrom_mav_planner/keyboard_teleop.h"

namespace mav_planner{
namespace teleop{

/**
 * Constructor
 * @param[in] key_gain that boost corresponding key actuation.
 */
Keyboard::Keyboard(double key_gain): _key_gain(key_gain){
  for(int i=0; i<4; ++i){
    positions.push_back(0);
    velocities.push_back(0);
    accelerations.push_back(0);
    effort.push_back(0);
  }
};

/**
 * Destructor.
 */ 
Keyboard::~Keyboard(){};

/**
 * Key pressed event handle
 * @param[in] key pressed
 */
void Keyboard::KeyPressed(uint16 key){
  
}

/**
 * Key unpressed event handle
 * @param[in] key pressed
 */
void Keyboard::KeyUnpressed(uint16 key){
  
}

/**
 * Get keyboard teleop command
 * @return trajectory_msgs::JointTrajectoryPoint message 
 */
trajectory_msgs::JointTrajectoryPoint GetKeyboardTeleopCommand(void){
  return trajectory_;
}


} // teleop namespace
} // mav_planner namespace
