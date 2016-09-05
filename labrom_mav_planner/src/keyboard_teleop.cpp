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
    trajectory_.positions.push_back(0);
    trajectory_.velocities.push_back(0);
    trajectory_.accelerations.push_back(0);
    trajectory_.effort.push_back(0);
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
void Keyboard::KeyPressed(int &key){
  switch (key){
    case (KEY_UP):
      trajectory_.velocities[0] = 0.1;
      break;
    case (KEY_DOWN):
      trajectory_.velocities[0] = -0.1;
      break;
    case (KEY_RIGHT):
      trajectory_.velocities[1] = 0.1;
      break;
    case (KEY_LEFT):
      trajectory_.velocities[1] = -0.1;
      break;    
    case (KEY_a):
      trajectory_.velocities[2] = 0.1;
      break;
    case (KEY_z):
      trajectory_.velocities[2] = -0.1;
      break;   
  }
}

/**
 * Key unpressed event handle
 * @param[in] key unpressed
 */
void Keyboard::KeyUnpressed(int &key){
  switch (key){
    case (KEY_UP):
      trajectory_.velocities[0] = 0;
      break;
    case (KEY_DOWN):
      trajectory_.velocities[0] = 0;
      break;
    case (KEY_RIGHT):
      trajectory_.velocities[1] = 0;
      break;
    case (KEY_LEFT):
      trajectory_.velocities[1] = 0;
      break;    
    case (KEY_a):
      trajectory_.velocities[2] = 0;
      break;
    case (KEY_z):
      trajectory_.velocities[2] = 0;
      break;   
  }
}

/**
 * Get keyboard teleop command
 * @return trajectory_msgs::JointTrajectoryPoint message 
 */
trajectory_msgs::JointTrajectoryPoint  Keyboard::GetKeyboardTeleopCommand(void){
  return trajectory_;
}


} // teleop namespace
} // mav_planner namespace
