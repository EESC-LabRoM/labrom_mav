/*************************************************************************
*   Keyboard tele operation header files
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
#ifndef KEYBOARD_TELEOP_H_
#define KEYBOARD_TELEOP_H_

// ROS message libraries
#include <keyboard/Key.h>
#include "trajectory_msgs/JointTrajectoryPoint.h"

namespace mav_planner{
namespace teleop{
class Keyboard{
  public:
    //! Constructor
    Keyboard(double _key_gain = 0.1);
    //! Destructor
    ~Keyboard();
    //! Key pressed event handle
    void KeyPressed(uint16 key);
    //! Key unpressed event handle
    void KeyUnpressed(uint16 key);
    //! Get command
    trajectory_msgs::JointTrajectoryPoint GetKeyboardTeleopCommand(void);
    
  private:
    trajectory_msgs::JointTrajectoryPoint trajectory_;        //!< Trajectory from keyboard input
    double _key_gain;                                         //!< Multiplicative factor
    
};
} // teleop namespace
} // mav_planner namespace

#end
