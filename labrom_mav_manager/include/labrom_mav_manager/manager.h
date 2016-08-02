/*************************************************************************
*   Manager header files
*   This file is part of labrom_mav_manager
*
*   labrom_mav_manager is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_mav_manager is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_mav_manager.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

#ifndef LABROM_MAV_MANAGER_H_
#define LABROM_MAV_MANAGER_H_

// top-level namespace
namespace manager{

enum ManagerState{IDLE=0, TURN_MOTORS_ON, TAKE_OFF, WAIT_TAKE_OFF, FREE_MODE, LAND, TURN_MOTORS_OFF};

//! ROS spin function 
void Spin();

} // manager namespace
#endif // LABROM_MAV_MANAGER_H_