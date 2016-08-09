/*************************************************************************
*   Blind header file for namespace definition
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

#ifndef BLIND_H_
#define BLIND_H_


// labrom_mav_manager libraries
#include "labrom_mav_manager/take_off_server.h"
#include "labrom_mav_manager/take_off_client.h"

#include "labrom_mav_manager/landing_server.h"
#include "labrom_mav_manager/landing_client.h"

#include <actionlib/server/simple_action_server.h>

// actionlib libraries
#include <actionlib/client/simple_action_client.h>

//! Top-level namespace
namespace blind{
} // blind namespace

#endif //BLIND_H_   
