/*************************************************************************
*   Blind header file for namespace definition
*   This file is part of blind_action
*
*   blind_action is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   blind_action is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with blind_action.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

#ifndef BLIND_H_
#define BLIND_H_


// labrom_mav_blind_action libraries
#include "labrom_mav_blind_action/take_off_server.h"
#include "labrom_mav_blind_action/take_off_client.h"

#include "labrom_mav_blind_action/landing_server.h"
#include "labrom_mav_blind_action/landing_client.h"

#include <actionlib/server/simple_action_server.h>

// actionlib libraries
#include <actionlib/client/simple_action_client.h>

//! Top-level namespace
namespace blind{
} // blind namespace

#endif //BLIND_H_   
