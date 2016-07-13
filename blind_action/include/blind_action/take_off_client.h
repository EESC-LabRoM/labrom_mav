/*************************************************************************
*   Blind::TakeOffClient header files
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

#ifndef BLIND_TAKE_OFF_CLIENT_H_
#define BLIND_TAKE_OFF_CLIENT_H_

// take_off libraries
#include <blind_action/blind.h>
#include <blind_action/TakeOffAction.h>

// Actionlib libraries
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

namespace blind{
namespace take_off{
//! Class TakeOffClient scope
class TakeOffClient{
  public:
    //! Constructor
    TakeOffClient(void);
    //! Empty Destructor 
    ~TakeOffClient(void);
    //! Set Goal
    void SetGoal(double take_off_accel = 10.0);
    //! Send Goal
    bool SendGoal(double timeout = 2.0);
    
  private:
    blind_action::TakeOffGoal goal_;           //!< Goal
};

} // take_off namespace
} // blind namespace

#endif //BLIND_TAKE_OFF_CLIENT_H_
