/*************************************************************************
*   Blind::ClimbClient header files
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

#ifndef BLIND_CLIMB_CLIENT_H_
#define BLIND_CLIMB_CLIENT_H_

// climb libraries
#include <blind_action/blind.h>
#include <blind_action/ClimbAction.h>

// Actionlib libraries
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

namespace blind{
namespace climb{
//! Class ClimbClient scope
class ClimbClient{
  public:
    //! Constructor
    ClimbClient(void);
    //! Empty Destructor 
    ~ClimbClient(void);
    //! Set Goal
    void SetGoal(double climb_accel = 10.0, double timeout = 1);
    //! Send Goal
    bool SendGoal(double timeout = 2.0);
    //! Get result thrust
    int getResultThrust(void); 
  private:
    blind_action::ClimbGoal goal_;           //!< Goal
    blind_action::ClimbResult result_;     //!< Result
};

} // climb namespace
} // blind namespace

#endif //BLIND_CLIMB_CLIENT_H_
