/*************************************************************************
*   blind::landing::Client header files
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

#ifndef BLIND_LANDING_CLIENT_H_
#define BLIND_LANDING_CLIENT_H_

// Actionlib libraries
#include <actionlib/client/simple_action_client.h>
#include <labrom_mav_manager/LandingAction.h>


namespace blind{
//! Take off namespace
namespace landing{
class Client{
  public:
    //! Constructor
    Client(std::string name);
    //! Destructor
    ~Client(void);
    //! Send order
    void SendGoal(double take_off_accel, int climb_time);
    //! Called once when the goal completes
    void DoneCallback(const actionlib::SimpleClientGoalState& state, const labrom_mav_manager::LandingResult::ConstPtr& result);
    //! Feedback
    void FeedbackCallback(const labrom_mav_manager::LandingFeedback::ConstPtr& feedback);
    //! Cancel all goals
    void CancelAllGoals(void);
    //! Get thrust value
    double GetThrust(void);
    //! GEt success flag
    bool IsDone(void);

  private:
    bool success_flag_;                                                              //! Indicates action success
    double thrust_;                                                                  //! Last thrust received from server
    actionlib::SimpleActionClient<labrom_mav_manager::LandingAction> ac_;       //! Actionlib client
};
} // take_off namespace 
} // blind namespace

#endif