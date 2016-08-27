/*************************************************************************
*   blind::landing::Client implementation
*   This file is part of blind_action
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
// labrom_mav_manager libraries
#include "labrom_mav_manager/landing_client.h"

namespace blind{
namespace landing{
/**
* Constructor
*/
Client::Client(std::string name): ac_(name, true){
  // Wait for blind landing server 
  ac_.waitForServer();
};

/**
* Destructor
*/
Client::~Client(void){};

/**
* Send order
*/

void Client::SendGoal(double descend_accel, int hit_ground_accel){
  // Cancel all goals
  CancelAllGoals();
  // Assemble message
  labrom_mav_manager::LandingGoal goal;
  goal.descend_accel = descend_accel;
  goal.hit_ground_accel = hit_ground_accel;
  // Deactive success flags
  success_flag_ = false;
  // Send goal via actionlib client
  ac_.sendGoal(goal, boost::bind(&Client::DoneCallback, this, _1, _2),
                    NULL,
                    boost::bind(&Client::FeedbackCallback, this, _1) );              
}

/**
* Done Callback
* Called when the goal completes (triggered by action server)
*/
void Client::DoneCallback(const actionlib::SimpleClientGoalState& state, const labrom_mav_manager::LandingResult::ConstPtr& result){
  // Save results
  thrust_ = result->thrust; 
  // Setting success flag
  success_flag_ = true;
    
};
    
/**
* Feedback Callback
* Called when the goal is active and action client receives a feedback from action server (triggered by action server)
*/
void Client::FeedbackCallback(const labrom_mav_manager::LandingFeedback::ConstPtr& feedback){
  // Save current thrust
  thrust_ = feedback->thrust;
};

/** 
* Cancel all goal (bypass function)
*/
void Client::CancelAllGoals(void){
  ac_.cancelAllGoals();
}

/**
* Get last thrust value received by action client
* @return double corresponding to current thrust
*/
double Client::GetThrust(void){
  return thrust_;
}

/**
* Get success flag
* Use this function to check if current goal has finished.
* @return bool set true when current goal is done
*/
bool Client::IsDone(void){
  return success_flag_;
}

} // landing namespace
} // blind namespace