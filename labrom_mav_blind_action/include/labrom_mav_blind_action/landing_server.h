/*************************************************************************
*   Blind::LandingServer header files
*   This file is part of labrom_mav_blind_action
*
*   labrom_mav_blind_action is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_mav_blind_action is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_mav_blind_action.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

#ifndef BLIND_LANDING_SERVER_H_
#define BLIND_LANDING_SERVER_H_

// Actionlib libraries
#include <actionlib/server/simple_action_server.h>
#include <labrom_mav_blind_action/LandingAction.h>

// ROS message libraries
#include "sensor_msgs/Imu.h"

namespace blind{
//! Landing namespace
namespace landing{
//! Possible states for take off action
enum State{IDLE=0, DESCEND, WAIT_HIT_GROUND, FINISHED};
//! Class  Blind Landing Server scope
class Server{
  public:
    //! Constructor
    Server(std::string name);
    //! Empty Destructor
    ~Server(void);
    //! Imu callback
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &imu);
    //! Goal Callback
    void GoalCallback(const labrom_mav_blind_action::LandingGoalConstPtr &goal);
    //! Preemption Callback
    void PreemptCallback(void);
    //! Set thrust command parameters
    void SetParams(int feedforward=0, int max_thrust=10, int loop_rate=10);
  
  private:
    ros::NodeHandle nh_;                //!< ROS node handle
    // ROS publishers and subscribers
    ros::Subscriber imu_sub_;           //!< ROS IMU subscriber
    // Landing blind action server (as)
    actionlib::SimpleActionServer<labrom_mav_blind_action::LandingAction> as_;          //!< Action server
     // Landing blind messages defined within actionlib
    labrom_mav_blind_action::LandingFeedback feedback_;  //!< Feedback message
    labrom_mav_blind_action::LandingResult result_;      //!< Result message
    labrom_mav_blind_action::LandingGoal goal_;          //!< Goal message
    // Landing rotine variables
    int loop_rate_;                       //!< Actuation loop rate
    int feedforward_;                     //!< Feedforward thrust 0..100
    int max_thrust_;                      //!< Maximum thrust 0..100    
    double previous_time_;                //!< memory for obtain time between imu reading
    State state_;                         //!< Indicates action state

};

} // landing namespace
} // blind namespace

#endif //BLIND_LANDING_SERVER_H_
