/*************************************************************************
*   Blind::LandingServer header files
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

#ifndef BLIND_LANDING_SERVER_H_
#define BLIND_LANDING_SERVER_H_

// landing libraires
#include "blind_action/blind.h"

// laberom_control libraries
#include "labrom_control/controllers.h"

// Actionlib libraries
#include <actionlib/server/simple_action_server.h>
#include <blind_action/LandingAction.h>

// ROS message libraries
#include "sensor_msgs/Imu.h"

namespace blind{
//! Landing namespace
namespace landing{
//! Class LandingServer scope
class LandingServer{
  public:
    //! Constructor
    LandingServer(controllers::Controller &controller);
    //! Empty Destructor
    ~LandingServer(void);
    //! Imu callback
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &imu);
    //! Goal Callback
    void GoalCallback(const blind_action::LandingGoalConstPtr &goal);
    //! Preemption Callback
    void PreemptCallback(void);
    //! Set thrust command parameters
    void SetParams(int feedforward=0, int max_thrust=10, int loop_rate=10);
  
  private:
    ros::NodeHandle nh_;                //!< ROS node handle
    // ROS publishers and subscribers
    ros::Subscriber imu_sub_;           //!< ROS IMU subscriber
    ros::Publisher thrust_pub_;         //!< ROS Thrust publisher
    ros::Publisher attitude_pub_;       //!< ROS Attitude(RPY) publisher          
    // Landing blind action server (as)
    actionlib::SimpleActionServer<blind_action::LandingAction> as_;          //!< Action server
     // Landing blind messages defined within actionlib
    blind_action::LandingFeedback feedback_;  //!< Feedback message
    blind_action::LandingResult result_;      //!< Result message
    blind_action::LandingGoal goal_;          //!< Goal message
    // Landing rotine variables
    controllers::Controller* controller_; //!< Controller specified by user
    int start_time_;                      //!< Time current goal was received
    int loop_rate_;                       //!< Actuation loop rate
    int feedforward_;                     //!< Feedforward thrust 0..100
    int max_thrust_;                      //!< Maximum thrust 0..100    
    bool is_reading_imu_;                 //!< Indicates imu data reception


};

} // landing namespace
} // blind namespace

#endif //BLIND_LANDING_SERVER_H_
