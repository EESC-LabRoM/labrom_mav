/*************************************************************************
*   Blind::TakeOffServer header files
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

#ifndef BLIND_TAKE_OFF_SERVER_H_
#define BLIND_TAKE_OFF_SERVER_H_

// laberom_control libraries
#include "labrom_control/controllers.h"

// Actionlib libraries
#include <actionlib/server/simple_action_server.h>
#include <blind_action/TakeOffAction.h>

// ROS message libraries
#include "sensor_msgs/Imu.h"

namespace blind{
//! Take off namespace
namespace take_off{
//! Possible states for take off action
enum State{IDLE=0, ACTIVE, TAKE_OFF_DETECTED, CLIMB, FINISHED};
//! Class BlindTakeOffServer scope
class Server{
  public:
    //! Constructor
    Server(void);
    //! Empty Destructor
    ~Server(void);
    //! Imu callback
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &imu);
    //! Goal Callback
    void GoalCallback(const blind_action::TakeOffGoalConstPtr &goal);
    //! Preemption Callback
    void PreemptCallback(void);
    //! Set thrust command parameters
    void SetParams(int feedforward=0, int max_thrust=10, int loop_rate=10);
  
  private:
    ros::NodeHandle nh_;                //!< ROS node handle
    // ROS publishers and subscribers
    ros::Subscriber imu_sub_;           //!< ROS IMU subscriber     
    // Take off blind action server (as)
    actionlib::SimpleActionServer<blind_action::TakeOffAction> as_;          //!< Action server
     // Take off blind messages defined within actionlib
    blind_action::TakeOffFeedback feedback_;  //!< Feedback message
    blind_action::TakeOffResult result_;      //!< Result message
    blind_action::TakeOffGoal goal_;          //!< Goal message
    // Take off rotine variables
    int loop_rate_;                       //!< Actuation loop rate
    int feedforward_;                     //!< Feedforward thrust 0..100
    int max_thrust_;                      //!< Maximum thrust 0..100    
    State state_;                         //!< Indicates action state
};

} // take_off namespace
} // blind namespace

#endif //BLIND_TAKE_OFF_SERVER_H_
