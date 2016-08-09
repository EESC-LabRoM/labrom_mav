/*************************************************************************
*   blind::take_off::Server header files
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

// Actionlib libraries
#include <actionlib/server/simple_action_server.h>
#include <labrom_mav_manager/TakeOffAction.h>

// ROS message libraries
#include "sensor_msgs/Imu.h"

namespace blind{
//! Take off namespace
namespace take_off{
//! Possible states for take off action
enum State{IDLE=0,  TRYING_TO_TAKE_OFF,  CLIMB, FINISHED};
//! Class BlindTakeOffServer scope
class Server{
  public:
    //! Constructor
    Server(std::string name);
    //! Empty Destructor
    ~Server(void);
    //! Imu callback
    void ImuCallback(const sensor_msgs::Imu::ConstPtr &imu);
    //! Goal Callback
    void GoalCallback(const labrom_mav_manager::TakeOffGoalConstPtr &goal);
    //! Preemption Callback
    void PreemptCallback(void);
    //! Set thrust command parameters
    void SetParams(int feedforward=0, int max_thrust=10, int loop_rate=10);
  
  private:
    ros::NodeHandle nh_;                //!< ROS node handle
    // ROS publishers and subscribers
    ros::Subscriber imu_sub_;           //!< ROS IMU subscriber     
    // Take off blind action server (as)
    actionlib::SimpleActionServer<labrom_mav_manager::TakeOffAction> as_;          //!< Action server
     // Take off blind messages defined within actionlib
    labrom_mav_manager::TakeOffFeedback feedback_;  //!< Feedback message
    labrom_mav_manager::TakeOffResult result_;      //!< Result message
    labrom_mav_manager::TakeOffGoal goal_;          //!< Goal message
    // Take off rotine variables
    int loop_rate_;                       //!< Actuation loop rate
    int feedforward_;                     //!< Feedforward thrust 0..100
    int max_thrust_;                      //!< Maximum thrust 0..100    
    double previous_time_;                //!< memory for obtain time between imu reading
    State state_;                         //!< Indicates action state
};

} // take_off namespace
} // blind namespace

#endif //BLIND_TAKE_OFF_SERVER_H_
