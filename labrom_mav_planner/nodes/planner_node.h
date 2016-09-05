/*************************************************************************
*   Planner node header files
*   This file is part of labrom_mav_planner
*
*   labrom_mav_planner is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_mav_planner is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_mav_planner.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

#ifndef PLANNER_NODE_H_
#define PLANNER_NODE_H_

// labrom_mav_planner libraries
#include "labrom_mav_planner/keyboard_teleop.h"
// ROS libraries
#include "ros/ros.h"
// ROS message libraries
#include "trajectory_msgs/JointTrajectory.h"
#include "keyboard/Key.h"
//! top-level namespace
namespace mav_planner{
class Planner{
  public:
    //! Constructor
    Planner(void);
    //! Destructor
    ~Planner(void);
    //! Publisher
    void PublishTrajectory(void);
    //! Keyboard pressed callback
    void KeyboardPressedCallback(const keyboard::Key::ConstPtr &msg);
   //! Keyboard unpressed callback
    void KeyboardUnpressedCallback(const keyboard::Key::ConstPtr &msg);

  private:
    ros::NodeHandle nh_;                                //!< Global node handle
    ros::NodeHandle pnh_;                               //!< Private node handle
    ros::Publisher traj_pub_;                           //!< Trajectory publisher
    ros::Subscriber keyboard_pressed_sub_;              //!< Keyboard pressed Subscriber
    ros::Subscriber keyboard_unpressed_sub_;            //!< Keyboard unpressed subscriber
    trajectory_msgs::JointTrajectory trajectory_;       //!< trajectory message

    teleop::Keyboard key_teleop;                                //!< Keyboard teleop
};
}// mav_planner namespace

#endif
