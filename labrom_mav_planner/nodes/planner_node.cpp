/*************************************************************************
*   Planner node implementation
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

// labrom_mav_planner libraries
#include "planner_node.h"

namespace mav_planner{

/**
* Constructor
*/
Planner::Planner(void): pnh_("~"){
  trajectory_msgs::JointTrajectoryPoint trajPoint; 
  for(int i=0; i<4; ++i){
    trajPoint.positions.push_back(0);
    trajPoint.velocities.push_back(0);
    trajPoint.accelerations.push_back(0);
    trajPoint.effort.push_back(0);
  };
  trajectory_.points.push_back(trajPoint);

  // ROS publishers and subscribers
  keyboard_pressed_sub_ = nh_.subscribe("key_down",1,&Planner::KeyboardPressedCallback,this);
  keyboard_unpressed_sub_ = nh_.subscribe("key_up",1,&Planner::KeyboardUnpressedCallback,this);
  traj_pub_     = nh_.advertise<trajectory_msgs::JointTrajectory>("trajectory",1);  
}

/**
* Destructor
*/
Planner::~Planner(void){};

/**
* Keyboard pressed callback
* param[in] msg key pressed
*/
void Planner::KeyboardPressedCallback(const keyboard::Key::ConstPtr &msg){
  int key = msg->code;
  key_teleop.KeyPressed(key);
  PublishTrajectory();
}

/**
* Keyboard unpressed callback
* param[in] msg key unpressed
*/
void Planner::KeyboardUnpressedCallback(const keyboard::Key::ConstPtr &msg){
  int key = msg->code;
  key_teleop.KeyUnpressed(key);
  PublishTrajectory();
}

/**
* Trajectory publisher
*/
void Planner::PublishTrajectory(void){
  trajectory_.points[0] = key_teleop.GetTrajectory();
  traj_pub_.publish(trajectory_);
}

}// mav_planner namespace

int main(int argc, char **argv){
  ros::init(argc,argv,"Planner");

  // Instatiate planner
  mav_planner::Planner planner;

  ros::spin();
}