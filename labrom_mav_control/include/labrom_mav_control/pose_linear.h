/*************************************************************************
*   Linear plant associate with a PID control law for position control (Header file)
*   This file is part of labrom_mav_control
*
*   labrom_mav_control is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_mav_control is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_mav_control.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

#ifndef POSITION_LINEAR_H_
#define POSITION_LINEAR_H_

//labrom_control libraries
#include<labrom_control/pid_simple.h>

// ROS libraries
#include "ros/ros.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>

// ROS message libraries
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"

//! top level namespace
namespace mav_control{
//! velocity controllers
namespace pose{
//! Linearized plants
namespace linear{
//! Linearized quadrotor controllers for position commands.
class PID{
  public:
    //! Empty constructor
    PID(void);
    //! Empty destructor
    ~PID(void);
    //! Odometry callback
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
    //! Goal (pose) callback
    void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    //! TF callback
    void TFCallback(void);
    //! Update vehicle control commands
    void ComputeActuation( const geometry_msgs::PoseStamped &pose, const geometry_msgs::TwistStamped &twist);
    //! ROS loop
    void Spin(void);

  private:     
    ros::NodeHandle nh_;                //! node handle 
    ros::NodeHandle pnh_;               //! private node handle

    ros::Publisher attitude_pub_;       //!< ROS attitude publisher
    ros::Publisher thrust_pub_;         //!< ROS thrust publisher
    ros::Subscriber odom_sub_;          //!< ROS odometry subscriber
    ros::Subscriber goal_sub_;         //!< ROS goal pose subscriber

    tf::TransformListener listener_;     //!< TF transformer listener

    geometry_msgs::Pose desired_pose_;    //!< Last command pose received (desired)

    controllers::pid::Simple pid_ddx_;      //! body frame velocity x-axis controller
    controllers::pid::Simple pid_ddy_;      //! body frame velocity y-axis controller
    controllers::pid::Simple pid_ddz_;      //! body frame velocity z-axis controller
    
    struct{
      double mass;
      double gravity;
    } params_;

    bool use_tf_;                   //! set true to use TF or false to use Odometry
    int loop_rate_;                 //! Control loop rate (TF usage only)
    std::string world_frame_;       //! world frame (AKA fixed frame)
    std::string body_frame_;        //! body frame (AKA control frame)
};
} // linear namespace
} // pose namespace
} // mav_control namespace
#endif