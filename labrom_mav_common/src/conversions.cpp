#include "labrom_mav_common/conversions.h"

// ROS libraries
#include <nav_msgs/Odometry.h>
#include <cmath>

labrom_mav_common::State ConvertFromROSMsg(const nav_msgs::Odometry::ConstPtr &msg)
{
  labrom_mav_common::State state;
  
  // Copyng values to labrom state data structure
  state.pose.position.x = msg->pose.pose.position.x;
  state.pose.position.y = msg->pose.pose.position.y;
  state.pose.position.z = msg->pose.pose.position.z;  
  
  state.pose.qt.x       = msg->pose.pose.orientation.x;
  state.pose.qt.y       = msg->pose.pose.orientation.y;  
  state.pose.qt.z       = msg->pose.pose.orientation.z;
  state.pose.qt.w       = msg->pose.pose.orientation.w;    
  
  state.pose.euler      =  Quaternion2Euler(state.pose.qt);
  
  state.velocity.linear.x = msg->twist.twist.linear.x;
  state.velocity.linear.y = msg->twist.twist.linear.y;
  state.velocity.linear.z = msg->twist.twist.linear.z;  
  state.velocity.angular.x = msg->twist.twist.angular.x;
  state.velocity.angular.y = msg->twist.twist.angular.y;  
  state.velocity.angular.z = msg->twist.twist.angular.z;
  
  return state;
}

labrom_mav_common::Euler Quaternion2Euler(const labrom_mav_common::Quaternion &qt){
  labrom_mav_common::Euler euler;
  
  euler.roll  = atan2(2*(qt.w*qt.x + qt.y*qt.z), 1 - 2*(qt.x*qt.x + qt.y*qt.y) );
  euler.pitch = asin(2*(qt.w*qt.y - qt.z*qt.x) );
  euler.yaw   = atan2(2*(qt.w*qt.z + qt.x*qt.y), 1 - 2*(qt.y*qt.y + qt.z*qt.z) );
  
  return euler;      
}

Eigen::Vector3d Labrom2Eigen(const labrom_mav_common::Vector3 &vec)
{
  return Eigen::Vector3d(vec.x,vec.y,vec.z);
}
