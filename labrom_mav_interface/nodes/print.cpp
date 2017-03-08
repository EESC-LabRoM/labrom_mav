#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

void odomCallbackRef(const nav_msgs::Odometry::ConstPtr &msg)
{
  geometry_msgs::Point position = msg->pose.pose.position;
  geometry_msgs::Quaternion qt = msg->pose.pose.orientation;
  double yaw   = atan2(2*(qt.w*qt.z + qt.x*qt.y), 1 - 2*(qt.y*qt.y + qt.z*qt.z) );
  std::cout << "0 "; 
  std::cout << position.x << " " << position.y << " " << position.z << " " << yaw << " ";
  std::cout << msg->pose.covariance[0] << " " << msg->pose.covariance[7] << " " << msg->pose.covariance[14] << " ";  
  std::cout << std::endl;

};

void odomCallbackEst(const nav_msgs::Odometry::ConstPtr &msg)
{
  geometry_msgs::Point position = msg->pose.pose.position;
  geometry_msgs::Quaternion qt = msg->pose.pose.orientation;
  double yaw   = atan2(2*(qt.w*qt.z + qt.x*qt.y), 1 - 2*(qt.y*qt.y + qt.z*qt.z) );
  std::cout << "1 "; 
  std::cout << position.x << " " << position.y << " " << position.z << " " << yaw << " ";
  std::cout << msg->pose.covariance[0] << " " << msg->pose.covariance[7] << " " << msg->pose.covariance[14] << " ";  
  std::cout << std::endl;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "debug");

  ros::NodeHandle nh;

  ros::Subscriber odom_ref = nh.subscribe("/crazyflie/odometry", 1, odomCallbackRef);
  ros::Subscriber odom_est = nh.subscribe("/crazyflie/ekf/odometry", 1, odomCallbackEst);
  ros::spin();
  
  return 0;
  
}
