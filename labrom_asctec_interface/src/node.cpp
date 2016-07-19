#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <asctec_hl_comm/mav_ctrl.h>

geometry_msgs::Vector3 attitude;
std_msgs::Int32 thrust;
asctec_hl_comm::mav_ctrl msg_mav_ctrl;

void thrustCallback(const std_msgs::Int32::ConstPtr &msg) {
  thrust = *msg;
}

void attitudeCallback(const geometry_msgs::Vector3::ConstPtr &msg)
{
  attitude = *msg;
}

int main(int argc, char **argv)
{
  // Start ROS within this node
  ros::init(argc,argv,"node");
  
  // Start ROS Node Handle
  ros::NodeHandle node;
  
  // Loop rate
	ros::Rate loop_rate(60);
  
  // Start subscribers
  ros::Subscriber sub_thrust = node.subscribe("/cmd_thrust", 1, thrustCallback);
  ros::Subscriber sub_attitude = node.subscribe("/cmd_attitude", 1, attitudeCallback);
  
  // Start publishers
  ros::Publisher pub_mav_ctrl = node.advertise<asctec_hl_comm::mav_ctrl>("/mav_ctrl", 1);

  while(ros::ok()) {
    // Publishers
    /*
    int8 acceleration = 1
    int8 velocity = 2
    int8 position = 3
    int8 velocity_body = 4
    int8 position_body = 5
    */

    msg_mav_ctrl.type = 1;
    /*
    Currently x~pitch, y~roll, z~thrust, units in rad and rad/s for yaw
    */
    // pitch
    msg_mav_ctrl.x = attitude.x;
    // roll
    msg_mav_ctrl.y = attitude.y;
    /*
    thrust - 0 ... 100
    mav_ctrl.z - 0 ... 1
    */
    msg_mav_ctrl.z = ((float) thrust.data) / 100;
    // yaw
    msg_mav_ctrl.yaw = attitude.z;
    pub_mav_ctrl.publish(msg_mav_ctrl);
    
    // ROS routines
    ros::spinOnce();
		loop_rate.sleep();
  }

}
