// ROS libraries
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

//labrom_mav_common
#include <labrom_mav_common/conversions.h>
#include <labrom_mav_common/MavCmd.h>
#include <labrom_mav_common/TeleOpCmd.h>

//labrom_mav_common
#include <labrom_mav_control/PoseLinear.h>

// eigen_conversion
#include <eigen_conversions/eigen_msg.h>

// std libraries
#include<string>

namespace labrom_mav_manager{
enum EnumState{ EMERGENCY=0, HARD_LAND, REST, TAKE_OFF, SOFT_LAND, HOVER, GO2POINT, TRACK, VELOCITY_CTRL };
enum EnumTransitionCritical{NO_CRITICAL=0, GO_EMERGENCY, GO_HARD_LAND, DO_CRITICAL_EMERGENCY, DO_CRITICAL_HARD_LAND};
enum EnumTransitionAction{NO_ACTION=0, RESTART, DO_TAKE_OFF, DO_SOFT_LAND, DO_HOVER, DO_GO2POINT, DO_TRACK, DO_VEL_CTRL };
//! State machine that controls basic functionalities
class StateMachine{
public:
  /**
  * Constructor
  * @param[in] name - string identifier */
  StateMachine(std::string name);
  
  /** Destructor */
  ~StateMachine();
  
  /** Odometry callback odom. SAve the current odometry in the labrom data format
  * @param[in] Odometry message received from ROS
  */
  void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

  /** Imu callback. 
  * @param[in] Imu message received from ROS
  */
  void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);

  /** Imu callback. 
  * @param[in] Imu message received from ROS
  */
  void TwistCallback(const geometry_msgs::Twist::ConstPtr& msg);

  /** TeleOp callback. 
  * @param[in] Odometry message received from ROS
  */
  void TeleOpCallback(const labrom_mav_common::TeleOpCmd::ConstPtr& msg);
 
  /** Continuously check state estimation quality */
  void CheckStateEstimation(const ros::TimerEvent& e);
 
  /** Continuously check if has communication */
  void CheckCommunication();
 
  /** State machine iteration */
  void Run(const ros::TimerEvent& e);
  
  /** Continuous loop 
  * @param[in] frequency - state machine rate */
  void Loop(double frequency);
  
private:
  ros::NodeHandle m_nh;       //!< global ros node handle
  ros::NodeHandle m_pnh;      //!< private ros node handle
  
  ros::Subscriber m_odom_sub; //!< odometry subscriber
  ros::Subscriber m_imu_sub;  //!< imu subscriber
  ros::Subscriber m_joy_sub;  //!< odometry subscriber
  ros::Subscriber m_goal_twist_sub;  //!< goal twist subscriber    
  ros::Publisher m_cmd_pub;   //!< quad command publisher
  
  EnumTransitionAction m_action;
  EnumTransitionCritical m_critical;

  labrom_mav_common::State m_robot_state;    //!< robot state
  labrom_mav_common::State m_des_state;      //!< desired state
  labrom_mav_common::MavCommand m_mav_cmd;   //!< command to quadrotor
  labrom_mav_common::MavParams m_mav_params; //!< quadrotor parameters
  labrom_mav_common::Vector3 m_pose_uncertainty;
  labrom_mav_common::Velocity m_goal_velocity;  
  
  double m_timer;
  double m_time_rcv_odom;
  double m_time_rcv_imu;
  double m_max_z_accel, m_min_z_accel;
  EnumState m_curr_state;                  //!< current state of the state machine

  // Failure flags
  bool m_flag_has_state_estimation;
  bool m_flag_take_off_timeout;
  bool m_flag_lost_track;
  bool m_flag_data_link;
  // Success flags
  bool m_flag_take_off_success;
  bool m_flag_reach_destination;
  bool m_flag_land_detected;
  // Active flag
  bool m_flag_active;
  bool m_land_descend;
  
  // Outer loop controllers
  labrom_mav_control::PoseLinear m_pose_controller;
  
  // User defined parameters
  std::string m_name;          //!< name identifier  
  double m_take_off_timeout;
  double m_landing_accel;
  double m_loop_rate;

    
};

} // labrom_mav_manager namespace
