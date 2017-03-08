#include "labrom_mav_manager/state_machine.h"

namespace labrom_mav_manager{
/* Constructor */
StateMachine::StateMachine(std::string name): m_pnh("~"),
                                              m_name(name),
                                              m_time_rcv_odom(0),
                                              m_time_rcv_imu(0)
{
  // Subscribing and advertising topics
  m_odom_sub = m_nh.subscribe("odometry", 1, &StateMachine::OdometryCallback, this);
  m_joy_sub  = m_nh.subscribe("teleop_cmd", 1, &StateMachine::TeleOpCallback, this);
  m_imu_sub  = m_nh.subscribe("imu", 1, &StateMachine::ImuCallback, this);
  m_cmd_pub  = m_nh.advertise<labrom_mav_common::MavCmd>("mav_cmd",1);
  m_goal_twist_sub = m_nh.subscribe("cmd_vel", 1, &StateMachine::TwistCallback, this);
  // Get parameters 
  std::string filename;
  m_pnh.param("mass", m_mav_params.mass, 0.0);
  m_pnh.param("take_off_timeout", m_take_off_timeout, 5.0);
  m_pnh.param("landing_accel", m_landing_accel, 12.0);    
  m_pnh.param("loop_rate", m_loop_rate, 50.0);
  m_pnh.getParam("control_params",filename );   
  // Initializing
  ///flags
  m_curr_state = REST;
  m_action   = NO_ACTION;
  m_critical = NO_CRITICAL;
  m_flag_active = false;
  /// controllers
  m_pose_controller = labrom_mav_control::PoseLinear( m_mav_params, filename,"");
  
}
  
/* Destructor */
StateMachine::~StateMachine(){;}

/* Odometry callback */
void StateMachine::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
  // Copyng values to labrom state data structure
  m_robot_state = ConvertFromROSMsg(msg);  
  m_pose_uncertainty = labrom_mav_common::Vector3(msg->pose.covariance[0], msg->pose.covariance[7], msg->pose.covariance[14]);
  m_time_rcv_odom = ros::Time::now().toSec();
}

/* Imu callback */
void StateMachine::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  //### Check if acceleration is the minimum/maximum acc registered ### //
  if ( msg->linear_acceleration.z < m_min_z_accel )
    m_min_z_accel =  msg->linear_acceleration.z;
  else if ( msg->linear_acceleration.z > m_max_z_accel )
    m_max_z_accel =  msg->linear_acceleration.z;
  
  m_time_rcv_imu = ros::Time::now().toSec();
}


/* Goal twist callback */
void StateMachine::TwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  m_goal_velocity.linear.x = msg->linear.x;
  m_goal_velocity.linear.y = msg->linear.y;
  m_goal_velocity.linear.z = msg->linear.z; 
  m_goal_velocity.angular.z = msg->angular.z;      
}


/* Continuously check state estimation quality */
void StateMachine::CheckStateEstimation(const ros::TimerEvent& e)
{
  // State estimation verification
  bool valid_uncertainty = (m_pose_uncertainty.x < 1) && (m_pose_uncertainty.y < 1) && (m_pose_uncertainty.z < 1); 
  bool valid_height = (m_robot_state.pose.position.z - m_pose_uncertainty.z) > 0;
  bool state_estimation_available = (ros::Time::now().toSec() - m_time_rcv_odom) < 0.1;
  m_flag_has_state_estimation = state_estimation_available && valid_uncertainty && valid_height;

  // Data link
  m_flag_active = ( (ros::Time::now().toSec() - m_time_rcv_odom) < 2);
};
 

/* TeleOp callback */
void StateMachine::TeleOpCallback(const labrom_mav_common::TeleOpCmd::ConstPtr& msg)
{
  if (msg->emergency)
    m_critical = DO_CRITICAL_EMERGENCY;
  else if (msg->hard_land)
    m_critical = DO_CRITICAL_HARD_LAND;
  else if (msg->take_off)
    m_action = DO_TAKE_OFF;
  else if (msg->soft_land)
    m_action = DO_SOFT_LAND;
  else if (msg->restart)
    m_action = RESTART;
  else if (msg->hover)
    m_action = DO_HOVER;
  else if (msg->go2point)
    m_action = DO_GO2POINT;
  else if (msg->track)
    m_action = DO_TRACK; 
} 

/* A state machine iteration  */
void StateMachine::Run(const ros::TimerEvent& e){
  double dt = e.current_real.toSec() - e.last_real.toSec();
    
  // Wait for communication (receive imu data stream)
  if (!m_flag_active){
    // Cleaning flags
    m_action = NO_ACTION;
    m_critical = NO_CRITICAL;  
    return;
  }
  
    
  // ### Critical Action (High priority!!!)### //
  if( m_curr_state != EMERGENCY ){
    if (m_critical == DO_CRITICAL_EMERGENCY){
      ROS_WARN("Emergency state activated");
      m_curr_state = EMERGENCY;
      m_critical = NO_CRITICAL;
    }else if ((m_critical == DO_CRITICAL_HARD_LAND ||  !m_flag_has_state_estimation)  &&
              (m_curr_state > REST ) ){
      ROS_WARN("Hard land state activated");
      m_curr_state = HARD_LAND;
      m_critical = NO_CRITICAL;    
        m_min_z_accel = 9.81;
        m_max_z_accel = 9.81;
            
    }
  }
    
  // State machine
  switch(m_curr_state)
  {     
    // Nothing to do..
    case(REST):
    {
      // motors off 
      m_mav_cmd.thrust = 0;
      m_mav_cmd.euler.roll   = 0;
      m_mav_cmd.euler.pitch  = 0;
      m_mav_cmd.angular_vel.z= 0; 
      
      // ### Check transitions ### //
      if (m_action==DO_TAKE_OFF) {
        ROS_INFO("Take off requested!");
        m_pose_controller.Reset();
        m_mav_cmd.thrust = 0.6*m_mav_params.mass*9.81;
        m_timer = 0;
        m_max_z_accel = 9.81;
        m_min_z_accel = 9.81;
        m_curr_state = TAKE_OFF;
        
      }
    }
    break;
    
    // 
    case(TAKE_OFF):
    {
      // motors on and increasing
      m_mav_cmd.thrust += 0.3*m_mav_params.mass*9.81*dt;
      m_mav_cmd.euler.roll    = 0;
      m_mav_cmd.euler.pitch   = 0;
      m_mav_cmd.angular_vel.z = 0; 
      
      m_timer += dt; 

      // Update automatic transitions
      m_flag_take_off_timeout = m_timer > m_take_off_timeout;
      m_flag_take_off_success = m_flag_has_state_estimation && m_robot_state.pose.position.z > 0.1;
            
      if(m_flag_take_off_timeout){
        ROS_WARN("Take off timeout (%f secs).", m_take_off_timeout);
        ROS_INFO("Automatically transitioning to hard land state.");
        m_curr_state = HARD_LAND;
        
      } else if (m_flag_take_off_success){
        m_des_state.pose.position = labrom_mav_common::Vector3(
                                             m_robot_state.pose.position.x, m_robot_state.pose.position.y,0.8); 
        ROS_INFO("Successful take off! Good state estimation available.");
        ROS_INFO("Automatically transitioning to hovering state.");
        m_pose_controller.Reset();
        m_curr_state = HOVER;  
      }
    }
    break;
    
    case(HOVER):
    {
      
      m_mav_cmd = m_pose_controller.Run(m_robot_state, m_des_state, dt);


      if (m_action==DO_SOFT_LAND) {
        ROS_INFO("Soft land requested!");
        m_land_descend = false;
        m_curr_state = SOFT_LAND;
      } else if (m_action==DO_GO2POINT) {
        ROS_INFO("GO2POINT requested!");
        m_curr_state = GO2POINT;
      } else if (m_action==DO_TRACK) {
        ROS_INFO("Tracking requested!");
        m_curr_state = TRACK;
      }

    }
    break;

    case(GO2POINT):
    {
      // Pose controller
      static int wp = 0;
      switch (wp){
        case (0): m_des_state.pose.position = labrom_mav_common::Vector3(0.667, 1.383,0.5);
                  break;
        case (1): m_des_state.pose.position = labrom_mav_common::Vector3(-0.508, 1.362,0.5);
                  break;
        case (2): m_des_state.pose.position = labrom_mav_common::Vector3(-0.514, -0.94,0.5);
                  break;
        case (3): m_des_state.pose.position = labrom_mav_common::Vector3(0.63, -0.95,0.5);
                  break;                                      
      }
      
      double error = Eigen::Vector3d(m_des_state.pose.position.x - m_robot_state.pose.position.x, 
                                     m_des_state.pose.position.y - m_robot_state.pose.position.y,
                                     0).norm();
      
      if (error < 0.15){
        ROS_INFO("Arrived at wp #%d", wp);
        wp = (wp + 1) % 4;
      }
      

      m_mav_cmd = m_pose_controller.Run(m_robot_state, m_des_state, dt);
      
      std::cout << wp << " " << error << " ";
      std::cout << m_des_state.pose.position.x << " "<< m_des_state.pose.position.y << " "<< m_des_state.pose.position.z << " "<< m_des_state.pose.euler.yaw << " ";
      std::cout << m_robot_state.pose.position.x << " "<< m_robot_state.pose.position.y << " "<< m_robot_state.pose.position.z << " "<< m_robot_state.pose.euler.yaw << " ";
       std::cout << m_mav_cmd.euler.roll * 180 / M_PI << " " << m_mav_cmd.euler.pitch * 180 / M_PI ;   
      std::cout << std::endl;

    

      if (m_action==DO_HOVER) {
        ROS_INFO("Hover requested!");
        m_curr_state = HOVER;
      } else if (m_action==DO_SOFT_LAND) {
        ROS_INFO("Soft land requested!");
        m_curr_state = SOFT_LAND;
      }
    }
    break;
    
    case(TRACK):
    {
      // Pose controller
      m_des_state.pose.position.x = m_robot_state.pose.position.x + m_goal_velocity.linear.x;
      m_des_state.pose.position.y = m_robot_state.pose.position.y + m_goal_velocity.linear.y;

      m_des_state.pose.position.z = 0.8; //m_robot_state.pose.position.z + m_goal_velocity.linear.z;       
      std::cout << m_goal_velocity.linear.x << " " << m_goal_velocity.linear.y << std::endl;
      m_mav_cmd = m_pose_controller.Run(m_robot_state, m_des_state , dt);
      
      bool hover_flag = (abs(m_robot_state.pose.position.x) > 2.0) || (abs(m_robot_state.pose.position.y) > 2.5);
      
      if (m_action==DO_HOVER || hover_flag) {
        ROS_INFO("Hover requested!");
        m_curr_state = HOVER;
      } else if (m_action==DO_SOFT_LAND) {
        ROS_INFO("Soft land requested!");
        m_curr_state = SOFT_LAND;
      }
    }
    break;
    
    
    
    case(SOFT_LAND):
    {
      // Check if minimum landing height achieved                     
      if(m_robot_state.pose.position.z < 0.1)
      {
        ROS_INFO("Land height achieve. Automatically transitioning to hard land state!");
        m_min_z_accel = 9.81;       
        m_max_z_accel = 9.81;       
        m_curr_state = HARD_LAND;
      }                                          

      double horizontal_error = Eigen::Vector3d(m_des_state.pose.position.x - m_robot_state.pose.position.x, 
                                                m_des_state.pose.position.y - m_robot_state.pose.position.y,
                                                0.0).norm();
      
      /// Here goes the landing controller..
      //if(!m_land_descend){      // Minimize horizontal error
      //  if(horizontal_error < 0.1)
      //    m_land_descend = true;    
      //} else {                 // Minimize vertical error
      //  if(horizontal_error < m_robot_state.pose.position.z)
          m_des_state.pose.position.z = std::max(m_des_state.pose.position.z-0.2*dt,0.0);
      //} 
      m_mav_cmd = m_pose_controller.Run(m_robot_state, m_des_state , dt);
    }
    break;    
    
    case(HARD_LAND):
    {
      // motors 95% of theoretical hovering thrust 
      m_mav_cmd.thrust = (0.80 +  0.0001*(rand() % 100))*m_mav_params.mass*9.81;
      m_mav_cmd.euler.roll    = 0;
      m_mav_cmd.euler.pitch   = 0;
      m_mav_cmd.angular_vel.z = 0;  
       
       if (m_action==RESTART){
        ROS_INFO("Restart request. Ready to take off!");
        m_curr_state = REST; 
      } else if ( m_max_z_accel > 12.75){//m_landing_accel){
        ROS_INFO("Landing detected! Automatically transitioning to rest state.");
        ROS_INFO("Ready to take off!");
        m_curr_state = REST;       
      }
    }
    break;
    
    // Emergency: STOP THE MOTORS!!! Dead end state. 
    // Need to restart the machine.
    case(EMERGENCY):
    {
      m_mav_cmd.thrust = 0;
      m_mav_cmd.euler.roll    = 0;
      m_mav_cmd.euler.pitch   = 0;
      m_mav_cmd.angular_vel.z = 0;             
    }
    break;    
  }

  // Cleaning flags
  m_action = NO_ACTION;
  m_critical = NO_CRITICAL;
  
  // Send commands to quadrotor
  labrom_mav_common::MavCmd msg;
  msg.thrust = m_mav_cmd.thrust;
  msg.roll   = m_mav_cmd.euler.roll;
  msg.pitch  = m_mav_cmd.euler.pitch;
  msg.wz     = m_mav_cmd.angular_vel.z ;      
  m_cmd_pub.publish(msg); 
}
  
/* Continuous loop */
void StateMachine::Loop(double frequency){
  ros::Timer timer_run = m_pnh.createTimer(ros::Duration(1.0/m_loop_rate), &StateMachine::Run, this);
  ros::Timer timer_se = m_pnh.createTimer(ros::Duration(1.0/(0.5*m_loop_rate)), &StateMachine::CheckStateEstimation, this);  
  ros::spin();
}

} // labrom_mav_manager namespace

int main(int argc, char **argv){
  ros::init(argc,argv,"StateMachine");
  
  labrom_mav_manager::StateMachine sm("StateMachine");
  
  sm.Loop(10);
}
