/*************************************************************************
*   This file is part of labrom_mav_control
*   Linear plant associate with a PID control law for velocity control (Implemenation)
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
double zero_time;
// labrom_mav_control libraries
#include <labrom_mav_control/velocity_linear.h>
#include <vector>
namespace mav_control{
namespace velocity{
namespace linear{
/**
* Empty constructor
*/
Controller::Controller(void): pnh_("~"){
  double fc;

  // Adversite and subscribe topics
  attitude_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("cmd_attitude",1);
  thrust_pub_   = nh_.advertise<std_msgs::Float32>("cmd_thrust",1);
  cmdvel_sub_   = nh_.subscribe("cmd_vel",1,&Controller::CmdVelCallback,this);
  
  // Parameters
  pnh_.param<std::string>("world_frame", world_frame_, "/world");
  pnh_.param<std::string>("body_frame", body_frame_, "/base_link");
  pnh_.param<int>("loop_rate", loop_rate_, 50);
  pnh_.getParam("mass", params_.mass);
  pnh_.param<bool>("use_tf", use_tf_, false);
  pnh_.param<double>("cutoff_freq", fc, -1.0);

  // Controllers parameters from configuration file
  std::vector<double> params;
  std::string coordinate[] = {"/x","/y","/z"}, gains[]={"/kp","/ki","/kd","/anti_windup"};
  for(int i=0;i<3;++i){
    for(int j=0;j<4;++j){
      params.push_back(0);
      pnh_.getParam("velocity"+coordinate[i]+gains[j], params[4*i+j]);
    }
  }

  // Loading PID controllers with configuration
  pid_ddx_ = controllers::pid::Simple("PID_ddx", params[0], params[1], params[2],  params[3]);
  pid_ddy_ = controllers::pid::Simple("PID_ddy", params[4], params[5], params[6],  params[7]);
  pid_ddz_ = controllers::pid::Simple("PID_ddz", params[8], params[9], params[10], params[11]);

  // Twist filter
  if (fc > 0)
    alpha_ = 2*M_PI*fc/(2*M_PI*fc + 1.0*loop_rate_);
  else
    alpha_ = 1;

        std::cout << "ALPHA: " << alpha_ << std::endl;
  // vehicle parameters
  params_.gravity = 9.81;

};

/**
* Empty destructor
*/
Controller::~Controller(void){};

/** Twist callback. Receives desired twist.
* @param[in] msg last twist message receivedist
*/
void Controller::CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg){
   desired_twist_ = *msg;
   /*
   transform.setOrigin(tf::Vector3(0,0,0));
   transform.setRotation(transform.inverse().getRotation());
   tf::vector3TFToMsg(
     transform*tf::Vector3(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z),
     twist.twist.linear);
   tf::vector3TFToMsg(
     transform*tf::Vector3(twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z),
     twist.twist.angular);
       */
}

/**
* Odometry message callback
* @param[in] odom current estimated odometry
*/
void Controller::OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg){
  geometry_msgs::PoseStamped pose;
  geometry_msgs::TwistStamped twist;
  tf::Pose bt;
  // Copying values
  pose.pose   = msg->pose.pose;
  pose.header = msg->header;

  tf::poseMsgToTF(pose.pose, bt);
  bt.setOrigin(tf::Vector3(0,0,0));

  tf::vector3TFToMsg(bt*tf::Vector3(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z),
                twist.twist.linear);
  tf::vector3TFToMsg(bt*tf::Vector3(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z),
                twist.twist.angular);

  twist.header = msg->header;
  twist.header.frame_id = msg->child_frame_id;
  // Call method that actually computes/publishes control signals
  ComputeActuation(pose, twist);
}

/**
* TFCallback. Listen for TF to compute actuation inputs [DOES NOT WORK PROPERLY]
*/
void Controller::TFCallback(void){
  try{
    geometry_msgs::PoseStamped pose;
    
    // Pose
    tf::StampedTransform transform;
    listener_.lookupTransform	(	world_frame_,
                              	body_frame_,
                                ros::Time(0),
                                transform); 
    tf::poseTFToMsg(tf::Pose(transform), pose.pose);

    // Twist
    //! @TODO We want the body_frame velocity w.r.t. the world_frame, expressed in the world_frame. Notice that the
    //! only reason this actually works is because there is a bug in lookupTwist.
    geometry_msgs::TwistStamped twist; 
    listener_.lookupTwist(body_frame_,
                          world_frame_,
                          body_frame_,
                          tf::Point(0,0,0),
                          body_frame_,
                          ros::Time(0), 
                          ros::Duration(0.05),
                          twist.twist );
    // Applying filter
    FilterTwist(twist.twist);
    twist.twist = filtered_twist_;
    std::cout << ros::Time::now().toSec() - zero_time << " " <<
                 filtered_twist_.linear.x << " " <<
                 filtered_twist_.linear.y << " " <<
                 filtered_twist_.linear.z << " " <<
                 filtered_twist_.angular.x << " " <<
                 filtered_twist_.angular.y << " " <<
                 filtered_twist_.angular.z << " " << std::endl;
    // Call method that actually computes/publishes control signals.
    //ComputeActuation(pose, twist);

  }catch (tf::TransformException ex){
    ROS_ERROR("[Velocity Controller]%s",ex.what());
    ros::Duration(1.0).sleep();
  }
}

/**
 * Exponentially weigthed moving average discrete-time filter for twist.
 * @param twist unfiltered twist
 */
void Controller::FilterTwist(const geometry_msgs::Twist twist) {
  double beta = 1-alpha_;
  filtered_twist_.linear.x = alpha_*std::min(std::max(twist.linear.x, MIN_VEL), MAX_VEL) + beta*filtered_twist_
          .linear.x;
  filtered_twist_.linear.y = alpha_*std::min(std::max(twist.linear.y, MIN_VEL), MAX_VEL) + beta*filtered_twist_
          .linear.y;
  filtered_twist_.linear.z = alpha_*std::min(std::max(twist.linear.z, MIN_VEL), MAX_VEL) + beta*filtered_twist_
          .linear.z;
  filtered_twist_.angular.x = alpha_*std::min(std::max(twist.angular.x, MIN_VEL), MAX_VEL) + beta*filtered_twist_
          .angular.x;
  filtered_twist_.angular.y = alpha_*std::min(std::max(twist.angular.y, MIN_VEL), MAX_VEL) + beta*filtered_twist_
          .angular.y;
  filtered_twist_.angular.z = alpha_*std::min(std::max(twist.angular.z, MIN_VEL), MAX_VEL) + beta*filtered_twist_
          .angular.z;
}

/**
* Compute vehicle actuation input (collective thrust and attitude angles)
* @param[in] pose contains the current vehicle pose
* @param[in] twist contains the current vehicle twist
*/
void Controller::ComputeActuation(const geometry_msgs::PoseStamped &pose,const geometry_msgs::TwistStamped &twist){
        // Command accelerations
        double ddx_c = pid_ddx_.LoopOnce(desired_twist_.linear.x, twist.twist.linear.x);
        double ddy_c = pid_ddy_.LoopOnce(desired_twist_.linear.y, twist.twist.linear.y);
        double ddz_c = pid_ddz_.LoopOnce(desired_twist_.linear.z, twist.twist.linear.z);

        double yaw_rate = desired_twist_.angular.z;
        // Quadrotor input commands
        double yaw = tf::getYaw(pose.pose.orientation);
        double T_d     = (9.81 - ddz_c)*params_.mass;
        double pitch_d =  - 1/9.81  * (  ddx_c*cos(yaw) + ddy_c*sin(yaw) );
        double roll_d  =    1/9.81  * ( -ddx_c*sin(yaw) + ddy_c*cos(yaw) );

        // Assemble command message
        std_msgs::Float32 thrust;
        geometry_msgs::Vector3Stamped attitude;

        thrust.data = T_d;
        attitude.vector.x = roll_d ;
        attitude.vector.y = pitch_d;
        attitude.vector.z = yaw_rate;
        attitude.header.frame_id = body_frame_;

        thrust_pub_.publish(thrust);
        attitude_pub_.publish(attitude);
}

/**
* ROS loop
*/

void Controller::Spin(void){
zero_time = ros::Time::now().toSec();
  // Odometry based
  if (!use_tf_) {
    odom_sub_     = nh_.subscribe("odometry",1,&Controller::OdometryCallback,this);
    ros::spin();
    // TF based (not working properly)
  }else{
    ros::Rate rate(loop_rate_);
    while(ros::ok()) {
      // Try to grab tf
      TFCallback();
      // Sleep until next iteration
      ros::spinOnce();
      rate.sleep();
    }
  }


};

} // linear namespace
} // velocity namespace
} // mav_control namespace

int main(int argc, char** argv){
  ros::init(argc, argv,"velocity_controller");

  mav_control::velocity::linear::Controller vel_control;

 vel_control.Spin();

}