#ifndef LABROM_MAV_COMMON_CONVERSION_H
#define LABROM_MAV_COMMON_CONVERSION_H

// project libraries
#include "labrom_mav_common/typedef.h"

// ROS libraries
#include <nav_msgs/Odometry.h>

labrom_mav_common::State ConvertFromROSMsg(const nav_msgs::Odometry::ConstPtr &msg);

labrom_mav_common::Euler Quaternion2Euler(const labrom_mav_common::Quaternion &qt);

Eigen::Vector3d Labrom2Eigen(const labrom_mav_common::Vector3 &vec);

#endif
