#ifndef LABROM_MAV_COMMON_TYPEDEF_H
#define LABROM_MAV_COMMON_TYPEDEF_H

#include<math.h>
// customized ROS msgs 
#include <labrom_mav_common/MavCmd.h>
#include <labrom_mav_common/TeleOpCmd.h>

#include <Eigen/Dense>

namespace labrom_mav_common{
struct Vector3{
  double x, y, z;
  Vector3(): x(0), y(0), z(0){};
  Vector3(double x0, double y0, double z0): x(x0), y(y0), z(z0){};
  Eigen::Vector3d toEigen(void)
  {
    return Eigen::Vector3d(x,y,z);
  }
 
};

struct Euler{
  double roll, pitch, yaw;
  Euler(): roll(0), pitch(0), yaw(0){};
};

struct Quaternion{
  double w, x, y, z;
  Quaternion(): w(1), x(0), y(0), z(0){};
};


/* STATE RELATED */
struct Velocity{
  Vector3 linear;
  Vector3 angular;
};

struct Pose{
  Vector3 position;
  Euler euler;
  Quaternion qt;
};

struct State{
  Pose pose;
  Velocity velocity;
};


/* ACTUATION RELATED */
struct MavCommand{
  Euler euler;
  Quaternion qt;  
  Vector3 angular_vel;
  double thrust;
  
  MavCommand(): thrust(0){};
};

/* VEHICLE RELATED */
struct MavParams{
  double mass;
  double inertia;
};


} // motion_control namespace

#endif
