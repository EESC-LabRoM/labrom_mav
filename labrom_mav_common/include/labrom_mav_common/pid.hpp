/*************************************************************************
*   PID controller (Header files)
*   This file is part of labrom_mav_control
*   This code was adapted from Wolfgang Hoenig crazyflie controller.
*   Reference: Hoenig, W. et al . Mixed reality for robotics. In:IEEE/RSJ Intl
*   Conf. IntelligentRobots and Systems. Hamburg, Germany: , 2015. p. 5382 – 5387.
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

#ifndef LABROM_MAV_COMMON_CONTROLLER_PID_H
#define LABROM_MAV_COMMON_CONTROLLER_PID_H

// std libraries
#include<string>

// OpenCV
#include "opencv2/core/core.hpp"

namespace labrom_mav_common{
//! Controller classes namespace
namespace controller{
// PID controller
class PID{
public:
/** Empty constructor */
PID(void): m_kp(0), 
           m_kd(0),
           m_ki(0){};

/** Constructor 
* @param[in] kp proportioal gain
* @param[in] ki integrative gain
* @param[in] kd derivative gain
* @param[in] minOutput pid controller minimum output
* @param[in] maxOutput pid controller maximum output
* @param[in] integratorMin integral component lower saturation (anti-windup)
* @param[in] integratorMax integral component upper saturation (anti-windup)
* @param[in] name name of the controller
*
*/
PID(double kp, double kd, double ki, double minOutput, double maxOutput, 
    double integratorMin, double integratorMax, const std::string& name) : m_kp(kp), 
                                                                           m_kd(kd),
                                                                           m_ki(ki),
                                                                           m_minOutput(minOutput),
                                                                           m_maxOutput(maxOutput),
                                                                           m_integratorMin(integratorMin),
                                                                           m_integratorMax(integratorMax),
                                                                           m_sum_error(0),
                                                                           m_previousError(0),
                                                                           m_name(name){};

/** Constructor 
* @param[in] filename file (.yaml) that contains PID configuration parameters
* @param[in] name namespace for searching tree parameters 
*/
PID(std::string filename, std::string name): m_sum_error(0),
                                             m_previousError(0),
                                             m_name(name)
{
  // open file
  cv::FileStorage fSettings(filename, cv::FileStorage::READ);

  if (name.size()>0)
    name += ".";
    
  // Reading parameters from file
  m_kp = fSettings[name + "pid.kp"];
  m_ki = fSettings[name + "pid.ki"];
  m_kd = fSettings[name + "pid.kd"];  
  m_minOutput= fSettings[name + "pid.minOutput"];
  m_maxOutput = fSettings[name + "pid.maxOutput"];
  m_integratorMin = fSettings[name + "pid.integratorMin"]; 
  m_integratorMax = fSettings[name + "pid.integratorMax"];      
}


/** Destructor */
~PID(){};

/** Flush values */
void Reset()
{
  m_sum_error = 0;
  m_previousError = 0;
}

/** Get proportional component */
double getProportional(void)
{
  return m_p;
}

/** Get integrative component */
double getIntegrative(void)
{
  return m_i;
}

/** Get derivative component */
double getDerivative(void)
{
  return m_d;
}

/** Get name*/
std::string getName(void)
{
  return m_name;
}

/** Run PID 
* @param[in] value current
* @param[in] targetValue reference
* @param[in] dt sampling time
* @return pid controller output
*/
double Run(double value, double targetValue, double dt)
{
  // Proportional
  double error = targetValue - value;
  m_p = m_kp * error;
  // Integrative
  m_sum_error += error * dt;
  m_sum_error = std::max(std::min(m_sum_error, m_integratorMax), m_integratorMin);
  m_i = m_ki * m_sum_error;
  // Derivative
  m_d = 0;
  if (dt > 0){
    m_d = m_kd * (error - m_previousError) / dt;
  }
  m_previousError = error;
  // Total output
  double output = m_p + m_i + m_d;
   return std::max(std::min(output, m_maxOutput), m_minOutput);
}

private:
  double m_kp, m_ki, m_kd;                  //! Proportional, integrative and derivative gain
  double m_p, m_d, m_i;                     //! proportional, integrative and derivative components
  double m_minOutput, m_maxOutput;          //! Controller's output saturation limit
  double m_integratorMin, m_integratorMax;  //! Anti-windup
  double m_feedforward;                     //! Feedforward component 
  double m_previousError;                   //! PRevious error
  double m_sum_error;                       //! Cumulative error for integrative component 
  std::string m_name;                       //! Controller name
}; // Class

}  // controller
}  // labrom_mav_common
#endif
