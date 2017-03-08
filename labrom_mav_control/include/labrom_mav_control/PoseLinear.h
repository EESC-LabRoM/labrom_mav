/*************************************************************************
*   Linearized plant associate with a PID control law for pose control (Header files)
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

#ifndef LABROM_MAV_POSE_LINEAR_H
#define LABROM_MAV_POSE_LINEAR_H

// labrom_mav_common libraries
#include <labrom_mav_common/typedef.h>
#include <labrom_mav_common/conversions.h>
#include <labrom_mav_common/pid.hpp>

//! Top level namespace
namespace labrom_mav_control{
//! PID Controller based on linearized plant
class PoseLinear
{
public:
  /** Constructor */
  PoseLinear();

  /** Constructor 
  * @param[in] mav_params particular paremeters inherent to mav
  * @param[in] filename file that contains pid parameters
  * @param[in] name file that name of control law
  */
  PoseLinear(labrom_mav_common::MavParams mav_params, std::string filename, std::string name);

  /** Destructor */
  ~PoseLinear();

  /** Reset
  * Reset dynamic values, specifically PIDs and feedforward */
  void Reset();
  
  /** Set FeedForward 
  * Update feedforward value
  */
  void SetFeedforward(labrom_mav_common::MavCommand feedforward);
  
  /** Run 
  * @param[in] curr_state current mav state 
  * @param[in] target_state desired mav state
  * @param[in] controller sampling time
  */
  labrom_mav_common::MavCommand Run(const labrom_mav_common::State &curr_state, const labrom_mav_common::State &goal_state, double dt);

private:

  labrom_mav_common::MavCommand m_feedforward;    //!< feedforward action
  labrom_mav_common::controller::PID m_pidX, m_pidY, m_pidZ;   //!< PID controller on the z-axis
  labrom_mav_common::MavParams m_mav;
  std::string m_name;                             //!< Controller's name
};

} // labrom_mav_control

#endif
