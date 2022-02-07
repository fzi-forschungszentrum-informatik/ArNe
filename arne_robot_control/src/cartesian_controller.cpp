////////////////////////////////////////////////////////////////////////////////
// Copyright 2022 FZI Research Center for Information Technology
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//-----------------------------------------------------------------------------
/*!\file    cartesian_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/11/13
 *
 */
//-----------------------------------------------------------------------------

#include <arne_robot_control/cartesian_controller.h>
#include <cartesian_motion_controller/cartesian_motion_controller.h>
#include <cartesian_compliance_controller/cartesian_compliance_controller.h>
#include <pluginlib/class_list_macros.h>
#include <controller_interface/controller_base.h>

namespace arne_robot_control
{
  // For free space motion without contact
  using MotionControl = cartesian_motion_controller::CartesianMotionController<hardware_interface::PositionJointInterface>;
  using MotionSkillController = arne_robot_control::CartesianController<MotionControl>;

  // For motion in end-effector contact with the environment
  using ComplianceControl = cartesian_compliance_controller::CartesianComplianceController<hardware_interface::PositionJointInterface>;
  using ComplianceSkillController = arne_robot_control::CartesianController<ComplianceControl>;
}

PLUGINLIB_EXPORT_CLASS(arne_robot_control::MotionSkillController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(arne_robot_control::ComplianceSkillController, controller_interface::ControllerBase)
