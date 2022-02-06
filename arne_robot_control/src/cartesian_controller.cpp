// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

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
