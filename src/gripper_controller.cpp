// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    gripper_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/08/31
 *
 */
//-----------------------------------------------------------------------------

#include <arne_robot_control/gripper_controller.h>
#include <pluginlib/class_list_macros.h>

namespace arne_robot_control
{
  bool GripperController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& controller_nh)
  {
    std::string gripper = controller_nh.param("gripper", std::string("no-gripper-joint-specified"));
    m_gripper_handle = hw->getHandle(gripper);

    m_control_subscriber = controller_nh.subscribe("gripper_control_input", 3, &GripperController::controlCallback, this);
    m_replay_subscriber = controller_nh.subscribe("replay_input", 3, &GripperController::replayCallback, this);
    return true;
  }

  void GripperController::starting(const ros::Time& time)
  {
    m_gripper_state = m_gripper_handle.getPosition();
  }

  void GripperController::update(const ros::Time& time, const ros::Duration& period)
  {
    // Integrate gripper control into abstract state in [0, 1].
    m_gripper_state = m_gripper_state + m_gripper_control * period.toSec();
    m_gripper_state = std::max(0.0, std::min(1.0, m_gripper_state.load()));

    m_gripper_handle.setCommand(m_gripper_state);
  }

  void GripperController::stopping(const ros::Time& time)
  {
  }

  void GripperController::controlCallback(const std_msgs::Float64& input)
  {
    m_gripper_control = input.data;
  }

  void GripperController::replayCallback(const arne_motion_simulator::State& state)
  {
    m_gripper_state = state.gripper.data;
  }

}

PLUGINLIB_EXPORT_CLASS(arne_robot_control::GripperController, controller_interface::ControllerBase)
