// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    gripper_hw_interface.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/08/25
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>

namespace arne_robot_control
{

  /**
   * @brief A ROS-control handle for a gripper
   *
   * Use this handle in a gripper controller to read and write position data.
   * Note: Our current concept of robot skills does only include positional
   * data, which is the consequence of providing a common minimal base line for
   * different possible grippers in the project.
   */
  class GripperHandle
  {
    public:
      GripperHandle() = delete;

      GripperHandle(const std::string& name, const double* pos, double* cmd)
        : m_name(name), m_pos(pos), m_cmd(cmd)
      {
        if (!pos || !cmd)
        {
          throw hardware_interface::HardwareInterfaceException(
              "Cannot create handle '" + name + "'. Position | Command data pointer is null.");
        }
      }

      std::string getName() const {return m_name;}
      double getPosition()  const {assert(m_pos); return *m_pos;}
      double getCommand()   const {assert(m_cmd); return *m_cmd;}

      void setCommand(double command) {assert(m_cmd); *m_cmd = command;}

    private:
      std::string m_name;
      const double* m_pos = {nullptr};
      double* m_cmd       = {nullptr};
  };

  /**
   * @brief A simple ROS-control interface for a gripper
   *
   * Instantiate one of these interfaces in your RobotHW class according to
   * ROS-control practices and map the commanded positional data to something
   * that works with the gripper driver in the RobotHW's write() routine.
   */
  class GripperInterface : public
                           hardware_interface::HardwareResourceManager<GripperHandle,
                           hardware_interface::ClaimResources> {};
}
