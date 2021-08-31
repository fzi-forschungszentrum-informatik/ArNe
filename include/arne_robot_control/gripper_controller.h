// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    gripper_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/08/25
 *
 */
//-----------------------------------------------------------------------------

#include <ros/subscriber.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <arne_motion_simulator/State.h>
#include <std_msgs/Float64.h>
#include <hardware_interface/joint_command_interface.h>


namespace arne_robot_control
{

  /**
   * @brief A generic controller for 1-dof grippers in the ArNe project
   *
   * This controller integrates gripper speed into an abstract position in [0, 1]
   *
   * Note: Our current concept of robot skills only includes gripper positions,
   * which is the consequence of providing a common minimal base line for
   * different grippers in the project.
   *
   * Note: This controller provides input topics for both direct control and
   * replay. When both callbacks receive data in parallel, the replay callback
   * will overrule the direct control one. This behavior is intentional for the
   * sake of simplicity in high-level state machine code.
   */
  class GripperController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
  {
    public:
      GripperController() = default;

      bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& controller_nh);
      void starting(const ros::Time& time);
      void update(const ros::Time& time, const ros::Duration& period);
      void stopping(const ros::Time& time);

    private:
      hardware_interface::JointHandle m_gripper_handle;

      double m_gripper_control; //!< Speed, positive = open, negative = close
      std::atomic<double> m_gripper_state; //!< Abstract state between [0, 1]

      ros::Subscriber m_control_subscriber;
      ros::Subscriber m_replay_subscriber;
      void controlCallback(const std_msgs::Float64& input);
      void replayCallback(const arne_motion_simulator::State& state);
  };
}
