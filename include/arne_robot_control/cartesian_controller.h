// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/08/18
 *
 */
//-----------------------------------------------------------------------------

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <cartesian_motion_controller/cartesian_motion_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <arne_motion_simulator/State.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <arne_robot_control/ControlConfig.h>

namespace arne_robot_control
{
  using MotionBase =
    cartesian_motion_controller::CartesianMotionController<hardware_interface::PositionJointInterface>;

  /**
   * @brief A combined arm-gripper controller for both direct control and skill replay
   *
   * This controller provides the following interfaces:
   *
   *   1) Direct: Cartesian motion control (geometry_msgs::TwistStamped)
   *
   *   2) Direct: Gripper speed control (std_msgs::Float64)
   *
   *   3) Replay: Cartesian motion and gripper state (arne_motion_simulator::State)
   *
   * Note: This controller processes input topics for both direct control and
   * replay in parallel. If both receive data, the replay callback will
   * overrule the direct control ones. This behavior is intentional for the sake
   * of simplicity in high-level state machine code.
   *
   * Cartesian motion can be directly controlled either in the end-effector's
   * local frame or the robots global base frame.  The switch is done via via
   * dynamic reconfigure.
   *
   * This controller also publishes the current robot pose and the gripper
   * position to ROS topic for skill recording.  The publishing frequency is
   * bound to the controller_manager's update rate.
   */
  class CartesianController : public MotionBase
  {
    public:

      //! Override MotionBase' callbacks
      virtual bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh) override;

      void starting(const ros::Time& time) override;

      virtual void update(const ros::Time& time, const ros::Duration& period) override;

    private:
      //! Switch between global and relative coordinates for control
      void dynamicReconfigureCallback(ControlConfig& config, uint32_t level);
      std::atomic<bool> m_local_coordinates;

      std::shared_ptr<dynamic_reconfigure::Server<ControlConfig>> m_reconfig_server;
      dynamic_reconfigure::Server<ControlConfig>::CallbackType callback_type_;

      // Motion control
      geometry_msgs::Twist m_motion_control; //!< Cartesian end-effector velocity
      ros::Subscriber m_motion_control_subscriber;
      void motionControlCallback(const geometry_msgs::Twist& input);

      // Gripper control
      double m_gripper_control; //!< Speed, positive = open, negative = close
      std::atomic<double> m_gripper_state; //!< Abstract state between [0, 1]
      hardware_interface::JointHandle m_gripper_handle;
      ros::Subscriber m_gripper_control_subscriber;
      void gripperControlCallback(const std_msgs::Float64& input);

      // Feedback and Replay
      ros::Subscriber m_replay_subscriber;
      ros::Publisher m_current_target_publisher;
      ros::Publisher m_state_publisher;
      void replayCallback(const arne_motion_simulator::State& state);
  };

}
