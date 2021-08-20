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

namespace arne_robot_control
{
  using MotionBase =
    cartesian_motion_controller::CartesianMotionController<hardware_interface::PositionJointInterface>;

  /**
   * @brief TODO
   *
   * - Use the cartesian_motion_controller's update() mechanism to compute joint motion.
   * - The m_target_pose is computed in the callbacks or some helper function
   * - Numerically integrate twist messages
   * - Replay overrules direct control
   * - End-effector control vs base frame control via dynamic reconfigure
   *
   */
  class CartesianController : public MotionBase
  {
    public:

      //! Override MotionBase' callbacks
      virtual bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh) override;

      //! Integrate twist control before calling MotionBase::update()
      virtual void update(const ros::Time& time, const ros::Duration& period) override;

    private:

      geometry_msgs::Twist m_control;
      ros::Subscriber m_control_subscriber;
      ros::Subscriber m_replay_subscriber;
      ros::Publisher m_current_target_publisher;
      void controlCallback(const geometry_msgs::Twist& input);
      void replayCallback(const arne_motion_simulator::State& state);
  };

}
