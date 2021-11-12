// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2021/08/18
 *
 */
//-----------------------------------------------------------------------------

#include "arne_skill_pipeline/State.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/time.h"
#include <arne_robot_control/cartesian_controller.h>
#include <kdl/frames.hpp>
#include <pluginlib/class_list_macros.h>
#include <controller_interface/controller_base.h>

#include <Eigen/Dense>

namespace arne_robot_control
{
  bool CartesianController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& nh)
  {
    Base::init(hw, nh);

    // Connect dynamic reconfigure and overwrite the default values with values
    // on the parameter server. This is done automatically if parameters with
    // the according names exist.
    callback_type_ =
      std::bind(&CartesianController::dynamicReconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);

    m_reconfig_server = std::make_shared<dynamic_reconfigure::Server<ControlConfig> >(nh);
    m_reconfig_server->setCallback(callback_type_);

    // Motion control
    m_motion_control_subscriber = nh.subscribe("motion_control_input", 3, &CartesianController::motionControlCallback, this);

    // Gripper control
    std::string gripper = nh.param("gripper", std::string(""));
    if (!gripper.empty())
    {
      m_gripper_control_subscriber = nh.subscribe("gripper_control_input", 3, &CartesianController::gripperControlCallback, this);
      m_gripper_handle = hw->getHandle(gripper);
      m_use_gripper = true;;
    }
    else
    {
      m_use_gripper = false;;
    }

    // Feedback and replay
    m_replay_subscriber = nh.subscribe("replay_input", 3, &CartesianController::replayCallback, this);
    m_current_target_publisher = nh.advertise<geometry_msgs::PoseStamped>("current_target", 3);
    m_state_publisher = nh.advertise<arne_skill_pipeline::State>("state_output", 3);

    return true;
  }

  void CartesianController::starting(const ros::Time& time)
  {
    MotionBase::starting(time);
    if (m_use_gripper)
    {
      m_gripper_state = m_gripper_handle.getPosition();
    }
    else
    {
      m_gripper_state = -1.0;
    }
  }

  void CartesianController::update(const ros::Time& time, const ros::Duration& period)
  {
    // State feedback for skill recording
    arne_skill_pipeline::State state;
    state.header.frame_id = Base::m_robot_base_link;
    state.header.stamp = ros::Time::now();
    state.gripper.data = m_gripper_state;
    state.pose.position.x = m_current_frame.p.x();
    state.pose.position.y = m_current_frame.p.y();
    state.pose.position.z = m_current_frame.p.z();
    m_current_frame.M.GetQuaternion(
        state.pose.orientation.x, state.pose.orientation.y,
        state.pose.orientation.z, state.pose.orientation.w);
    m_state_publisher.publish(state);

    // Integrate gripper control into abstract state in [0, 1].
    if (m_use_gripper)
    {
      m_gripper_state = m_gripper_state + m_gripper_control * period.toSec();
      m_gripper_state = std::max(0.0, std::min(1.0, m_gripper_state.load()));
      m_gripper_handle.setCommand(m_gripper_state);
    }

    // Quaternion velocity from angular velocity:
    // https://math.stackexchange.com/questions/1792826
    Eigen::Quaterniond q;
    m_target_frame.M.GetQuaternion(q.x(), q.y(), q.z(), q.w());
    Eigen::Quaterniond w(0, m_motion_control.angular.x, m_motion_control.angular.y, m_motion_control.angular.z);

    if (m_local_coordinates)
    {
      // Frames are given w.r.t. the robot base frame, so we need a
      // transformation before integrating in end-effector relative
      // coordinates.
      auto tmp = m_current_frame.Inverse() * m_target_frame.p;
      tmp += KDL::Vector(m_motion_control.linear.x, m_motion_control.linear.y, m_motion_control.linear.z) * period.toSec();

      // Transform back
      tmp = m_current_frame * tmp;
      m_target_frame.p = KDL::Vector(tmp.x(), tmp.y(), tmp.z());

      q.coeffs() += 0.5 * (q * w).coeffs() * period.toSec();
    }
    else
    {
      // Base frame relative time integration
      m_target_frame.p += KDL::Vector(m_motion_control.linear.x, m_motion_control.linear.y, m_motion_control.linear.z) * period.toSec();

      q.coeffs() += 0.5 * (w * q).coeffs() * period.toSec();
    }
    q.normalize();
    m_target_frame.M = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());

    // Make sure we don't stray too far from our current pose.
    limitTargetOffset(m_target_frame);

    // Give visual feedback on the current target
    geometry_msgs::PoseStamped current_target;
    current_target.header.stamp = ros::Time::now();
    current_target.header.frame_id = Base::m_robot_base_link;
    current_target.pose.position.x = m_target_frame.p.x();
    current_target.pose.position.y = m_target_frame.p.y();
    current_target.pose.position.z = m_target_frame.p.z();
    current_target.pose.orientation.x = q.x();
    current_target.pose.orientation.y = q.y();
    current_target.pose.orientation.z = q.z();
    current_target.pose.orientation.w = q.w();
    m_current_target_publisher.publish(current_target);

    // Process m_target_frame as usual
    MotionBase::update(time, period);
  }

  void CartesianController::dynamicReconfigureCallback(ControlConfig& config, uint32_t level)
  {
    m_local_coordinates = config.local_coordinates;
    m_max_lin_offset = config.max_lin_offset;
  }

  void CartesianController::motionControlCallback(const geometry_msgs::Twist& input)
  {
    m_motion_control = input;
  }

  void CartesianController::gripperControlCallback(const std_msgs::Float64& input)
  {
    m_gripper_control = input.data;
  }

  void CartesianController::replayCallback(const arne_skill_pipeline::State& state)
  {
    m_gripper_state = state.gripper.data;
    geometry_msgs::PoseStamped p;

    // State feedback during recording is always given in robot base frame.
    // Generalizing skills from this recording does not change this, so it's
    // safe to always set this frame here for control.
    p.header.frame_id = Base::m_robot_base_link;
    p.pose = state.pose;
    MotionBase::targetFrameCallback(p);
  }

  void CartesianController::limitTargetOffset(KDL::Frame& target)
  {
    auto offset = target.p - m_current_frame.p;
    if (offset.Normalize() > m_max_lin_offset)
    {
      target.p = m_current_frame.p + offset * m_max_lin_offset;
    }
  }
}

PLUGINLIB_EXPORT_CLASS(arne_robot_control::CartesianController, controller_interface::ControllerBase)
