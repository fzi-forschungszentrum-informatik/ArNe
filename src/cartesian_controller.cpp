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

    m_control_subscriber = nh.subscribe("control_input", 3, &CartesianController::controlCallback, this);
    m_replay_subscriber = nh.subscribe("replay_input", 3, &CartesianController::replayCallback, this);
    m_current_target_publisher = nh.advertise<geometry_msgs::PoseStamped>("current_target", 3);

    return true;
  }

  void CartesianController::update(const ros::Time& time, const ros::Duration& period)
  {
    // Quaternion velocity from angular velocity:
    // https://math.stackexchange.com/questions/1792826
    Eigen::Quaterniond q;
    m_target_frame.M.GetQuaternion(q.x(), q.y(), q.z(), q.w());
    Eigen::Quaterniond w(0, m_control.angular.x, m_control.angular.y, m_control.angular.z);

    if (m_local_coordinates)
    {
      // Frames are given w.r.t. the robot base frame, so we need a
      // transformation before integrating in end-effector relative
      // coordinates.
      auto tmp = m_current_frame.Inverse() * m_target_frame.p;
      tmp += KDL::Vector(m_control.linear.x, m_control.linear.y, m_control.linear.z) * period.toSec();

      // Transform back
      tmp = m_current_frame * tmp;
      m_target_frame.p = KDL::Vector(tmp.x(), tmp.y(), tmp.z());

      q.coeffs() += 0.5 * (q * w).coeffs() * period.toSec();
    }
    else
    {
      // Base frame relative time integration
      m_target_frame.p += KDL::Vector(m_control.linear.x, m_control.linear.y, m_control.linear.z) * period.toSec();

      q.coeffs() += 0.5 * (w * q).coeffs() * period.toSec();
    }
    q.normalize();
    m_target_frame.M = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());

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
  }

  void CartesianController::controlCallback(const geometry_msgs::Twist& input)
  {
    m_control = input;
  }

  void CartesianController::replayCallback(const arne_motion_simulator::State& state)
  {
    geometry_msgs::PoseStamped p;
    p.header = state.header;
    p.pose = state.pose;
    MotionBase::targetFrameCallback(p);
  }
}

PLUGINLIB_EXPORT_CLASS(arne_robot_control::CartesianController, controller_interface::ControllerBase)
