#!/usr/bin/env python3
################################################################################
# Copyright 2022 FZI Research Center for Information Technology
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import quaternion
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from arne_skill_pipeline.msg import State
from arne_motion_simulator.cfg import MotionControlConfig
import tf


class MotionSimulator:
    """ A dummy simulator for gripper control in 6-D Cartesian space

    This class provides two interfaces:
    1) Control:
       Send 6-D motion and gripper commands via the control topics (twist and speed).
       The current state is published continuously for recording.

    2) Simulation:
       Playback of recorded streams via the simulation topic.
    """

    def __init__(self):
        rospy.init_node('motion_controller', anonymous=False)

        self.frame_id = rospy.get_param('~frame_id', default="world")
        self.rate = rospy.Rate(rospy.get_param('~publishing_rate', default=100))

        # Limits (check urdf)
        self.gripper_max = 0.075
        self.gripper_min = 0.01
        self.pos_min = -0.75
        self.pos_max = 0.75

        # Cartesian motion specs
        self.state = State()
        self.state.header.frame_id = self.frame_id
        self.rot = np.quaternion(0, 0, 0, 1)
        self.pos = [0, 0, 0]
        self.linear_sensitivity = 0.0
        self.angular_sensitivity = 0.0
        self.gripper_sensitivity = 0.0
        self.gripper_pos = self.gripper_max  # Start with open gripper

        # Runtime configuration
        self.config = Server(MotionControlConfig, self.config_cb)

        # Input
        self.motion_control = rospy.Subscriber("motion_control_input", Twist, self.motion_control_cb)
        self.gripper_control = rospy.Subscriber("gripper_control_input", Float64, self.gripper_control_cb)
        self.simulation = rospy.Subscriber("replay_input", State, self.simulation_cb)

        # Output
        self.gripper_pub = rospy.Publisher("/joint_states", JointState, queue_size=3)
        self.state_pub = rospy.Publisher("state_output", State, queue_size=3)
        self.tf_broadcaster = tf.TransformBroadcaster()

    def config_cb(self, config, level):
        """ Get configuration from dynamic reconfigure """
        self.linear_sensitivity = config.linear_sensitivity
        self.angular_sensitivity = config.angular_sensitivity
        self.gripper_sensitivity = config.gripper_sensitivity
        return config

    def simulation_cb(self, state):
        """ Store incomming states into buffers """
        self.pos[0] = state.pose.position.x
        self.pos[1] = state.pose.position.y
        self.pos[2] = state.pose.position.z
        self.rot.x = state.pose.orientation.x
        self.rot.y = state.pose.orientation.y
        self.rot.z = state.pose.orientation.z
        self.rot.w = state.pose.orientation.w
        self.gripper_pos = state.gripper.data

    def gripper_control_cb(self, msg):
        """ integrate gripper speed into position"""
        dt = 0.001  # idealized behavior
        self.gripper_pos = max(self.gripper_min, min(self.gripper_max, self.gripper_pos + msg.data * dt))  # in [0, 1]
        self.state.header.stamp = rospy.Time.now()
        self.state.gripper.data = self.gripper_pos

    def motion_control_cb(self, data):
        """ Numerically integrate twist message into a pose

        Note: Ignores data.header

        Use self.frame_id as reference for the navigation commands, i.e. the
        resulting pose is with respect to that frame.  Sensitivity of linear
        and angular motion is adjusted with dynamic reconfigure.
        The pose is restricted to specified limits.
        """
        dt = 0.001  # idealized behavior

        def limit(value):
            return max(self.pos_min, min(self.pos_max, value))

        # Position update
        self.pos[0] = limit(self.pos[0] + data.linear.x * dt * self.linear_sensitivity)
        self.pos[1] = limit(self.pos[1] + data.linear.y * dt * self.linear_sensitivity)
        self.pos[2] = limit(self.pos[2] + data.linear.z * dt * self.linear_sensitivity)

        # Orientation update
        wx = data.angular.x * self.angular_sensitivity
        wy = data.angular.y * self.angular_sensitivity
        wz = data.angular.z * self.angular_sensitivity
        _, q = quaternion.integrate_angular_velocity(lambda _: (wx, wy, wz), 0, dt, self.rot)
        self.rot = q[-1]  # Get end point of interpolation

        # Update state message
        self.state.header.stamp = rospy.Time.now()
        self.state.pose.position.x = self.pos[0]
        self.state.pose.position.y = self.pos[1]
        self.state.pose.position.z = self.pos[2]
        self.state.pose.orientation.x = self.rot.x
        self.state.pose.orientation.y = self.rot.y
        self.state.pose.orientation.z = self.rot.z
        self.state.pose.orientation.w = self.rot.w

    def update(self):
        """ Run one simulation step """
        if not rospy.is_shutdown():
            try:
                # Motion control
                self.tf_broadcaster.sendTransform(
                    (self.pos[0], self.pos[1], self.pos[2]),
                    (self.rot.x, self.rot.y, self.rot.z, self.rot.w),
                    rospy.Time.now(),
                    "gripper",
                    self.frame_id)

                # Gripper control
                gripper_state = JointState()
                gripper_state.header.stamp = rospy.Time.now()
                gripper_state.name.append("finger_left_joint")
                gripper_state.position.append(self.gripper_pos)
                self.gripper_pub.publish(gripper_state)

                # Feedback
                self.state_pub.publish(self.state)

            except rospy.ROSException:
                # Swallow 'publish() to closed topic' error.
                pass


if __name__ == '__main__':
    sim = MotionSimulator()
    try:
        while not rospy.is_shutdown():
            sim.update()
            sim.rate.sleep()
    except rospy.ROSInterruptException:
        pass
