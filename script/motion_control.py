#!/usr/bin/env python

import numpy as np
import quaternion
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.server import Server
from arne_motion_simulator.cfg import MotionControlConfig


class MotionController:
    """ Convert Twist messages to PoseStamped

    """

    def __init__(self):
        rospy.init_node('motion_controller', anonymous=False)

        self.twist_topic = rospy.get_param('~twist_topic', default="my_twist")
        self.pose_topic = rospy.get_param('~pose_topic', default="my_wrench")
        self.frame_id = rospy.get_param('~frame_id', default="world")
        self.rate = rospy.Rate(rospy.get_param('~publishing_rate', default=100))

        # Cartesian motion specs
        self.pose = PoseStamped()
        self.rot = np.quaternion(0, 0, 0, 1)
        self.pos = [0, 0, 0]
        self.linear_sensitivity = 0.0
        self.angular_sensitivity = 0.0

        # Runtime configuration
        self.config = Server(MotionControlConfig, self.config_cb)

        # Input / output
        self.pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=3)
        self.sub = rospy.Subscriber(self.twist_topic, Twist, self.twist_cb)

    def config_cb(self, config, level):
        """ Get configuration from dynamic reconfigure """
        self.linear_sensitivity = config.linear_sensitivity
        self.angular_sensitivity = config.angular_sensitivity
        return config

    def twist_cb(self, data):
        """ Numerically integrate twist message into a pose

        Note: Ignores data.header

        Use self.frame_id as reference for the navigation commands, i.e. the
        resulting pose is with respect to that frame.  Sensitivity of linear
        and angular motion is adjusted with dynamic reconfigure.
        """
        dt = 0.001  # idealized behavior

        # Position update
        self.pos[0] += data.linear.x * dt * self.linear_sensitivity
        self.pos[1] += data.linear.y * dt * self.linear_sensitivity
        self.pos[2] += data.linear.z * dt * self.linear_sensitivity

        # Orientation update
        wx = data.angular.x * self.angular_sensitivity
        wy = data.angular.y * self.angular_sensitivity
        wz = data.angular.z * self.angular_sensitivity
        _, q = quaternion.integrate_angular_velocity(lambda _: (wx, wy, wz), 0, dt, self.rot)
        self.rot = q[-1]  # Get end point of interpolation

        # Build pose message
        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.frame_id = self.frame_id
        self.pose.pose.position.x = self.pos[0]
        self.pose.pose.position.y = self.pos[1]
        self.pose.pose.position.z = self.pos[2]
        self.pose.pose.orientation.x = self.rot.x
        self.pose.pose.orientation.y = self.rot.y
        self.pose.pose.orientation.z = self.rot.z
        self.pose.pose.orientation.w = self.rot.w

    def update(self):
        """ Publish the current pose to ROS topic """
        if not rospy.is_shutdown():
            try:
                self.pub.publish(self.pose)
            except rospy.ROSException:
                # Swallow 'publish() to closed topic' error.
                pass


if __name__ == '__main__':
    controller = MotionController()
    try:
        while not rospy.is_shutdown():
            controller.update()
            controller.rate.sleep()
    except rospy.ROSInterruptException:
        pass
