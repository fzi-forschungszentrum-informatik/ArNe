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

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import WrenchStamped


class converter:
    """ Convert Twist messages to WrenchStamped """

    def __init__(self):
        rospy.init_node('converter', anonymous=False)

        self.twist_topic = rospy.get_param('~twist_topic',default="my_twist")
        self.wrench_topic = rospy.get_param('~wrench_topic',default="my_wrench")
        self.frame_id = rospy.get_param('~frame_id',default="world")
        self.rate = rospy.Rate(rospy.get_param('~publishing_rate',default=100))

        self.buffer = WrenchStamped()

        self.pub = rospy.Publisher(self.wrench_topic, WrenchStamped, queue_size=3)
        self.sub = rospy.Subscriber(self.twist_topic, Twist, self.twist_cb)


    def twist_cb(self,data):
        self.buffer.header.stamp     = rospy.Time.now()
        self.buffer.header.frame_id  = self.frame_id
        self.buffer.wrench.force.x   = data.linear.x
        self.buffer.wrench.force.y   = data.linear.y
        self.buffer.wrench.force.z   = data.linear.z
        self.buffer.wrench.torque.x  = data.angular.x
        self.buffer.wrench.torque.y  = data.angular.y
        self.buffer.wrench.torque.z  = data.angular.z


    def publish(self):
        if not rospy.is_shutdown():
            try:
                self.pub.publish(self.buffer)
            except rospy.ROSException:
                # Swallow 'publish() to closed topic' error.
                # This rarely happens on killing this node.
                pass


if __name__ == '__main__':
    conv = converter()
    try:
        while not rospy.is_shutdown():
            conv.publish()
            conv.rate.sleep()
    except rospy.ROSInterruptException:
        pass
