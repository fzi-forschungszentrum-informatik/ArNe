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
from arne_skill_pipeline.msg import State
import threading
import quaternion


class TrajectoryPlayer(object):
    """ Publish trajectories to ROS topics for robot control

    This mechanism's primarily application is to turn a freshly created
    trajectory into robot control commands.
    The trajectory usually comes from the skill pipeline and is user
    parameterized with a duration, which is implicitly given by the timestamps
    of each point.  Since `cartesian_controllers` are used for robot control,
    this player continuously publishes the trajectory as discrete target poses
    for the robot's end-effector and its gripper.

    """

    def __init__(self, publisher):
        """ Initialize the player with an ArNe State publisher

        Passing the publisher is important to not become a rosnode ourselves.
        """
        self.pub = publisher
        self.paused = False
        self.stopped = True

    def _publish(self, trajectory, done_cb):
        """ Publish the trajectory as discrete State topics to ROS

        We use a timer object to publish trajectory waypoints in equidistant
        time steps.  This function returns once the trajectory is finished or
        once the player has been stopped.
        """
        msg = State()
        idx = 0

        def finished():
            return idx >= trajectory.nr_points or self.stopped

        def _do_publish(event):
            nonlocal idx
            if not self.paused and not finished():
                msg.header.stamp = rospy.Time.now()
                msg.pose.position.x = trajectory.states[idx][0]
                msg.pose.position.y = trajectory.states[idx][1]
                msg.pose.position.z = trajectory.states[idx][2]

                # There's no plausibility check on the skill's generalized trajectory,
                # so normalization is required here for the orientation quaternion.
                q = quaternion.quaternion(
                        trajectory.states[idx][6], # w
                        trajectory.states[idx][3], # x
                        trajectory.states[idx][4], # y
                        trajectory.states[idx][5], # z
                    ).normalized()
                msg.pose.orientation.x = q.x
                msg.pose.orientation.y = q.y
                msg.pose.orientation.z = q.z
                msg.pose.orientation.w = q.w

                msg.gripper.data = trajectory.states[idx][7]
                self.pub.publish(msg)
                idx += 1

        # Trajectories have equidistant time spacing
        period = rospy.Duration(trajectory.duration / trajectory.nr_points)
        timer = rospy.Timer(period, _do_publish)

        while self.play_thread.is_alive() and not finished():
            rospy.sleep(0.1)
        timer.shutdown()

        # Finish with caller-specified lambda
        if done_cb is not None:
            done_cb()


    def play(self, trajectory, done_cb=None):
        """ Play the trajectory for robot control

        This function is non-blocking.
        Each trajectory starts with the robot's current pose.
        The high-level code using this player must make sure that each
        trajectory is adequately parameterized before replay.
        We require a trajectory on each call to underline this importance and
        do not provide any asynchronous load or read methods for this reason.

        This method does the following:
        - Preempt old trajectories with new ones
        - Continuously publish to Ros topics in a separate thread
        - Call `done_cb` when finished
        """
        # Preempt any playing trajectory.
        self.stop()

        # Start publishing in a separate thread
        self.stopped = False
        self.play_thread = threading.Thread(target=self._publish, args=(trajectory, done_cb), daemon=True)
        self.play_thread.start()
        return self.play_thread.is_alive()

    def toggle_pause(self):
        """ Pause or unpause the current trajectory replay

        """
        self.paused = not self.paused
        return True

    def stop(self):
        """ Stop trajectory replay

        """
        if hasattr(self, 'play_thread'):
            self.stopped = True
            self.paused = False
            self.play_thread.join()
            return not self.play_thread.is_alive()
        else:
            return True
