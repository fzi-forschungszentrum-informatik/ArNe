import rospy
import time
import os
from datetime import datetime
from geometry_msgs.msg import PoseStamped
import time
import threading


class TrajectoryPlayer(object):
    """ Publish trajectories to ROS topics for robot control

    This mechanism's primarily application is to turn a freshly created
    trajectory into robot control commands.
    The trajectory usually comes from the skill pipeline and is user
    parameterized with a duration, which is implicitly given by the timestamps
    of each point.  Since `cartesian_controllers` are used for robot control,
    this player continuously publishes the trajectory as discrete target poses
    for the robot's end-effector and its gripper.


    Things to clarify
    - interpolate between waypoints?
       - duration should stay as implicitly given
       - but the interpolation might need adjustments to work with different
         controllers and hardware interfaces

    """

    def __init__(self, publisher):

        # The message type is known here.
        # It does not have to be too generic.
        # Passing the publisher is important, though.  We use this here as a
        # library and should not become a rosnode ourselves.
        self.pub = publisher
        self.paused = False

    def _publish(self):
        while self.play_thread.is_alive():
            if not self.paused:
                msg = PoseStamped()
                self.pub.publish(msg)
                time.sleep(0.1)
            else:
                time.sleep(0.1)

    def play(self, trajectory):
        """ Play the trajectory for robot control

        Each trajectory starts with the robot's current pose.
        The high-level code using this player must make sure that each
        trajectory is adequately parameterized before replay.
        We require a trajectory on each call to underline this importance and
        do not provide any asynchronous load or read methods for this reason.

        This method does the following:
        - Preempt old trajectories with new ones
        - Continuously publish to defined ros topics in a separate thread
        """
        # Preempt any playing trajectory.
        # This is a no-op in case nothing is playing.
        self.stop()

        # Start publishing in a separate thread
        self.play_thread = threading.Thread(target=self._publish, args=(), daemon=True)
        self.play_thread.start()
        return self.play_thread.is_alive()

    def toggle_pause(self):
        """ Pause or unpause the current trajectory replay

        """
        self.paused = not self.paused
        return self.play_thread.is_alive()

    def stop(self):
        """ Stop trajectory replay

        """
        if hasattr(self, 'self.play_thread'):
            self.play_thread.join()
            return not self.play_thread.is_alive()
        else:
            return True
