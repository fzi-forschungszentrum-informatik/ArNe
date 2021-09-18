import rospy
from arne_motion_simulator.msg import State
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

    """

    def __init__(self, publisher):
        """ Initialize the player with an ArNe State publisher

        Passing the publisher is important to not become a rosnode ourselves.
        """
        self.pub = publisher
        self.paused = False
        self.stopped = True

    def _publish(self, trajectory):
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
                msg.pose.orientation.x = trajectory.states[idx][3]
                msg.pose.orientation.y = trajectory.states[idx][4]
                msg.pose.orientation.z = trajectory.states[idx][5]
                msg.pose.orientation.w = trajectory.states[idx][6]
                msg.gripper.data = trajectory.states[idx][7]
                self.pub.publish(msg)
                idx += 1

        # Trajectories have equidistant time spacing
        period = rospy.Duration(trajectory.duration / trajectory.nr_points)
        timer = rospy.Timer(period, _do_publish)

        while self.play_thread.is_alive() and not finished():
            rospy.sleep(0.1)
        timer.shutdown()

    def play(self, trajectory):
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
        """
        # Preempt any playing trajectory.
        self.stop()

        # Start publishing in a separate thread
        self.stopped = False
        self.play_thread = threading.Thread(target=self._publish, args=(trajectory, ), daemon=True)
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
