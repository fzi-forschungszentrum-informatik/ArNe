import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class TrajectoryVisualizer(object):
    """ Publish trajectories for visualization in RViz

    The purpose of this class is to visualize macro generated trajectories in
    RViz. It can be used as a plausibility check before execution on the robot
    or for debugging during development.
    Add a _Path_ element to RViz that points to the right topic prior to using
    this functionality.
    """

    def __init__(self, publisher):
        """ Initialize the player with a nav_msgs.Path publisher

        Passing the publisher is important to not become a rosnode ourselves.
        """
        self.pub = publisher

    def show(self, trajectory, frame):
        """ Publish the trajectory to RViz

        Use _frame_ as the reference frame for the given trajectory.
        """
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = frame

        for s in trajectory.states:
            p = PoseStamped()
            p.pose.position.x = s[0]
            p.pose.position.y = s[1]
            p.pose.position.z = s[2]
            p.pose.orientation.x = s[3]
            p.pose.orientation.y = s[4]
            p.pose.orientation.z = s[5]
            p.pose.orientation.w = s[6]
            path.poses.append(p)

        self.pub.publish(path)
