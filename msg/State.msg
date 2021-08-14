# This is the central message type for robot control and skill learning.
# It shall be used both for simulation and real hardware and shall represent a
# common abstraction for different robots and grippers.
#
# The skill pipeline will generalize the message's dimensions as follows:
#
# idx     message value
# -----------------------
#  0      pose.position.x
#  1      pose.position.y
#  2      pose.position.z
#  3      pose.orientation.x
#  4      pose.orientation.y
#  5      pose.orientation.z
#  6      pose.orientation.w
#  7      gripper.data


# A header with time stamp and specification of the reference frame_id.
# The frame_id is classically a fixed reference of the robot, such as its
# mounting point.
std_msgs/Header header

# The robot's end-effector pose with respect to the header's frame_id.
# It describes the center of the gripper's closed fingers and will be used to
# navigate and grasp for objects.
geometry_msgs/Pose pose

# A scalar value to represent and control the gripper's state.  This can be
# chosen freely, and each gripper controller must map this to something that
# makes sense for them, e.g. `opening width in cm`, `percent closed`, etc..
std_msgs/Float64 gripper
