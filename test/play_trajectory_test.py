#!/usr/bin/env python3
from arne_skill_pipeline.trajectory_player import TrajectoryPlayer
import arne_skill_pipeline.trajectories as traj
import rospy
from arne_motion_simulator.msg import State
import time

"""
Play a trajectory as continous stream of robot control commands.

"""

if __name__ == '__main__':
    rospy.init_node("trajectory_player")

    trajectory_data = '../trajectories/example.csv'

    trajectory = traj.read_trajectory(trajectory_data)

    # Prepare ROS topic publishing here
    pub = rospy.Publisher('simulation_input', State, queue_size=10)

    player = TrajectoryPlayer(pub)
    player.play(trajectory)  # spawns separate thread

    time.sleep(5)
    player.toggle_pause()
    print("paused")

    time.sleep(2)
    player.toggle_pause()
    print("unpaused")

    time.sleep(5)
    player.stop()
    print("stopped")
    print("done")
