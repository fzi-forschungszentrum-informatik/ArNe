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

from arne_skill_pipeline.trajectory_player import TrajectoryPlayer
import arne_skill_pipeline.trajectories as traj
import rospy
from arne_skill_pipeline.msg import State
import time

"""
Play a trajectory as continous stream of robot control commands.

"""

if __name__ == '__main__':
    rospy.init_node("trajectory_player")

    trajectory_data = '../trajectories/example.csv'

    trajectory = traj.read_trajectory(trajectory_data)

    # Prepare ROS topic publishing here
    pub = rospy.Publisher('replay_input', State, queue_size=10)

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
