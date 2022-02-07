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

import arne_skill_pipeline.trajectories as traj

"""
Compute a state-dimensional trajectory from a rosbag recording.
The trajectory has position, velocity and acceleration values for each of the
state's dimensions.

The trajectory is saved as a .csv file in the trajectories subfolder of this
package, keeping the rosbag's name.
"""

if __name__ == '__main__':
    bagfile = '../rosbags/example.bag'

    # Re-use recorded rosbag name for .csv file
    file_name = bagfile.split('/')[-1]
    file_name = file_name.split('.')[0] + '.csv'

    times, states = traj.read_rosbag(bagfile)
    trajectory = traj.compute_trajectory(times, states)
    traj.save_trajectory(file_name, trajectory)

    print("done")
