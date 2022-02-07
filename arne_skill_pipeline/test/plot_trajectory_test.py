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

""" Read a trajectory .csv file and plot the state-dimensional time series to pdf
"""

if __name__ == '__main__':
    traj_file = '../trajectories/example.csv'

    # Drog .csv extension and save as pdf.
    file_name = traj_file.split('/')[-1]
    file_name = file_name.split('.')[0] + '.pdf'

    trajectory = traj.read_trajectory(traj_file)
    traj.plot_trajectory(trajectory, file_name)
    print("done")
