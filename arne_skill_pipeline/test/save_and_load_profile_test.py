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
from arne_skill_pipeline.skill import Skill
import os

if __name__ == "__main__":

    """ Learn an example trajectory and test saving and loading of its profile
    """
    traj_file = "../trajectories/example.csv"
    trajectory = traj.read_trajectory(traj_file)

    skill = Skill()

    skill.learn_trajectory(trajectory)

    # Create skill folder if non-existent
    skill_folder = "../skills"
    if not os.path.exists(skill_folder):
        os.mkdir(skill_folder)

    # Save to file
    outfile = traj_file.split('/')[-1]
    outfile = outfile.split('.')[0]
    outfile = '{}/{}.dmp'.format(skill_folder, outfile)
    skill.save_profile(outfile)

    # Load again
    skill.load_profile(outfile)

    print('done')
