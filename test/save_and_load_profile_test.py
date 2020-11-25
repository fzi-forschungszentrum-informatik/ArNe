#!/usr/bin/env python
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
