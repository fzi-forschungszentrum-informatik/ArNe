#!/usr/bin/env python3
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
