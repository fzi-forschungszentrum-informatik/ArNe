#!/usr/bin/env python
import arne_skill_pipeline.trajectories as traj
from arne_skill_pipeline.skill import Skill
import matplotlib.pyplot as plt
import os

if __name__ == "__main__":

    """ Learn an example trajectory and test temporal scaling

    Plot the state-dimensional time series to pdf.
    """

    # Use the example trajectory as reference.
    traj_file = "../trajectories/example.csv"
    trajectory = traj.read_trajectory(traj_file)
    start_state = trajectory.states[0]
    goal_state = trajectory.states[-1]

    skill = Skill()
    #skill.load_profile('./../skills/example.dmp')
    skill.learn_trajectory(trajectory)

    trajectory_1 = skill.generate_new_trajectory(start_state, goal_state, duration=5)
    trajectory_2 = skill.generate_new_trajectory(start_state, goal_state, duration=12.5)
    trajectory_3 = skill.generate_new_trajectory(start_state, goal_state, duration=14.5)

    # Store trajectories as rosbags for playback in the simulator
    traj.write_rosbag(trajectory_1, './../rosbags/example_1.sim')
    traj.write_rosbag(trajectory_2, './../rosbags/example_2.sim')
    traj.write_rosbag(trajectory_3, './../rosbags/example_3.sim')

    # Check specifications
    print(trajectory_1)
    print(trajectory_2)
    print(trajectory_3)

    # --------------------------------------------------------------------------------
    # Save a single pdf to the trajectory folder
    # --------------------------------------------------------------------------------
    nrows = 8  # state dim
    ncols = 1
    fig, grid = plt.subplots(nrows, ncols, sharey="row", figsize=(25, 20))
    fig.subplots_adjust(wspace=0.15, hspace=0.15)

    # Labels for y axis
    y = [
        r'$x \quad [m]$',
        r'$y \quad [m]$',
        r'$z \quad [m]$',
        r'$q_x \quad []$',
        r'$q_y \quad []$',
        r'$q_z \quad []$',
        r'$q_w \quad []$',
        r'$g \quad [m]$'
    ]

    # --------------------------------------------------------------------------------
    # Plot desired trajectory and dmp produced trajectory for each dimension
    # --------------------------------------------------------------------------------
    for i in range(nrows):
        # f
        color0 = 'tab:blue'
        color1 = 'tab:red'
        color2 = 'tab:green'
        color3 = 'tab:orange'

        label0 = '{}$'.format(y[i].split(' ')[0]) + ' desired'
        label1 = '{}$'.format(y[i].split(' ')[0]) + ' dmp 1'
        label2 = '{}$'.format(y[i].split(' ')[0]) + ' dmp 2'
        label3 = '{}$'.format(y[i].split(' ')[0]) + ' dmp 3'

        grid[i].set_xlabel('time [s]')
        grid[i].set_ylabel(y[i], fontsize=15)

        traj_0 = trajectory.get_dimension(i)
        traj_1 = trajectory_1.get_dimension(i)
        traj_2 = trajectory_2.get_dimension(i)
        traj_3 = trajectory_3.get_dimension(i)

        grid[i].plot(traj_0['time'], traj_0['pos'], color=color0, label=label0)
        grid[i].plot(traj_1['time'], traj_1['pos'], color=color1, label=label1)
        grid[i].plot(traj_2['time'], traj_2['pos'], color=color2, label=label2)
        grid[i].plot(traj_3['time'], traj_3['pos'], color=color3, label=label3)

        grid[i].tick_params(axis='y', labelcolor=color1)
        grid[i].legend(
            markerscale=5,
            loc='upper right',
            bbox_to_anchor=(
                1.0,
                1.0))

    # --------------------------------------------------------------------------------
    # Save a single pdf to a separate folder
    # --------------------------------------------------------------------------------

    # Create trajectory folder if non-existent
    traj_folder = "../dmp"
    if not os.path.exists(traj_folder):
        os.mkdir(traj_folder)

    # Drog .csv extension and save as pdf.
    outfile = traj_file.split('/')[-1]
    outfile = outfile.split('.')[0]
    outfile = '{}/{}.pdf'.format(traj_folder, outfile + '_dmp_traj')
    plt.savefig(outfile, format='pdf', bbox_inches='tight')

    # Finish
    print("done")
