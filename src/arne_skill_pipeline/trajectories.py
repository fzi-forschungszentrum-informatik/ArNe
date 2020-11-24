#!/usr/bin/env python
from rosbag import Bag
import numpy as np
import os
import csv
import matplotlib.pyplot as plt


class Trajectory(object):
    """ A minimal trajectory class"""

    def __init__(self, times, states, states_dot, states_ddot):
        """ All containers are time-major

        First index time, second the state's dimension.
        """
        self.times = times
        self.states = states
        self.states_dot = states_dot
        self.states_ddot = states_ddot


def read_rosbag(bagfile):
    """ Read states from a rosbag file """
    states = []
    time = []
    with Bag(bagfile) as bag:
        start = bag.get_start_time()
        for topic, msg, t in bag:
            if topic == '/state_output':
                states.append([
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                    msg.gripper.data
                ])
                time.append(t.to_sec() - start)  # Seconds from start

    # Make states an array of sampled functions
    # states = [x(t), y(t), z(t), qx(t), qy(t), qz(t), qw(t), g(t)]
    states = np.array(states).transpose()
    return time, states


def diff(states, h):
    """ Smooth differentiator

    Compute time differences with a 5-point smoothing differentiator for every
    function contained in states.
    Formulas are from Pavel Holoborodko (holoborodko.com).

    Args:
        states: A list of sequences. Each sequence represents discrete function evaluations.
        h: Step width. Average duration between two samples.

    """
    values_dot = []
    for f in states:
        f_dot = []
        for t in range(len(f)):
            # Abbreviations for function indices.
            # Extrapolate values at the boundaries.
            f1 = f[t + 1] if t < len(f) - 1 else f[t]
            f2 = f[t + 2] if t < len(f) - 2 else f[t]
            f3 = f[t + 3] if t < len(f) - 3 else f[t]
            f_1 = f[t - 1] if t > 0 else f[t]
            f_2 = f[t - 2] if t > 1 else f[t]
            f_3 = f[t - 3] if t > 2 else f[t]

            # Central difference
            df = (5 * (f1 - f_1) + 4 * (f2 - f_2) + f3 - f_3) / (32 * h)
            f_dot.append(df)
        values_dot.append(f_dot)
    return np.array(values_dot)


def compute_trajectory(times, states):
    """ Compute a trajectory from a given rosbag file

    The trajectory has position, velocity and acceleration values for each of the
    state's dimensions.
    """

    # Differentiation
    h = times[-1] / len(times)  # step width
    states_dot = diff(states, h)
    states_ddot = diff(states_dot, h)

    return Trajectory(times, states, states_dot, states_ddot)


def save_trajectory(file_name, trajectory):
    """ Write trajectory as .csv file in the trajectories folder
    """
    # Create trajectory folder if non-existent
    traj_folder = "../trajectories"
    if not os.path.exists(traj_folder):
        os.mkdir(traj_folder)

    outfile = '{}/{}'.format(traj_folder, file_name)

    # Make data time major for easier writing
    states = np.array(trajectory.states).transpose()
    states_dot = np.array(trajectory.states_dot).transpose()
    states_ddot = np.array(trajectory.states_ddot).transpose()

    # Save values as .csv files.
    # Format: time (1 value), states (8 values), d/dt states (8 values), d^2/dt^2 states (8 values)
    with open(outfile, 'w') as f:
        writer = csv.writer(f, delimiter=' ')
        for i in range(len(trajectory.times)):
            writer.writerow([trajectory.times[i]] + states[i].tolist() + states_dot[i].tolist() + states_ddot[i].tolist())


def read_trajectory(traj_file):
    """ Read trajectory from .csv file """
    #traj_file = "../trajectories/example.csv"
    times = []
    states = []
    states_dot = []
    states_ddot = []

    with open(traj_file, 'r') as f:
        reader = csv.reader(f, delimiter=' ')
        for row in reader:
            values = np.array([float(row[i]) for i in range(len(row))])
            times.append(values[0])
            states.append(values[1:9])
            states_dot.append(values[9:17])
            states_ddot.append(values[17:25])
    return Trajectory(times, states, states_dot, states_ddot)


def plot_trajectory(trajectory, outfile):
    time = trajectory.times

    # Re-arrange for plotting
    states = np.array(trajectory.states).transpose()
    states_dot = np.array(trajectory.states_dot).transpose()
    states_ddot = np.array(trajectory.states_ddot).transpose()
    nrows = 8  # state dim
    ncols = 1

    fig, grid = plt.subplots(nrows, ncols, sharey="row", figsize=(25, 20))
    fig.subplots_adjust(wspace=0.15, hspace=0.15)

    # Labels for y axis
    y = [
        '$x \quad [m]$',
        '$y \quad [m]$',
        '$z \quad [m]$',
        '$q_x \quad []$',
        '$q_y \quad []$',
        '$q_z \quad []$',
        '$q_w \quad []$',
        '$g \quad [m]$'
    ]
    dy = [
        '$\dot{x} \quad [m/s]$',
        '$\dot{y} \quad [m/s]$',
        '$\dot{z} \quad [m/s]$',
        '$\dot{q}_x \quad [1/s]$',
        '$\dot{q}_y \quad [1/s]$',
        '$\dot{q}_z \quad [1/s]$',
        '$\dot{q}_w \quad [1/s]$',
        '$\dot{g} \quad [m/s]$'
    ]
    ddy = [
        '$\ddot{x} \quad [m/s^2]$',
        '$\ddot{y} \quad [m/s^2]$',
        '$\ddot{z} \quad [m/s^2]$',
        '$\ddot{q}_x \quad [1/s^2]$',
        '$\ddot{q}_y \quad [1/s^2]$',
        '$\ddot{q}_z \quad [1/s^2]$',
        '$\ddot{q}_w \quad [1/s^2]$',
        '$\ddot{g} \quad [m/s^2]$'
    ]

    #--------------------------------------------------------------------------------
    # Plot position, velocity and acceleration for each dimension
    #--------------------------------------------------------------------------------
    for i in range(nrows):
        # f
        color = 'tab:blue'
        label = '{}$'.format(y[i].split(' ')[0])
        grid[i].set_xlabel('time [s]')
        grid[i].set_ylabel(y[i], color=color, fontsize=15)
        grid[i].plot(time, states[i, :], color=color, label=label)
        grid[i].tick_params(axis='y', labelcolor=color)
        grid[i].legend(markerscale=5, loc='upper right', bbox_to_anchor=(1.0, 1.0))

        # f'
        color = 'tab:orange'
        label = '{}$'.format(dy[i].split(' ')[0])
        ax2 = grid[i].twinx()  # instantiate a second axes that shares the same x-axis
        ax2.set_ylabel(dy[i], color=color, fontsize=15)
        ax2.plot(time, states_dot[i, :], color=color, label=label)
        ax2.tick_params(axis='y', labelcolor=color)
        ax2.legend(markerscale=5, loc='upper right', bbox_to_anchor=(1.0, 0.8))

        # f''
        color = 'tab:red'
        label = '{}$'.format(ddy[i].split(' ')[0])
        ax3 = grid[i].twinx()
        ax3.spines["right"].set_position(("axes", 1.05))
        ax3.set_ylabel(ddy[i], color=color, fontsize=15)
        ax3.plot(time, states_ddot[i, :], color=color, label=label)
        ax3.tick_params(axis='y', labelcolor=color)
        ax3.legend(markerscale=5, loc='upper right', bbox_to_anchor=(1.0, 0.6))

    #--------------------------------------------------------------------------------
    # Save a single pdf to the trajectory folder
    #--------------------------------------------------------------------------------
    # Create trajectory folder if non-existent
    traj_folder = "../trajectories"
    if not os.path.exists(traj_folder):
        os.mkdir(traj_folder)

    outfile = '{}/{}'.format(traj_folder, outfile)
    plt.savefig(outfile, format='pdf', bbox_inches='tight')
