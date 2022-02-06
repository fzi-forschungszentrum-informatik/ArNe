#!/usr/bin/env python3
from rosbag import Bag
from rospy import Duration
import numpy as np
import transformations as tr
import os
import csv
import matplotlib.pyplot as plt
from arne_skill_pipeline.msg import State


class Trajectory(object):
    """ A multi-dimensional trajectory """

    def __init__(self, times, states, states_dot, states_ddot):
        """ Initialize a trajectory

        All arrays are time-major, i.e. they
        first index time, and second the state's dimension.
        """
        self.times = np.array(times)
        self.states = np.array(states)
        self.states_dot = np.array(states_dot)
        self.states_ddot = np.array(states_ddot)

        self.duration = self.times[-1]
        self.nr_points = self.times.shape[0]
        self.state_dim = self.states.shape[1]

        # Equal time axis
        if not self.nr_points == self.states.shape[0] == self.states_dot.shape[0] == self.states_ddot.shape[0]:
            raise ValueError("time axes are not equal")

        # Equal state dimension
        if not self.state_dim == self.states_dot.shape[1] == self.states_ddot.shape[1]:
            raise ValueError("state dimensions do not match")

    def get_dimension(self, index):
        """ Index individual dimensions

        Returns a dictionary of the trajectory data for the given index with
        the keys time, pos, vel, acc.
        """
        if int(index) < 0 or int(index) >= self.state_dim:
            raise ValueError("Index for state dimension exceeds bounds. Max index is {}".format(self.state_dim - 1))

        result = {
            'time': self.times,
            'pos': self.states.transpose()[index],
            'vel': self.states_dot.transpose()[index],
            'acc': self.states_ddot.transpose()[index]
        }
        return result

    def __getitem__(self, index):
        """ Index operator for individual trajectory dimensions

        Returns a dictionary of the trajectory data for the given index with
        the keys time, pos, vel, acc.
        """
        return self.get_dimension(index)

    def __str__(self):
        """ Return a readable string of most important specifications """

        s = "--------------------------"
        s += "\nTrajectory:"
        s += "\n--------------------------"
        s += "\ntimes: \t\t{}".format(len(self.times))
        s += "\nstates: \t{}".format(len(self.states))
        s += "\nstates_dot: \t{}".format(len(self.states_dot))
        s += "\nstates_ddot: \t{}".format(len(self.states_ddot))
        s += "\nduration: \t{}".format(self.duration)
        s += "\nnr_points: \t{}".format(self.nr_points)
        s += "\nstate_dim: \t{}".format(self.state_dim)
        return s


def read_rosbag(bagfile, state_topic='/state_output'):
    """ Read states from a rosbag file """
    states = []
    time = []
    with Bag(bagfile) as bag:
        start = bag.get_start_time()
        for topic, msg, t in bag:
            if topic == state_topic:
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

    return time, states


def write_rosbag(trajectory, filename):
    """ Write the given trajectory into a rosbag file

    This produces a state-only trajectory with the ROS topic /replay_input.
    Replay this rosbag for the arne_motion_simulator with

    rosbag play <bagfile>

    """
    with Bag(filename, 'w') as bag:
        for state, t in zip(trajectory.states, trajectory.times):
            msg = State()
            msg.pose.position.x = state[0]
            msg.pose.position.y = state[1]
            msg.pose.position.z = state[2]
            msg.pose.orientation.x = state[3]
            msg.pose.orientation.y = state[4]
            msg.pose.orientation.z = state[5]
            msg.pose.orientation.w = state[6]
            msg.gripper.data = state[7]

            bag.write('/replay_input', msg, Duration(t))


def diff(states, h):
    """ Smooth differentiator

    Compute time differences with a 7-point smooth noise-robust differentiator
    for every function contained in states.
    Formulas are from Pavel Holoborodko:
    http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/

    Args:
        states: A time major array of states.
        h: Step width. Average duration between two samples.

    """
    # Make states an array of sampled functions.
    # states = [x(t), y(t), z(t), qx(t), qy(t), qz(t), qw(t), g(t)]
    states = np.array(states).transpose()

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

    # Make time major again
    return np.array(values_dot).transpose()


def compute_trajectory(times, states):
    """ Compute a trajectory from given states

    The trajectory has position, velocity and acceleration values for each of the
    state's dimensions.
    """

    # Differentiation
    h = times[-1] / len(times)  # step width
    states_dot = diff(states, h)
    states_ddot = diff(states_dot, h)

    return Trajectory(times, states, states_dot, states_ddot)

def homogeneous(state):
    """ Return a homogeneous matrix from state's entries"""
    return tr.concatenate_matrices(
            tr.translation_matrix([state[0], state[1], state[2]]),
            tr.quaternion_matrix([state[6], state[3], state[4], state[5]])) # normalizes quat

def transform_state(state, transform, **kwargs):
    """ Transform the state with a static transform

    This performs state = transform * state
    with homogeneous matrices.

    Keyword arguments:
    If `position_only`, only transform the state's position.
    """
    T = transform
    if kwargs.get('position_only'):
        S = tr.translation_matrix([state[0], state[1], state[2]])
        S = tr.concatenate_matrices(T, S) # = T * S
        p = tr.translation_from_matrix(S) # [x, y, z]
        q = [state[6], state[3], state[4], state[5]] # unchanged
    else:
        S = homogeneous(state)
        S = tr.concatenate_matrices(T, S) # = T * S
        p = tr.translation_from_matrix(S) # [x, y, z]
        q = tr.quaternion_from_matrix(S) # [qw, qx, qy, qz]

    g = state[7] # gripper

    # We have a weight-last quaternion representation
    return [p[0], p[1], p[2], q[1], q[2], q[3], q[0], g]


def transform_states(states, transform, **kwargs):
    """ Transform each pose of a state array with a static transform

    Use this convenience function to change the coordinate frame of the
    `states` array.  This changes `states` in place.
    The gripper values are not changed.
    """
    for i in range(0, len(states)):
        states[i] = transform_state(states[i], transform, **kwargs)


def save_trajectory(file_name, trajectory):
    """ Write trajectory as .csv file in the trajectories folder

    Each line represents a single trajectory point with:
    time from start (1 value), states (8 values), d/dt states (8 values), d^2/dt^2 states (8 values)
    """
    # Create trajectory folder if non-existent
    traj_folder = "../trajectories"
    if not os.path.exists(traj_folder):
        os.mkdir(traj_folder)

    outfile = '{}/{}'.format(traj_folder, file_name)

    # Save values as .csv files.
    with open(outfile, 'w') as f:
        writer = csv.writer(f, delimiter=' ')
        for i in range(len(trajectory.times)):
            writer.writerow([trajectory.times[i]] + trajectory.states[i].tolist() + trajectory.states_dot[i].tolist() + trajectory.states_ddot[i].tolist())


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
