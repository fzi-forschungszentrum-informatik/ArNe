#!/usr/bin/env python
from rosbag import Bag
import numpy as np
import os
import csv


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


class Trajectory(object):
    """ A minimal trajectory class"""

    def __init__(self, times, states, states_dot, states_ddot):
        self.times = times
        self.states = states
        self.states_dot = states_dot
        self.states_ddot = states_ddot


def compute_trajectory(bagfile):
    """ Compute a trajectory from a given rosbag file

    The trajectory has position, velocity and acceleration values for each of the
    state's dimensions.
    """

    times, states = read_rosbag(bagfile)

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


if __name__ == '__main__':
    bagfile = '../rosbags/example.bag'

    # Re-use recorded rosbag name for .csv file
    outfile = bagfile.split('/')[-1]
    outfile = outfile.split('.')[0] + '.csv'

    trajectory = compute_trajectory(bagfile)
    save_trajectory(outfile, trajectory)

    print("done")
