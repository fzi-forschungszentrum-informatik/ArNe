#!/usr/bin/env python
from rosbag import Bag
import numpy as np
import os
import csv
"""
Compute a state-dimensional trajectory from a rosbag recording.
The trajectory has position, velocity and acceleration values for each of the
state's dimensions.

The trajectory is saved as a .csv file in the trajectories subfolder of this
package, keeping the rosbag's name.
"""

# Read states from rosbag and separate dimensions
states = []
time = []
bagfile = '../rosbags/example.bag'
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


# Differentiation
h = time[-1] / len(time)  # step width
states_dot = diff(states, h)
states_ddot = diff(states_dot, h)


#--------------------------------------------------------------------------------
# Write trajectory as .csv to the ../trajectories folder
#--------------------------------------------------------------------------------

# Create trajectory folder if non-existent
traj_folder = "../trajectories"
if not os.path.exists(traj_folder):
    os.mkdir(traj_folder)

# Re-use recorded rosbag name for .csv file
outfile = bagfile.split('/')[-1]
outfile = outfile.split('.')[0] + '.csv'
outfile = '{}/{}'.format(traj_folder, outfile)

# Make data time major for easier writing
states = np.array(states).transpose()
states_dot = np.array(states_dot).transpose()
states_ddot = np.array(states_ddot).transpose()

# Save values as .csv files.
# Format: time (1 value), states (8 values), d/dt states (8 values), d^2/dt^2 states (8 values)
with open(outfile, 'w') as f:
    writer = csv.writer(f, delimiter=' ')
    for i in range(len(time)):
        writer.writerow([time[i]] + states[i].tolist() + states_dot[i].tolist() + states_ddot[i].tolist())

# Finish
print("done")
