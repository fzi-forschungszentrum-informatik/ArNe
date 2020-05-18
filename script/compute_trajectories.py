#!/usr/bin/env python
from rosbag import Bag
import numpy as np
"""
Compute a state-dimensional trajectory from a rosbag recording
"""

# Read states from rosbag and separate dimensions
states = []
time = []
bagfile = '/home/scherzin/src/robot_folders/checkout/arne/catkin_ws/src/arne_skill_pipeline/rosbags/2020-05-13-17-21-55.bag'
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


def smooth(states):
    """ Noise robust smoothing filter

    Formulas are from Pavel Holoborodko (holoborodko.com).
    """
    values = []
    for f in states:
        f_smooth = []
        for t in range(len(f)):
            # Abbreviations for function indices.
            # Extrapolate values at the boundaries.
            f0 = f[t]
            f1 = f[t + 1] if t < len(f) - 1 else f[t]
            f2 = f[t + 2] if t < len(f) - 2 else f[t]
            f3 = f[t + 3] if t < len(f) - 3 else f[t]
            f_1 = f[t - 1] if t > 0 else f[t]
            f_2 = f[t - 2] if t > 1 else f[t]
            f_3 = f[t - 3] if t > 2 else f[t]

            # Smooth
            #s = (10 * f0 + 4 * (f_1 + f1) - f_2 + f2) / 16
            s = (32 * f0 + 18 * (f_1 + f1) - 2 * (f_3 + f3)) / 64
            f_smooth.append(s)
        values.append(f_smooth)
    return np.array(values)


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
            #df = 2 * ((f1 - f_1) + f2 - f_2) / (8 * h)
            df = (5 * (f1 - f_1) + 4 * (f2 - f_2) + f3 - f_3) / (32 * h)
            #df = (f1 - f_1) / (2 * h)
            #df = (f_2 - 8 * f_1 + 8 * f1 - f2) / (12 * h)
            f_dot.append(df)
        values_dot.append(f_dot)
    return np.array(values_dot)


def ddiff(states, h):
    """ 2nd order smooth differentiator

    Compute time differences with a 5-point 2nd order smoothing differentiator.
    Formulas are from Pavel Holoborodko (holoborodko.com).

    Same args as diff().
    """
    values_ddot = []
    for f in states:
        f_ddot = []
        for t in range(len(f)):
            # Abbreviations for function indices.
            # Extrapolate values at the boundaries.
            f0 = f[t]
            f2 = f[t + 2] if t < len(f) - 2 else f[t]
            f_2 = f[t - 2] if t > 1 else f[t]

            # Central difference
            ddf = (f2 + f_2 - 2 * f0) / (4 * h * h)
            f_ddot.append(ddf)
        values_ddot.append(f_ddot)
    return np.array(values_ddot)


# Smoothing and differentiation
h = time[-1] / len(time)  # step width
#states = smooth(states)
states_dot = diff(states, h)
states_ddot = diff(states_dot, h)
#states_ddot = ddiff(states, h)

# TODO: Write trajectory as .csv to the ../trajectories folder

#--------------------------------------------------------------------------------
import matplotlib.pyplot as plt
# Plot state-dimensional time series into different sub plots
states = np.array(states)
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


# Drog .bag extension and save to ../trajectories
outfile = bagfile.split('/')[-1]
outfile = outfile.split('.')[0]
outfile = '../trajectories/{}_traj.pdf'.format(outfile)
plt.savefig(outfile, format='pdf', bbox_inches='tight')
#--------------------------------------------------------------------------------

# Finish
print("done")
