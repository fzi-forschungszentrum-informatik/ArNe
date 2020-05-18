#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import csv
import os

""" Read trajectory .csv files and plot the state-dimensional time series to pdf
"""

# Read trajectory from .csv file
traj_file = "../trajectories/example.csv"

time = []
states = []
states_dot = []
states_ddot = []

with open(traj_file, 'r') as f:
    reader = csv.reader(f, delimiter=' ')
    for row in reader:
        values = np.array([float(row[i]) for i in range(len(row))])
        time.append(values[0])
        states.append(values[1:9])
        states_dot.append(values[9:17])
        states_ddot.append(values[17:25])


# Re-arrange for plotting
states = np.array(states).transpose()
states_dot = np.array(states_dot).transpose()
states_ddot = np.array(states_ddot).transpose()
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

# Drog .csv extension and save as pdf.
outfile = traj_file.split('/')[-1]
outfile = outfile.split('.')[0]
outfile = '{}/{}.pdf'.format(traj_folder, outfile)
plt.savefig(outfile, format='pdf', bbox_inches='tight')

# Finish
print("done")
