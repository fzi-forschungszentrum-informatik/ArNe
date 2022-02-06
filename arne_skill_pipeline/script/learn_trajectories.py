#!/usr/bin/env python
import numpy as np
import pyswarms as ps
import matplotlib.pyplot as plt
import matplotlib as mpl
mpl.rcParams['pdf.fonttype'] = 42   # Fix type3 errors on IEEE uploads
"""
Learn trajectories in form of function approximation with multivariate Gaussian
basis functions.  Instead of setting fixed parameters for the Gaussians and
learn the weights (like most DMP approaches), the idea is to learn the
full parameterization of the Gaussians with Particle Swarm Optimization (PSO).

This script uses the pyswarm implementation, which needs python3.
Source a python3 virtual environment before running this script.
"""


def f(x):
    """ Function to approximate"""
    #return np.array([-np.power(i, 2) + 5 for i in x])
    return np.array([np.sin(i) / i for i in x])

# Data points
x = np.arange(1, 20, 0.1)
y = f(x)

n_gaussians = 3


def gauss(x, param_set):
    """ Linear combination of Gaussian basis functions"""
    params = np.split(param_set, 3)
    w = params[0]  # weights
    h = params[1]  # widths
    c = params[2]  # locations

    def weighted_sum(x):
        result = 0
        for i in range(n_gaussians):
            result += w[i] * np.exp(-h[i] * np.power(x - c[i], 2))
        return result
    return np.array([weighted_sum(i) for i in x])


def find_optimal_params(candidates):
    """ Function to optimize with PSO

    Evaluates the Gaussian basis function with the suggested parameter set on x
    and computes a least squares error with the labels from y.
    """
    cost = []
    for param_set in candidates:
        y_pred = gauss(x, param_set)
        mse = (np.square(y - y_pred)).mean()
        cost.append(mse)
    return np.array(cost)


# Create bounds
# w, h, c
min_bound = np.array([-10] * n_gaussians + [0.001] * n_gaussians + [1.0] * n_gaussians)
max_bound = np.array([10] * n_gaussians + [10] * n_gaussians + [20.0] * n_gaussians)
bounds = (min_bound, max_bound)

# PSO hyper parameters
options = {'c1': 0.5, 'c2': 0.3, 'w': 0.99}
#import ipdb; ipdb.set_trace()

optimizer = ps.single.global_best.GlobalBestPSO(n_particles=20, dimensions=3 * n_gaussians, options=options, bounds=bounds)
cost, pos = optimizer.optimize(find_optimal_params, iters=100, n_processes=2)

# Plot results
y_pred = gauss(x, pos)

fig, ax = plt.subplots()
ax.plot(x, y, label='f', linewidth=0.5)
ax.plot(x, y_pred, label='gauss', linewidth=0.5)
ax.legend(markerscale=1, loc='upper right')

fig.tight_layout()
plt.savefig('learn_trajectories.pdf', format='pdf', bbox_inches='tight')

# Finish
print("cost: {}".format(cost))
print("best param set: {}".format(pos))
print("done")


