#!/usr/bin/env python3
from arne_skill_pipeline.trajectories import Trajectory
import numpy as np
import csv


class Skill:
    """ A simple representation for a scalable motion profile

    Skills are learned from recorded motion.
    They capture the `characteristics` of that recording and enable users to
    apply this motion to different use cases, in which start/goal positions are
    different and/or the duration is longer/shorter.

    They are intended as a way of programming robot motion once and re-use that
    in similar scenarios.  Their technical implementation is similar to Dynamic
    Movement Primitives (DMPs) but with substantial simplifications:

    Our intended application targets state-only trajectory execution in an
    open-loop manner.  Re-using the discretization of the initial trajectory
    allows us to use the forcing term directly for the generation of new
    trajectories, such that there is no need of parameterizing it with weighted
    Gaussians in a timeless phase variable.

    We achieve temporal scaling with a simple re-writing of time stamps for
    computed states.  Spatial scaling is achieved in a similar way to DMPs with
    numerical integrating the effect of forcing terms on our dynamical systems.
    """

    def __init__(self):
        # Transformation system and parameter settings inspired by
        # Ijspeert et al:
        # "Dynamical movement primitives: learning attractor models for motor behaviors",
        # Neural Computation, 2013

        # These will probably be constant for most skills.
        # TODO: Let's check if we need a skill-wise specialization after we
        # have more experience with this skill framework.
        self.stiffness = 0.55

        # Critical damping for > self.stiffness * 4, see Ijspeert et al, p. 4
        self.damping = 3.5

        # Nominal step width of the initial trajectory.
        self.dt = 0.01

    def learn_trajectory(self, trajectory):
        """ Learn the characteristics of a given trajectory

        Model each of the trajectory's state dimensions with a dynamical 2nd order system.
        Instead of just recording the motion, the idea is to inversely compute
        a virtual activation (=force) that was responsible throughout the
        trajectory execution to effect the recorded motion.
        This is similar to the inverse dynamics approach in mechanics, in which the applied
        forces are implicitly determined by the given motion.

        We can then apply this activation profile to our system under different
        spatial configuration (=start/goal state) and duration
        (=longer/shorter) to obtain new trajectories with similar
        characteristics for these use cases.

        Although nothing is actually learned (in the sense of ML),
        we use the term `learning`, because we can in fact generalize the
        trajectories temporally and spatially and apply their
        characteristics to unseen use cases.
        """
        # Compute the forcing term
        self.state_dim = trajectory.state_dim
        self.traj_points = trajectory.nr_points
        self.activation = np.zeros((self.state_dim, self.traj_points))

        self.dt = trajectory.duration / self.traj_points

        for d in range(self.state_dim):
            traj = trajectory.get_dimension(d)
            goal = traj['pos'][-1]
            for t in range(self.traj_points):
                # See Ijspeert et al, p.4
                self.activation[d][t] = traj['acc'][t] - self.damping * \
                    (self.stiffness * (goal - traj['pos'][t]) - traj['vel'][t])

        # Store in time major format
        self.activation = self.activation.transpose()
        print("learned trajectory")

    def save_profile(self, filename):
        """ Save the skill-defining activation to the given file

        The format is csv and each column represents the
        activation for one degree of freedom.
        """
        if not hasattr(self, 'activation'):
            print("No profile to save.")
            return

        with open(filename, 'w') as f:
            writer = csv.writer(f, quotechar='#', delimiter=' ')
            for row in self.activation:
                writer.writerow(row)
        print("saved trajectory profile here: {}".format(filename))

    def load_profile(self, filename):
        """ Read a previously saved profile

        Use this mechanism to initialize a skill before generating new trajectories
        with temporal and spatial scaling.
        """
        activation = []
        with open(filename, 'r') as f:
            reader = csv.reader(f, quotechar='#', delimiter=' ')
            for row in reader:
                activation.append([float(row[i]) for i in range(len(row))])
        self.activation = np.array(activation)
        self.state_dim = self.activation.shape[1]
        self.traj_points = self.activation.shape[0]

        print("loaded trajectory profile")

    def generate_new_trajectory(self, start_state, goal_state, duration, scale=1.0):
        """ Generate a temporally and spatially scaled state-only trajectory

        Note: Velocities and accelerations are computed as a by product of the
        numerical integration scheme. However, after refitting the scaled time
        stamps, they are no longer valid and are thus not returned to avoid
        confusion.
        """
        if not hasattr(self, 'activation'):
            print("No loaded profile.")
            return None

        # Rescaled time stamps
        timepoints = np.linspace(0, duration, self.traj_points)

        pos = np.zeros((self.state_dim, self.traj_points))
        vel = np.zeros((self.state_dim, self.traj_points))
        acc = np.zeros((self.state_dim, self.traj_points))

        # Rescale the translational part of the forcing term to take the new
        # goal distance into consideration. The is necessary to balance the
        # effect of the stiffness in the transformation system.
        # We achieve this by projecting the Euclidean part of the forcing term
        # onto the goal direction, scaling it, and re-adding the orthogonal
        # component.
        direction = np.array(goal_state[:3])
        for i in range(self.traj_points):
            act = self.activation[i]
            f = act[:3]
            f_projected = np.dot(f, direction) / np.dot(direction, direction) * direction
            f_orthogonal = f - f_projected
            f_projected *= scale
            f = f_orthogonal + f_projected
            self.activation[i] = np.concatenate((f, act[3:]))

        # Make state-major for next step
        self.activation = self.activation.transpose()

        # Obtain the trajectory as the solution to an initial
        # value problem by numerically integrating each dimension's
        # differential equation.
        for d in range(self.state_dim):
            pos[d][0] = start_state[d]
            for t in range(1, self.traj_points):
                acc[d][t] = self.damping * (
                    self.stiffness * (goal_state[d] - pos[d][t - 1]) - vel[d][t - 1]) + self.activation[d][t]
                vel[d][t] = vel[d][t - 1] + self.dt * acc[d][t]
                pos[d][t] = pos[d][t - 1] + self.dt * vel[d][t]

        print("generated new trajectory")
        return Trajectory(
            timepoints,
            np.array(pos.transpose()),
            np.zeros((self.state_dim, self.traj_points)).transpose(),
            np.zeros((self.state_dim, self.traj_points)).transpose())
