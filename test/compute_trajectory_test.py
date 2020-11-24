#!/usr/bin/env python
import arne_skill_pipeline.trajectories as traj

"""
Compute a state-dimensional trajectory from a rosbag recording.
The trajectory has position, velocity and acceleration values for each of the
state's dimensions.

The trajectory is saved as a .csv file in the trajectories subfolder of this
package, keeping the rosbag's name.
"""

if __name__ == '__main__':
    bagfile = '../rosbags/example.bag'

    # Re-use recorded rosbag name for .csv file
    file_name = bagfile.split('/')[-1]
    file_name = file_name.split('.')[0] + '.csv'

    times, states = traj.read_rosbag(bagfile)
    trajectory = traj.compute_trajectory(times, states)
    traj.save_trajectory(file_name, trajectory)

    print("done")
