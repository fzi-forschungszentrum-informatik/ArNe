#!/usr/bin/env python3

import os
import rospy
from pathlib import Path
from arne_application.srv import Macro, MacroRequest, MacroResponse
from arne_motion_simulator.msg import State
from arne_skill_pipeline.skill import Skill
from arne_skill_pipeline.rosbag_recorder import RosbagRecorder
from arne_skill_pipeline.trajectory_player import TrajectoryPlayer
from arne_skill_pipeline.trajectories import read_rosbag, compute_trajectory


class Application(object):
    """ High-level program logic for the ArNe platform

    """
    def __init__(self):

        # General config
        rospy.init_node('arne_application')
        self.macro_folder = '{}/.ros/recorded_macros'.format(os.path.expanduser('~'))
        self.state_topic = 'cartesian_controller/state_output'

        # Macro functionality
        self.macro_server = rospy.Service('~macro_mode', Macro, self.macro_mode)
        self.macro_recorder = RosbagRecorder({self.state_topic: State})

        rospy.loginfo("ArNe application ready.")

    def macro_mode(self, req):
        """ Principal callback for the handling of macro functionality

        This is the primary interface to the web GUI, supporting all relevant
        macro operations in one service callback via the rosbridge. Different
        `modes` can be set in `req.mode`.

        Macros are created in an internal two-step process: First, the current
        robot (and gripper) motion is recorded to a .bag file on disk.  This
        .bag file is then parsed into a trajectory, from which a characteristic
        profile is generalized (=skill) and saved to disk with the .dmp
        extension.

        """
        # Colored output for macro operations
        NORMAL = '\033[0m'
        CYAN = '\033[1;36m'
        GREEN = '\033[1;32m'
        RED = '\033[1;31m'
        YELLOW = '\033[1;33m'

        # Start recording robot motion into internal buffers.
        # Do nothing on repeated calls.
        if req.mode is MacroRequest.START_RECORDING:
            if self.macro_recorder.start_recording(wait_for_data=True):
                rospy.loginfo(f"{CYAN}START{NORMAL} macro recording")

        # Stop any active recording and save buffers to a .bag file.
        # If that was successful, generalize the .bag file into a macro and
        # save it with .dmp extension into the same directory.
        elif req.mode is MacroRequest.STOP_RECORDING:
            if self.macro_recorder.stop_recording(self.macro_folder, prefix=req.id):
                bagfile = '{}/{}.bag'.format(self.macro_folder, req.id) 
                if Path(bagfile).is_file():
                    times, states = read_rosbag(bagfile, state_topic=self.state_topic)
                    trajectory = compute_trajectory(times, states)
                    macro = Skill()
                    macro.learn_trajectory(trajectory)
                    macro.save_profile('{}/{}.dmp'.format(self.macro_folder, req.id))
                    rospy.loginfo(f"{RED}STOP{NORMAL} macro recording")

        elif req.mode is MacroRequest.START_PLAYBACK:
            rospy.loginfo(f"{GREEN}START{NORMAL} macro playback")

        elif req.mode is MacroRequest.STOP_PLAYBACK:
            rospy.loginfo(f"{RED}STOP{NORMAL} macro playback")

        elif req.mode is MacroRequest.TOGGLE_PLAYBACK:
            rospy.loginfo(f"{YELLOW}TOGGLE{NORMAL} macro playback")

        else:
            rospy.loginfo("Unsupported macro mode")
            return MacroResponse(False, "Unsupported macro mode.")

        return MacroResponse(True, "Success.")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        print("\ndone")


if __name__ == '__main__':
    with Application() as app:
        while not rospy.is_shutdown():
            rospy.spin()
