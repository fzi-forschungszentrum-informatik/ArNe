#!/usr/bin/env python3

import rospy
import os
from arne_application.srv import Macro, MacroRequest, MacroResponse
from arne_motion_simulator.msg import State
from arne_skill_pipeline.rosbag_recorder import RosbagRecorder
from arne_skill_pipeline.trajectory_player import TrajectoryPlayer


class Application(object):
    """ High-level program logic for the ArNe platform

    """
    def __init__(self):

        # General config
        rospy.init_node('arne_application')
        self.macro_folder = '{}/.ros/recorded_macros'.format(os.path.expanduser('~'))

        # Macro functionality
        self.macro_server = rospy.Service('~macro_mode', Macro, self.macro_mode)
        self.macro_recorder = RosbagRecorder({'cartesian_controller/state_output': State})

        rospy.loginfo("ArNe application ready.")

    def macro_mode(self, req):
        """ Principal callback for handling of macro functionality

        """
        # Colored output for macro operations
        NORMAL = '\033[0m'
        CYAN = '\033[1;36m'
        GREEN = '\033[1;32m'
        RED = '\033[1;31m'
        YELLOW = '\033[1;33m'

        if req.mode is MacroRequest.START_RECORDING:
            self.macro_recorder.start_recording(wait_for_data=True)
            rospy.loginfo(f"{CYAN}START{NORMAL} macro recording")

        elif req.mode is MacroRequest.STOP_RECORDING:
            self.macro_recorder.stop_recording(self.macro_folder, prefix=req.id)
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
