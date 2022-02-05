#!/usr/bin/env python3

import os
import rospy
import numpy as np
import transformations as tr
from pathlib import Path
from arne_application.srv import Macro, MacroRequest, MacroResponse
from arne_skill_pipeline.msg import State
from nav_msgs.msg import Path as PathMessage
from arne_skill_pipeline.skill import Skill
from arne_skill_pipeline.rosbag_recorder import RosbagRecorder
from arne_skill_pipeline.trajectory_player import TrajectoryPlayer
from arne_skill_pipeline.trajectory_visualizer import TrajectoryVisualizer
from arne_skill_pipeline.trajectories import read_rosbag, compute_trajectory, transform_state, transform_states, homogeneous


class Application(object):
    """ High-level program logic for the ArNe platform

    This node's primary purpose is to handle operations with macros, such as
    recording and replaying them when triggered by the GUI.  Note that there is
    a direct, topic-based connection of the GUI to the robot's Cartesian
    controller for streaming-based control of motion and gripper. There is no
    need here to interpolate and plausibility-check those commands, which is
    done by the controller itself.

    Details on macros:
    Global macros will converge to the end pose that the robot had in the
    environment during macro recording. A use case is repetitive manipulation
    from slightly different starts.
    Local macros will move entirely local to the current robot position.  A
    possible application is scratching an itchy spot on the forearm or opening
    a door handle.
    Hybrid macros converge to the end position of macro recording but do not
    enforce the final orientation.  It's suitable for Use cases when
    orientation doesn't matter, such as throwing things into a trash bin.
    """
    def __init__(self):

        # General config
        rospy.init_node('arne_application')
        self.macro_folder = '{}/.ros/recorded_macros'.format(os.path.expanduser('~'))
        self.state_topic = 'cartesian_controller/state_output'
        self.state_subscriber = rospy.Subscriber(self.state_topic, State, self.state_callback)

        # Macro functionality
        self.replay_publisher = rospy.Publisher('cartesian_controller/replay_input', State, queue_size=10)
        self.macro_server = rospy.Service('~macro_mode', Macro, self.macro_mode)
        self.motion_recorder = RosbagRecorder({self.state_topic: State})
        self.macro_player = TrajectoryPlayer(self.replay_publisher)
        self.log_execution = True

        # Visualization
        self.path_publisher = rospy.Publisher('~macro_motion', PathMessage, queue_size=10)
        self.macro_visualizer = TrajectoryVisualizer(self.path_publisher)

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
        extension. Both files are named according to the hash id of the macro
        with the respective extension.

        Macros are played with publishing to the specified replay topic of the
        controller.  Note that repeatedly starting playbacks is supported. The
        new callback will just preempt the old one. Macros always start from
        the current robot state. Playbacks can be paused/unpaused and stopped.
        A stopped playback cannot be resumed.

        Note the different, implicit coordinate systems of both data files:
        The .bag file holds the robot state with respect to the robot's base
        frame, whereas the data in the .dmp file are with respect to the
        robot's pose when recording started. Transformations between both
        frames assure that macros are generalized in a coordinate
        system-independent manner, and that robot control get's its reference
        motion in the expected base frame for replay.
        """
        # Colored output for macro operations
        NORMAL = '\033[0m'
        CYAN = '\033[1;36m'
        GREEN = '\033[1;32m'
        RED = '\033[1;31m'
        YELLOW = '\033[1;33m'
        BLUE = '\033[1;34m'

        #--------------------------------------------------------------------------------
        # Start recording
        #--------------------------------------------------------------------------------
        # Start recording robot motion into internal buffers.
        # Do nothing on repeated calls.
        if req.mode is MacroRequest.START_RECORDING:
            if self.motion_recorder.start_recording(wait_for_data=True):
                rospy.loginfo(f"{CYAN}START{NORMAL} macro recording")

        #--------------------------------------------------------------------------------
        # Stop recording
        #--------------------------------------------------------------------------------
        # Stop any active recording and save buffers to a .bag file.
        # If that was successful, generalize the .bag file into a macro and
        # save it with .dmp extension into the same directory.
        elif req.mode is MacroRequest.STOP_RECORDING:
            if self.motion_recorder.stop_recording(self.macro_folder, prefix=req.id):
                bagfile = '{}/{}.bag'.format(self.macro_folder, req.id) 
                if Path(bagfile).is_file():
                    times, states = read_rosbag(bagfile, state_topic=self.state_topic)

                    # Display all recorded states with respect to the robot's
                    # end-effector frame when recording started.  This is
                    # important for coordinate system-independent skill
                    # generalization.
                    transform_states(states, transform=tr.inverse_matrix(homogeneous(states[0])))

                    trajectory = compute_trajectory(times, states)
                    macro = Skill()
                    macro.learn_trajectory(trajectory)
                    macro.save_profile('{}/{}.dmp'.format(self.macro_folder, req.id))
                    rospy.loginfo(f"{RED}STOP{NORMAL} macro recording")

        #--------------------------------------------------------------------------------
        # Start playback
        #--------------------------------------------------------------------------------
        # Start playback of the selected macro if that exists.
        # Macros always start from the current robot state.
        elif req.mode is MacroRequest.START_PLAYBACK:
            macrofile = '{}/{}.dmp'.format(self.macro_folder, req.id) 
            bagfile = '{}/{}.bag'.format(self.macro_folder, req.id) 
            if Path(macrofile).is_file() and Path(bagfile).is_file():

                if req.duration <= 0.0:
                    return MacroResponse(False, "Invalid playback duration {}".format(req.duration))

                trajectory = self.compute_macro_motion(macrofile, bagfile, req.playback_type, req.duration)

                # Record the execution for later analysis
                if self.log_execution:
                    self.motion_recorder.start_recording(wait_for_data=True)
                    self.macro_player.play(
                        trajectory,
                        done_cb=lambda: self.motion_recorder.stop_recording(
                            self.macro_folder, prefix="{}_".format(req.id), stamped=True)
                    )
                else:
                    self.macro_player.play(trajectory)

                rospy.loginfo(f"{GREEN}START{NORMAL} macro playback")
            else:
                return MacroResponse(False, "Macro {} not found.".format(req.id))

        #--------------------------------------------------------------------------------
        # Stop playback
        #--------------------------------------------------------------------------------
        elif req.mode is MacroRequest.STOP_PLAYBACK:
            self.macro_player.stop()
            rospy.loginfo(f"{RED}STOP{NORMAL} macro playback")

        #--------------------------------------------------------------------------------
        # Pause/Unpause playback
        #--------------------------------------------------------------------------------
        # TODO: What happens when users move the robot with direct control
        # during pause?  Jumps might occur.
        elif req.mode is MacroRequest.TOGGLE_PLAYBACK:
            self.macro_player.toggle_pause()
            rospy.loginfo(f"{YELLOW}TOGGLE{NORMAL} macro playback")

        #--------------------------------------------------------------------------------
        # Delete macro
        #--------------------------------------------------------------------------------
        elif req.mode is MacroRequest.DELETE_MACRO:
            macrofile = '{}/{}.dmp'.format(self.macro_folder, req.id) 
            bagfile = '{}/{}.bag'.format(self.macro_folder, req.id) 
            try:
                os.remove(macrofile)
                os.remove(bagfile)
            except OSError:
                pass

        #--------------------------------------------------------------------------------
        # Show playback
        #--------------------------------------------------------------------------------
        # Show the macro's motion in RViz
        elif req.mode is MacroRequest.SHOW_PLAYBACK:
            macrofile = '{}/{}.dmp'.format(self.macro_folder, req.id) 
            bagfile = '{}/{}.bag'.format(self.macro_folder, req.id) 
            if Path(macrofile).is_file() and Path(bagfile).is_file():

                trajectory = self.compute_macro_motion(macrofile, bagfile, req.playback_type, req.duration)
                self.macro_visualizer.show(trajectory, frame=self.frame_id)

                rospy.loginfo(f"{BLUE}SHOW{NORMAL} macro playback")
            else:
                return MacroResponse(False, "Macro {} not found.".format(req.id))

        #--------------------------------------------------------------------------------
        # Unknown mode
        #--------------------------------------------------------------------------------
        else:
            rospy.loginfo("Unsupported macro mode")
            return MacroResponse(False, "Unsupported macro mode.")

        return MacroResponse(True, "Success.")

    def compute_macro_motion(self, macrofile, bagfile, playback_type, duration):
        """ Compute a trajectory for the given macro

        Invalid playback types default to hybrid execution.
        """
        macro = Skill()
        macro.load_profile(macrofile)

        _, recorded_states = read_rosbag(bagfile, state_topic=self.state_topic)
        start = [0, 0, 0, 0, 0, 0, 1, self.state[7]] # identity

        # Goals
        local_goal = transform_state(recorded_states[-1], transform=tr.inverse_matrix(homogeneous(recorded_states[0])))
        global_goal = transform_state(recorded_states[-1], transform=tr.inverse_matrix(homogeneous(self.state)))
        scale = np.linalg.norm(global_goal[:3]) / np.linalg.norm(local_goal[:3])

        # Local macros replicate the motion pattern in their local
        # coordinate system and apply it in our current end-effector
        # coordinate system.
        if playback_type is MacroRequest.LOCAL_MACRO:
            goal = local_goal
            scale = 1.0

        # Global macros drive to the globally recorded goal and need to
        # map that into our current end-effector coordinate system for
        # skill generation.
        elif playback_type is MacroRequest.GLOBAL_MACRO:
            goal = global_goal

        # Hybrid macros drive to the globally recorded position but
        # keep their local orientation.
        else:
            playback_type = MacroRequest.HYBRID_MACRO
            goal = [scale * i for i in local_goal[:3]] + local_goal[3:]

        # Compute how to move to the goal pose while keeping the
        # macro's motion profile.
        trajectory = macro.generate_new_trajectory(
                start_state=start,
                goal_state=goal,
                duration=duration,
                scale=scale)

        # Hybrid macros need an additional step to adequately display
        # the generated profile in the current end-effector frame.
        # See the documentation for details.
        if playback_type is MacroRequest.HYBRID_MACRO:

            T1 = tr.inverse_matrix(homogeneous(local_goal))
            p1 = tr.translation_from_matrix(T1)
            T2 = tr.inverse_matrix(homogeneous(global_goal))
            p2 = tr.translation_from_matrix(T2)

            angle = tr.angle_between_vectors(p1, p2)
            axis = tr.vector_product(p1, p2)
            if np.linalg.norm(axis) > np.finfo(float).eps:
                T_hybrid = tr.concatenate_matrices(tr.rotation_matrix(angle, axis), T1)
                T = tr.concatenate_matrices(tr.inverse_matrix(T2), T_hybrid)
                R = tr.quaternion_matrix(tr.quaternion_from_matrix(T)) # Rotation matrix
                transform_states(trajectory.states, transform=R, position_only=True)
            else:
                # The global and the local goal are so close that there's
                # no point for hybrid execution.  We default to local macro
                # execution instead.
                pass

        # Display the states back in the robot's base frame for control.
        transform_states(trajectory.states, transform=homogeneous(self.state))
        return trajectory

    def state_callback(self, state):
        """ Keep track of the robot's current state
        """
        self.frame_id = state.header.frame_id
        self.state = [
            state.pose.position.x,
            state.pose.position.y,
            state.pose.position.z,
            state.pose.orientation.x,
            state.pose.orientation.y,
            state.pose.orientation.z,
            state.pose.orientation.w,
            state.gripper.data
            ]

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        print("\ndone")


if __name__ == '__main__':
    with Application() as app:
        while not rospy.is_shutdown():
            rospy.spin()
