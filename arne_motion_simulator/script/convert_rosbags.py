#!/usr/bin/env python3
import rosbag
from rosbag import Bag
import os
"""Prepare playback of rosbags for the motion simulator

This script renames the state topic in all recorded .bag files from ../rosbags
so that they can be played with

rosbag play <file.sim>

in the simulator. This can be used to check if recordings behave as expected
prior to generalizing them into skills..
"""

# Get all files that match the .bag extension.
# Sub folders are ignored.
files = []
folder = '../rosbags'
for file in os.listdir(folder):
    if file.endswith(".bag"):
        files.append(folder + '/' + file)

print("Renaming topics")
for fname in files:
    simfile = fname.split('/')[-1]  # name of bagfile
    simfile = simfile.split('.')[0]  # drop .bag extension
    with rosbag.Bag('{}/{}.sim'.format(folder, simfile), 'w') as Y:
        for topic, msg, t in Bag(fname):
            if topic == '/state_output':
                Y.write('/replay_input', msg, t)

print("done")
