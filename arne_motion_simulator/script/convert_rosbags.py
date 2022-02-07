#!/usr/bin/env python3
################################################################################
# Copyright 2022 FZI Research Center for Information Technology
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
