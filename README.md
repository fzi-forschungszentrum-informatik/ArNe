[![License](https://img.shields.io/badge/License-Apache_2.0-yellow.svg)](https://opensource.org/licenses/Apache-2.0)

# ArNe
A skill recording and generalization framework for assistive manipulators with ROS.

## Goals and rationals
Robotic manipulators are becoming increasingly important as assistive devices for people with disabilities.
Instead of steering the robot through all actions, applying self-recorded motion skills could greatly facilitate repetitive tasks.
This package provides a *practical* implementation of Dynamic Movement Primitives (DMPs)
for recording and playing back such motion patterns while teleoperating robotic arms.
The core of the framework builds on *local*, *global*, and *hybrid* skills, which is a simple heuristic for composing single-handed tasks of daily life.
The current focus is on six-axes assistive manipulators without additional sensors for perception.

---
**Note** This software is in the stage of an early demonstrator.
It is currently meant for developers doing research with [ROS](https://www.ros.org/) in health care.

## Installation
Setup a ROS workspace
```bash
mkdir -p $HOME/arne_ws/src && cd "$_"
catkin_init_workspace
```
and get the relevant ROS packages

```bash
git clone -b master git@github.com:fzi-forschungszentrum-informatik/cartesian_controllers.git
git clone -b devel git@github.com:stefanscherzinger/aria-ros.git
rosdep install --ignore-src --from-paths ./ -y -r
```

and Python packages

```bash
pip3 install numpy-quaternion transformations
```

Now build your workspace with
```bash
cd $HOME/arne_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Getting started
```bash
source $HOME/arne_ws/devel/setup.bash
roslaunch arne_application application.launch
```
This will bring up a simulated robot and the node for skill management.
Additionally start the teleoperation with
```bash
roslaunch arne_robot_control spacemouse_control.launch
```
The primary interface is the `/arne_application/macro_mode` service.
Check its documentation [here](arne_application/srv/Macro.srv).

## Future work
We are currently developing this repository with the [Aria V2 robot](https://github.com/accrea/aria-ros).
Although a little bit less known, this is a great assistive robotic manipulator for research.
A graphical user interface (GUI), the inclusion of more robots, and tutorials for custom platforms will come later in the project.

## Acknowledgement
This work has received funding by the Federal Ministry of Education and Research
of Germany (BMBF) within the project [ArNe](https://www.interaktive-technologien.de/projekte/arne) in the framework of Robotic Systems
in Health Care (project number 16SV8412).
