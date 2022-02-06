# Arne Application
High-level program logic for skill-based robot control within the *ArNe* project.

The instructions below target the complete *ArNe platform* and show the main
components as dependencies. The only exception is the GUI, which is currently [here](https://essgitlab.fzi.de/projekt-arne/evaluation).

## Install and build everything
Setup a ROS workspace in a _folder of your choice_ with
```bash
mkdir src && cd src
catkin_init_workspace
```
and get the relevant ROS packages

```bash
git clone -b master git@ids-git.fzi.de:project-groups/arne/arne_application.git
git clone -b master git@ids-git.fzi.de:project-groups/arne/arne_motion_simulator.git
git clone -b master git@ids-git.fzi.de:project-groups/arne/arne_robot_control.git
git clone -b master git@ids-git.fzi.de:project-groups/arne/arne_skill_pipeline.git
git clone -b devel git@github.com:fzi-forschungszentrum-informatik/cartesian_controllers.git
git clone -b devel git@github.com:stefanscherzinger/aria-ros.git
rosdep install --ignore-src --from-paths ./ -y -r
```

and Python packages

```bash
pip3 install numpy-quaternion transformations
```

Now build everything in the _folder of your choice_ with
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Starting everything
There's only one launch file that starts everything except for the GUI:
```bash
roslaunch arne_application application.launch
```

