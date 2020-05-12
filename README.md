# ArNe Motion Simulator

This is a simple RViz-based motion simulator to test and evaluate functionality in Cartesian space.
Use this to quickly evaluate your development, such as:

- Intuitiveness of the 6D navigation and grasping with the GUI
- Recording and playback of motion and grasping commands (rosbag files)
- Parametrization and deployment of skills

## Starting the simulator
In a sourced environment, launch the simulator with
```bash
roslaunch arne_motion_simulator motion_simulator.launch
```
An RViz window will pop up with a basic gripper in a constrained environment.
the gripper takes motion (speed input) from the two topics

```bash
/gripper_control_input
/motion_control_input
```

which are numerically integrated to yield a 6D pose and gripper grasping width.

## Run the GUI dummy
Open ``gui_dummy/gui.html`` from this package with your favorite web browser.
The page will start sending a continuous (but nonsense) stream of commands
using [*roslibjs*][1] from the [*rosbridge_suite*][2]

[1]: https://github.com/RobotWebTools/roslibjs
[2]: https://github.com/RobotWebTools/rosbridge_suite
