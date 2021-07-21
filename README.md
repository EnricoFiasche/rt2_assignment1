# Research Track 2 - Second Assignment 4482512 Enrico Fiasche' (Robotics Engineering / JEMARO, Unige)

## General information
The assignment requires developing a software architecture for the control of a robot in the environment. The **go_to_point** node
must be modelled as a ROS action server.
During the simulation, the user can press "1" to give a random goal to the robot, and then press "0" to stop the robot.

You can read the documentation at this link https://enricofiasche.github.io/rt2_assignment1/

## How to run the code
### Simulation with Gazebo
The package contains the nodes and the simulation environment for controlling a mobile robot in the Gazebo simulation environment.
Before launching the node, please source your ROS workspace, just writing this on the terminal:
```
source path_your_workspace/devel/setup.bash
```
To launch the nodes, please run:
```
roslaunch rt2_assignment1 gazeboSim.launch
```
### Simulation with VRep
You can also run the simulation using **VRep**.
You need three different terminal to run this simulation.
In the first one you need to source your ROS workspace (or sourcing directly ros.sh) and run roscore:
```
source path_your_workspace/devel/setup.bash
roscore
```
In the second terminal you need to launch **CoppeliaSim**:
```
source path_your_workspace/devel/setup.bash
cd path_CoppeliaSim/
./coppeliaSim.sh
```
When the CoppeliaSim program is running, open the scene "vrepSceneR2D2.ttt" inside the rt2_assignment1 package and start the simulation
pressing play.
In the last terminal launch the nodes:
```
source path_your_workspace/devel/setup.bash
roslaunch rt2_assignment1 vrepSim.launch
```
