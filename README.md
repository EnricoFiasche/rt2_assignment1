# Research Track 2 - First Assignment 4482512 Enrico Fiasche' (Robotics Engineering / JEMARO, Unige)

## General information
The assignment requires developing a software architecture for the control of a robot in the environment. The **go_to_point** node
must be modelled as a ROS action server.
During the simulation, the user can press "1" to give a random goal to the robot, and then press "0" to stop the robot.

## Nodes and program description
In the assignment there are four different program used to achieve the goal required by the assignment.
The program are writtten two in Python and two in C++:
- The first one, **go_to_point.py**, is an _action server_ used to reach a given goal. This node use a publisher, which publishes
the velocity of the robot through the _cmd\_vel_ topic, and a subscriber, which checks the position of the robot through the _odom_
topic. If a new goal is set, this node fix the yaw angle in order to be aligned with the goal, then the robot can go straight towards
the target point and finally, when the robot is near the desired position, it can fix the final yaw angle in order to have the same
orientation of the given goal. During these operations, the node checks continuously if the goal is canceled in order to stop the 
robot.
- The second one, **user_interface.py**, is a client used to read the user command and to send it to the **state_machine** node.
This node sends the new command through the service message _Command_ and has an action client, which is connected to **go_to_point**.
If the user presses "1", the node sends a "_start_" message to the _user\_interface_ server; if the user types "0", the node cancels
the goal, using the action client, and sends a "_stop_" message to the _user\_interface_ server.
- The third one, **position_service.cpp**, is a simple server that generates three random number. This node uses the service message
_RandomPosition_ which has as request the minimum and maximum value allowed for the x and y position, and it has as response the
three coordinates of the random goal (x,y posiion and the orientation theta).
- The last one, **state_machine.cpp**, is a node used to communicate with all the other nodes. This one has a simple client, which
is connected to the **position_service.cpp** in order to receive the next goal, a server, which receives the request done 
by the user through the **user_interface.py**, and an action client, which is used to send new goals and to check if a target is
reached.
Before start searching a new goal the state machine checks if there is a _Command_ request from a client, then if the user command
is "_start_" the node make a request to **position_service.cpp** receiving a goal between -5.0 and 5.0, regarding x and y position,
and the theta orientation, then the state machine sends this goal to the server using the action client and waits for a result,
which could be a "success" in case of goal reached and a "failure" in case of goal canceled.

### Launch files
In the folder _launch_ there are two different launch files:
- The first one, **gazeboSim.launch**, used to launch the Gazebo environment and to run the four nodes together.
- The second one, **vrepSim.launch**, used to launch only the four nodes together.

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
