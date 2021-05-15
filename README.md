# Research Track 2 - First Assignment 4482512 Enrico Fiasche' (Robotics Engineering / JEMARO, Unige)

## General information
The assignment requires developing a software architecture for the control of a robot in the environment. The two cpp
files must be written in ROS2, as components, so that using ros1_bridge, they can be interfaces with ROS nodes.
During the simulation, the user can press "1" to give a random goal to the robot, and then press "0" to stop the robot.

## Launch the simulation
### Simulation with Gazebo
The package contains the cpp nodes written in ROS2. To run correctly the simulation you must have two additional 
packages, which are **ros1_bridge** and **rt2_assignment1**, which contains the two nodes in ROS and the gazebo 
simulation.
To launch quickly all the simulation there is a script **gazeboScript.sh** that must be launched in the _root_ folder.
Before launching the script please check that you have installed **gnome-terminal** in your terminal. To install this
new command, please run:
```
sudo apt-get install gnome-terminal
```
Now you can launch the simulation in two different way:
- running each launch file on different terminals. <br/> You need three different terminal.
In the first one, used to launch ROS nodes, you have to source your ROS workspace (or sourcing directly ros.sh):
```
source path_your_ros_workspace/devel/setup.bash
```
Then you have to launch the gazeboSim (in the ROS workspace, you can use the branch main to run it), please run:
```
roslaunch rt2_assignment1 gazeboSim.launch
```
In the second one, used to run the ros bridge, you have to source ROS and ROS2 workspace (or sourcing directly ros12.sh):
```
source path_your_ros_workspace/devel/setup.bash
source /opt/ros/foxy/setup.bash
source path_your_ros2_workspace/install/setup.bash
```
Then you have to run the ros1_bridge (in the ROS2 workspace), please run:
```
ros2 run ros1_bridge dynamic_bridge
```
In the last one, used to launch the ros2 components, you have to source your ROS2 workspace (or sourcing directly ros2.sh):
```
source /opt/ros/foxy/setup.bash
source path_your_ros2_workspace/install/setup.bash
```
Then you have to launch the components (in the ROS2 workspace), please run:
```
ros2 launch rt2_assignment1 my_launch.py
```
- using only the script. <br/> You can run directly the script **gazeboScript.sh**. Before running the script please
check if the path of the workspaces are correctly defined. To launch the script, please run:
```
./gazeboScript.sh
```

### Simulation with VRep
You can also run the simulation using **VRep**. To run correctly the simulation you must have two additional 
packages, which are **ros1_bridge** and **rt2_assignment1**, which contains the two nodes in ROS, and you must have 
CoppeliaSim in your machine.
To launch quickly all the simulation there is a script **vrepScript.sh** that must be launched in the _root_ folder.
Before launching the script please check that you have installed **gnome-terminal** in your terminal. To install this
new command, please run:
```
sudo apt-get install gnome-terminal
```
Before running the codes you need to open CoppeliaSim and the correct scene. You need two different terminal to run 
the scene. In the first one you have to source your ROS workspace (or sourcing directly ros.sh) and run roscore node:
```
source path_your_ros_workspace/devel/setup.bash
roscore
```
In the second terminal you need to source your ROS workspace and run CoppeliaSim:
```
source path_your_workspace/devel/setup.bash
cd path_CoppeliaSim/
./coppeliaSim.sh
```
When the CoppeliaSim program is running, open the scene "vrepSceneR2D2.ttt" inside the rt2_assignment1 package and 
start the simulation pressing play.
Now you can launch the simulation in two different way:
- running each launch file on different terminals. <br/> You need three different terminal.
In the first one, used to launch ROS nodes, you have to source your ROS workspace (or sourcing directly ros.sh):
```
source path_your_ros_workspace/devel/setup.bash
```
Then you have to launch the vrepSim (in the ROS workspace, you can use the branch main to run it), please run:
```
roslaunch rt2_assignment1 vrepSim.launch
```
In the second one, used to run the ros bridge, you have to source ROS and ROS2 workspace (or sourcing directly ros12.sh):
```
source path_your_ros_workspace/devel/setup.bash
source /opt/ros/foxy/setup.bash
source path_your_ros2_workspace/install/setup.bash
```
Then you have to run the ros1_bridge (in the ROS2 workspace), please run:
```
ros2 run ros1_bridge dynamic_bridge
```
In the last one, used to launch the ros2 components, you have to your ROS2 workspace (or sourcing directly ros2.sh):
```
source /opt/ros/foxy/setup.bash
source path_your_ros2_workspace/install/setup.bash
```
Then you have to launch the components (in the ROS2 workspace), please run:
```
ros2 launch rt2_assignment1 my_launch.py
```
- using only the script. <br/> You can run directly the script **gazeboScript.sh**. Before running the script please
check if the path of the workspaces are correctly defined. To launch the script, please run:
```
./vrepScript.sh
```

## Nodes and program description
In this package there are two different components used to achieve the goal required by the assignment. The programs
are written in C++:
- The first one, **random_point_server.cpp**, which is a ROS2 server component that generates three random numbers.
This node uses the service message RandomPosition which has as request the minimum and maximum value allowed for the
x and y position, and it has as response the three coordinates of the random goal (x,y posiion and the orientation theta).
- The second one, **state_machine.cpp**, which is a ROS2 component used to communicate with the other nodes. This one has
two different client, one which is connected to the **random_point_server.cpp** in order to receive the next goal,
the other one which is used to send new goals and to check if a target is reached, and a server, which
receives the request done by the user through the _user\_interface.py_. When the **state_machine** received a Command
request from a client, if the user command is "start" the component start a function that makes a request to
**random_point_server.cpp** receiving a goal between -5.0 and 5.0, regarding x and y position, and the theta orientation,
then the state machine sends this goal making a request to **go_to_point** server and when the response from the server
is received, so the goal is reached, call again the function in order to reach a new goal.

## Launch file
In this package there is a launch file called **my_launch.py**, which load and run the two components described before. 

