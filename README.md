# Semester-Project
This repository contains part of the code I developed for my semester project. This work was developed for Aerotain, a spinoff from ETH Zurich developing Skye, a robotic blimp.
My work consisted of developing a teach and repeat system for Skye.
Here you can find two nodes:
1. skye_controls: a ROS node that implements a Trajectory Tracking Geometric Controller specifically for Skye and a waypoint controller that parses a set of waypoints (6 DOF) and performs a movement
2. skye_teach_and_repeat: a ROS node that uses the previously developed skye_controls node to implement a teach and repeat system to automatically repeat trajectories.

Part of these were also ported to the PX4 hardware: this code is not uploaded as it consists of internal company knowledge.
Down here you can find the Readme of the intended usage, with part of the the information removed for said reasons. Credit should also go to Marco Tranzatto ( https://github.com/marco-tranzatto ) for the more general part of this readme (ROS installation, framework, Gazebo installation and propietary simulation setup.
I personally developed the readme file related to the code I developed.

NOTE: THIS CODE CANNOT RUN SINCE THE SIMULATION FRAMEWORK IS PROPIETARY SOFTWARE OF AEROTAIN. This code was released in agreement with Aerotain.


# Simulation Framework For Skye
This repository contains everything you need to simulate the airship Skye in [Gazebo](http://gazebosim.org/) and interact with it by using the Robotic Operating System [ROS](http://www.ros.org/). The recommended OS is Ubuntu 14.04.

## ROS Installation
These steps are taken from the main installation page of ROS. 
Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." You can [follow the Ubuntu guide](https://help.ubuntu.com/community/Repositories/Ubuntu) for instructions on doing this.
Setup your computer to accept software from packages.ros.org. ROS Indigo ONLY supports Saucy (13.10) and Trusty (14.04) for debian packages.
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
Set up your keys
```bash
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
```
Installation: first, make sure your Debian package index is up-to-date.
```bash
sudo apt-get update
```
Install the Desktop Install version, which provides you: ROS, rqt, rviz, and robot-generic libraries
```bash
sudo apt-get install ros-indigo-desktop
```
Initialize ROS.
 ```bash
sudo rosdep init
rosdep update
```
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:
 ```bash
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
rosinstall is a frequently used command-line tool in ROS that is distributed separately. It enables you to easily download many source trees for ROS packages with one command.
 ```bash
sudo apt-get install python-rosinstall
```
You will be using the ROS python tools wstool, rosinstall,and catkin_tools for this installation. While they may have been installed during your installation of ROS you can also install them with:
```bash 
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
```
## Gazebo6 Installation
Since some of the required plugins are not available in Gazebo2 (the official supported version of Gazebo in ROS Indigo) it is necessary to manually install Gazebo6 and its integration with ROS. To do so setup your computer to accept software from packages.osrfoundation.org.
 ```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```
Setup keys
 ```bash
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```
Install Gazebo with ROS integration.
 ```bash
sudo apt-get update
sudo apt-get install ros-indigo-gazebo6-ros-pkgs
```

## Test Gazebo-ROS Integration
Make sure the stand-alone Gazebo works by running in terminal (this command may need a couple of minutes the first time your run it):
 ```bash
gazebo
```
You should see the GUI open with an empty world. Also, test adding a model by clicking on the "Insert" tab on the left and selecting a model to add (then clicking on the simulation to select where to place the model). Now you can close Gazebo and you can kill all of its processes by
 ```bash
killall -9 gazebo & killall -9 gzserver & killall -9 gzclient
```
Finally we can test Gazebo with ROS Integration.
 ```bash
roscore &
rosrun gazebo_ros gazebo
```
The Gazebo GUI should appear with nothing inside the viewing window. To verify that the proper ROS connections are setup, view the available ROS topics by typing in a new terminal the command
 ```bash
rostopic list
```
You should see within the lists topics such as:
 ```bash
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
```
Now you can close Gazebo. To Make sure every processes started from the previous command has been closed you can run
 ```bash
killall -9 gazebo & killall -9 gzserver & killall -9 gzclient
```

## Create A Catkin Workspace And Compile Source Code
Removed as it contained propietary information

## Include Needed Plugins
To include the needed plugins in Gazebo6 you first must locate the Gazebo setup.sh file:
```bash
<install_path>/share/gazebo/setup.sh
```
where *\<install_path\>* is the path where Gazebo has been installed in your computer. For example the previous command should look similar to
```bash
/usr/share/gazebo/setup.sh
```
Now you can modify the path where Gazebo searchs for the plugin shared libraries at runtime.
The Imu plugin from "hector_gazebo" package is located, by default, in '~/catkin_ws/devel/lib/'.
```bash
echo "source <install_path>/share/gazebo/setup.sh" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib:\${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc
source ~/.bashrc
```

## Launch A Simulation With Empty World
Removed as it contained propietary information

## Interface 
Removed as it contained propietary information


## Skye Controls node usage
After the simulation frameworks starts with the inflate.launch launch file you should have gazebo open and running. The controls node provides the functionalities of the Waypoint controller and the geometric pose hold controller. This can be chosen in the skye_controls.yaml file located inside the "input" folder. Some other parameters can be changed there. Plese refer to the Doxygen-generated library to understand what those parameters do, namin is consistent across all files.

There, it is possible to choose between multiple waypoints or a single waypoint and, in case of multiple, the controller will hold the last pose provided for the whole time the node runs.

To run said node open a terminal and type:
 ```bash
roslaunch skye_controls skye_controls.launch
```
A rqt reconfiguration tool will open to allow you to tune the control parameters in real time, using dynamic rqt parameters.

## Skye Teach and Repeat node usage
You can start the teach and repeat node only when the simulation framework is already running by typing

```bash
roslaunch skye_ros inflate_skye.launch
```
You can change the node parameters by going inside the "input" folder and changing the values inside the skye_teach_and_repeat.yaml file.
Then, on another terminal, launch the teach and repeat node by typing:

 ```bash
roslaunch skye_teach_and_repeat skye_teach_and_repeat.launch
```
As for the controls_node, an rqt window will pop up to let you tune the control parameters real time.
The node will start to write a series of messages in the terminal and telling you that the mode chosen is wrong: this is normal and it means that the node is running properly.
The node uses two topics as an interface, they are listed here:

### Advertised topics
  * /skye_teach_and_repeat/std_msgs/control_mode
  * /skye_teach_and_repeat/std_msgs/action_choice

you can use them to change the control mode and the action to repeat choice as follows:

```bash
rostopic pub /skye_teach_and_repeat/std_msgs/control_mode std_msgs/Int16 "data: 2"
rostopic pub /skye_teach_and_repeat/std_msgs/control_mode std_msgs/Int16 "data: 1"
rostopic pub /skye_teach_and_repeat/std_msgs/control_mode std_msgs/Int16 "data: 3"
```
and 

```bash
rostopic pub /skye_teach_and_repeat/std_msgs/action_choice std_msgs/Int16 "data: 1" 
```
For the control mode, 1 is the teaching, 2 is the repeating and 3 is a standby mode that does nothing but it does not print lots of messages.

You can use the control node as explained before to have Skye flying in the simulation and learn the trajectory covered. Then, after killing the control mode with ctrl+c, choosing the action and control mode 2, Skye will start flying and repeating the trajectory.
