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
Create a catkin workspace in your home folder where you are going to clone every package needed to simulate Skye.
 ```bash
cd ~
mkdir -p catkin_ws/src
cd ~/catkin_ws
catkin init
```
Clone the required reposotories in the src folder (execute each line by pressing enter, do not copy and paste the whole code section):

 ```bash
cd ~/catkin_ws/src
git clone https://github.com/skye-git/skye_gazebo_simulation -b indigo-devel
git clone https://github.com/skye-git/hector_gazebo -b indigo-devel
```
Compile them. Suggestion: use the option **-j** to specify the number of jobs to run simultaneously; for example `catkin build -j4`
```bash
cd ~/catkin_ws
catkin build -j4
```
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:
 ```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

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
To launch a first simulation of Skye in an empty world in Gazebo type
```bash
roslaunch skye_ros inflate_skye.launch
```
The launch file 'inflate_skye.launch' does several things for you: it starts roscore, launches Gazebo with an instance of Skye and starts the node interface called 'skye_ros_node'.

## Skye-ROS Interface 
The package "skye_ros" provides an easy interface to interact with a simulation of Skye in Gazebo.

### Advertised Topics
  * /skye_ros/sensor_msgs/imu_sk IMU data expressed in a local frame attached to the IMU box, called IMU frame. See section *Frame Convetions* for further information.
  * /skye_ros/ground_truth/hull ground truth information of the hull. Contains the position, orientation,
  and linear velocity of the hull, expressed in the world NED frame (see below section **Frames Convention** for further information). **WARNING** The angular velocity is filled with zeros for now, until a problem in Gazebo is solved.

Example: echo imu_sk message. 
```bash
rostopic echo /skye_ros/sensor_msgs/imu_sk
```
Example: use of the orientation quaternion from /skye_ros/ground_truth/hull. This quaternion indicates the orientation of Skye's body frame with respect to the NED frame. To rotate a vector *v_b*, expressed in body frame, to a vector *v_i*, expressed in intertial frame, use:
```C++
v_i = R(q) * v_b;
```
where *R(q)* is a rotation matrix obtained from the published quaternion.

### Advertised Services
  * /skye_gz/hull/apply_wrench service to apply a wrench (i.e. a force and a torque) in the center of gravity (CoG) of  Skye. Wrench expressed in Skye's body frame attached to the CoG of Skye. See below section **Frames Convetion** for further information.
  *  /skye_gz/hull/apply_force service to apply (only) a force in the center of gravity (CoG) of  Skye. Force expressed in Skye's body frame attached to the CoG of Skye. The torque present of the body is not affected. See below section **Frames Convetion** for further information.
  *  /skye_gz/hull/apply_torque service to apply (only) a torque in the center of gravity (CoG) of  Skye. Force expressed in Skye's body frame attached to the CoG of Skye. The force present of the body is not affected.See below section **Frames Convetion** for further information.


Example: apply a torque of 3 Nm around Skye's X axes (in body frame).
```bash
rosservice call /skye_gz/hull/apply_wrench '{wrench: { force: { x: 0, y: 0, z: 0 }, torque: {x: 3, y: 0, z: 0} }, start_time: 0, duration: -1 }'
```
## Repository Layout
The following describes the directory structure and important files in the skye_gazebo_simulation repository

Folders:

  * skye_description   - Skye's Gazebo model descritpion in SDF.
  * skye_gazebo        - Contains launch files to run Gazebo and spawn Skye.
  * skye_ros           - Containes a simple interface and some gazebo plugins which convert data from Gazebo ENU frame to Skye's NED frame.

## Frames Convention
Gazebo and ROS use ENU frame convention, i.e. X axis points to East, Y axis to North and Z axis up. We use
trhee slightly different frames, that are common in air vehicles: r

  * World NED frame: X axis pointing to North, Y axis pointing to East and Z axis pointing to Down.
  * Skye's body frame: frame attached to the Center of Gravity (CoG) of the hull. It has the X axis pointing geometrically forward W.R.T the eye, the Y axis pointing geometrically right W.R.T the hull and the Z axis pointing geometrically down W.R.T the hull.
  * IMU frame: frame attached to the center of the IMU (red box). It has the X axis pointing geometrically forward W.R.T the eye, the Y axis pointing geometrically right W.R.T the hull and the Z axis pointing geometrically down W.R.T the hull.

The picture below gives an overview of these three frames. Note that the initial default position of Skye is rotated of 90 degrees about the Z axis with respect to the world NED frame.

<p align="center">
  <img src="skye_ros/doc/frames.png" width="650"/>
</p>

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