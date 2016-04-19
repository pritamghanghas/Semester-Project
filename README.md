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
cd ~/catkin_ws/src
catkin_init_workspace
```
Clone the required reposotories in the src folder:
 ```bash
cd ~/catkin_ws/src
git clone https://github.com/skye-git/skye_gazebo_simulation -b indigo-devel
git clone https://github.com/skye-git/hector_gazebo -b indigo-devel
```
Compile them. Suggestion: use the option **-j** to specify the number of jobs to run simultaneously; for example `catkin_make -j4`
```bash
cd ~/catkin_ws
catkin_make
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
echo "export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib:${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc
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
  * /skye_ros/ground_truth/hull ground truth infomration of the hull. Contains the position, orientation and 
  linear velocity of the hull, expressed in the world NED frame (see below section **Frames Convetion** for further information). **Warning**: angular velocity field into this topic is filled with zeros as long as 
  a problem in Gazebo is not resolved. 

Example: echo imu_sk message. 
```bash
rostopic echo /skye_ros/sensor_msgs/imu_sk
```

### Advertised Services
  * /skye_ros/apply_wrench_cog_bf service to apply a wrench (i.e. a force and a torque) in the center of gravity (CoG) of  Skye. Wrench expressed in Skye's body frame attached to the CoG of Skye. See below section **Frames Convetion** for further information.



Example: apply a torque of 3 Nm around Skye's X axes (in body frame).
```bash
rosservice call /skye_ros/apply_wrench_cog_bf '{wrench: { force: { x: 0, y: 0, z: 0 }, torque: {x: 3, y: 0, z: 0} }, start_time: 0, duration: -1 }'
```
## Repository Layout
The following describes the directory structure and important files in the skye_gazebo_simulation repository

Folders:

  * skye_description   - Skye's Gazebo model descritpion in SDF.
  * skye_gazebo        - Contains launch files to run Gazebo and spawn Skye.
  * skye_ros           - Containes a simple interface which converts data from Gazebo ENU frame to Skye's NED frame.

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

## Skye-Controls node
The package "skye_controls" provides a pose control (aiming for a trajectory control) of Skye. 

### Input and parameters
The controls node can be tuned in two ways:
  * by editing the parameters in the yaml file ./inputs/skye_controls.yaml
  * by tuning the control parameters with RQT dynamic parameters: this opens automatically when skye_controls package is run.

The node accepts waypoints in a 6DOF format providing 3 coordinates in the NED frame (see Frames Convention section above) and and a quaternion for rotation of the body frame with respect to NED frame.

### How to run it
Running the controller is really easy: after following the guide above and having tested the simulation environment (see Launch A Simulation With Empty World section) follow these steps.
  1) Open a terminal window and type:
  ```bash
  roslaunch skye_ros inflate_skye.launch
  ```
  2) Open another terminal window and type:
  ```bash
  roslaunch skye_controls skye_controls.launch
  ```

###Example
skye_controls package comes equipped with this configuration:
6 waypoints in the 3D space at which Skye's IMU (the red box) faces the starting point (0,0,-3) in NED frame. Whenever Skye approaches the waypoint, the next one is selected and both position and attitude tend to it.