# Simulation Framework For Skye
This repository contains everything you need to simulate the airship Skye in [Gazebo](http://gazebosim.org/) and interact with it by using the Robotic Operating System [ROS](http://www.ros.org/). The reccomanded OS is Ubuntu 14.04.

## ROS Installation
These steps are taken from the main instllation page of ROS. 
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
Since some of the required plugins are not available in Gazebo2 (the offiacial supported version of Gazebo in ROS Indigo) it is necessary to manually install Gazebo6 and its integration with ROS. To do so setup your computer to accept software from packages.osrfoundation.org.
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
Now you can close Gazebo. To Make sure every processes started from the previous command has been close you can run
 ```bash
killall -9 gazebo & killall -9 gzserver & killall -9 gzclient
```

## Create A Catkin Workspace And Compile Source Code
Create a catkin workspace in your home folder where you are going to install every package needed to simulate Skye.
 ```bash
cd ~
mkdir -p catkin_ws/src
```
Clone the "skye_gazebo_simulation" repo in the src folder:
 ```bash
cd ~/catkin_ws/src
git clone https://github.com/skye-git/skye_gazebo_simulation -b indigo-devel
```
Clone the "hector_gazebo" repo which containes usefull plugin for our simulation in Gazebo (make sure you are still in the 'catkin_ws/src' folder).
 ```bash
git clone https://github.com/skye-git/hector_gazebo -b indigo-devel
```
Compile the packages you have just cloned.
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
To include the needed plugins in Gazebo6 you first must source the Gazebo setup.sh file:
```bash
source <install_path>/share/gazebo/setup.sh
```
where '<install_path>' is the path where Gazebo has been insalled in your computer. For example the previous command should look like similar to
```bash
source /usr/share/gazebo/setup.sh
```
Now you can modify the path where Gazebo searchs for the plugin shared libraries at runtime.
The Imu plugin from "hector_gazebo" package is located, by default, in '~/catkin_ws/devel/lib/'.
```bash
echo "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/devel/lib/" >> ~/.bashrc
source ~/.bashrc
```

## Launch A Simulation With Empty World
To launch a first simlation of Skye in an empty world in Gazebo type
```bash
roslaunch skye_ros inflate_skye.launch
```
The launch file 'inflate_skye.launch' does several things for you: it starts roscore, launches Gazebo with an instance of Skye and starts the node interface called 'skye_ros_node'.

## Skye-ROS Interface 
The package "skye_ros" provides an easy interface to interact with a simulation of Skye in Gazebo.

### Advertised Topics
  * /skye_ros/sensor_msgs/imu_ned IMU data expressed in a local NED frame attached to the IMU box.
 
### Advertised Services
  * /skye_ros/apply_wrench_cog_ned service to apply a wrench (i.e. a force and a torque) in the center of gravity (CoG) of  Skye. Wrench expressed in a NED frame attached to the CoG of Skye.
  * /skye_ros/get_link_state_ned service to get a link state (position, orientation, linear and angular velocity). This state is expressed in a world fixed NED frame.



