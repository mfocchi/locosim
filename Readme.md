# What is locosim?

Locosim is  didactic framework to learn/test basic controllers schemes on quadrupeds (HyQ/Solo robots are supported) and manipulators (UR5 robot is supported).

Locosim is composed by a **roscontrol** node called ros_impedance_controller written in C++ that interfaces the python ros node (where the controller is written) to a Gazebo simulator. All the didactic labs have a description, with exercises of increasing complexity, in the folder **lab_descriptions** inside robot_control submodule. For the controller plotting / logging utilities are available to evaluate the results and a config file (LX_conf.py) to change the controller parameters. 

# Usage with a Virtual Machine

Download the following [virtual machine](https://www.dropbox.com/sh/5trh0s5y1xzdjds/AACchznJb7606MbQKb6-fUiUa) (made for VirtualBox) and run the lab experiments that are present in  **robot_control/lab_descriptions**. Note that there are 2 Virtual machines available, one for Ubuntu 16 and one for Ubuntu 20. I strongly recommend to use the Ubuntu 20 one, because all the files base_controllerXX.py files are no longer compatible with Ubuntu 16. 



# Installation on Ubuntu 16 / Ubuntu 20

### SOFTWARE VERSIONS:

In the following commands, if you are installing for Ubuntu 16, you need to consider **kinetic** as  ROS version, **py27**, and **pip**, for Ubuntu 20, consider **noetic** as ROS version, **py38**  and **pip3**.

First clone the repository inside a ros workspace, then remember to update its submodules  (robot_control and ros_impedance_controller) running this command in the locosim root:

```
git submodule update --init --recursive
```

**IMPORTANT NOTE!** you will not be able to checkout the submodules unless you generate and add your SSH key to your Github account, as explained here:

https://github.com/mfocchi/lab-docker/blob/master/install_docker.md#installing-git-and-ssh-key



### Pinocchio stuff

**add robotpkg repositories**

```
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
```

```
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub $(lsb_release -sc) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
```

**Register the authentication certificate of robotpkg:**

```
sudo apt install curl
```

```
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
```

```
sudo apt-get update
```

```
sudo apt install robotpkg-py27-eigenpy	
```

```
sudo apt install robotpkg-py27-pinocchio
```



### Install ROS 

setup your source list:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up your keys:

```
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
```

Find which ROS version (e.g. "kinetic") is compatible with your Ubuntu version (check here http://wiki.ros.org/Distributions) and replace in ROSVERSION string in the the following command:

```
sudo apt-get install ros-ROSVERSION-desktop-full
```

install packages:

```
sudo apt-get install ros-$(rosversion -d)-urdfdom-py
```

```
sudo apt-get install ros-$(rosversion -d)-joint-state-publisher
```

```
sudo apt-get install ros-$(rosversion -d)-joint-state-publisher-gui
```

```
sudo apt-get install ros-$(rosversion -d)-joint-state-controller 
```

```
sudo apt-get install ros-$(rosversion -d)-rviz-visual-tools
```

```
sudo apt-get install ros-$(rosversion -d)-gazebo-msgs
```

```
sudo apt-get install ros-$(rosversion -d)-control-toolbox
```

```
sudo apt-get install ros-$(rosversion -d)-gazebo-ros
```

```
sudo apt-get install ros-$(rosversion -d)-controller-manager
```



### Install robot urdfs 

```
sudo apt install robotpkg-py27-example-robot-data
```



###  Python

```
sudo apt-get install python-scipy
```

```
sudo apt-get install python-matplotlib=2.0.2
```

```
sudo apt-get install robotpkg-py27-quadprog  
```

```
sudo apt-get install python-termcolor
```

```
pip install cvxpy==1.2.0
```



### Download code and Setup ros workspace

```
mkdir -p ~/ros_ws/src
```

```
cd ~/ros_ws/src
```

```
catkin_init_workspace
```

This will create ""/opt/ros/$(rosversion -d)/setup.bash" that you add to your .bashrc with the following:

```
echo "source /opt/ros/$(rosversion -d)/setup.bash" >> ~/.bashrc
```

```
source ~/.bashrc
```

```
cd ~/ros_ws/
```

```
 catkin_make
```

```
 cd ~/ros_ws/src/ 
```

```
git clone https://github.com/mfocchi/locosim.git
```

```
cd locosim
```

```
git submodule update --init --recursive
```



### Configure environment variables 

```
gedit  ~/.bashrc
```

copy the following lines (at the end of the .bashrc), remember to replace PYTHON_VERSION as explained in [here](#software-versions) :

```
source /opt/ros/kinetic/setup.bash
source $HOME/ros_ws/install/setup.bash
export PATH=/opt/openrobots/bin:$PATH
export LOCOSIM_DIR=$HOME/ros_ws/src/locosim
export PYTHONPATH=/opt/openrobots/lib/pythonPYTHON_VERSION/site-packages:$LOCOSIM_DIR/robot_control:$PYTHONPATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/opt/openrobots/share/
```

#### Compile the code

```
cd ~/ros_ws/ 
```

```
 catkin_make install 
```



### **Running the software** from Python IDE: Pycharm  

We recommend to use an IDE to run and edit the python files, like Pycharm community. To install it,  you just need to download and unzip the program:

https://download.jetbrains.com/python/pycharm-community-2021.1.1.tar.gz

 and unzip it  *inside* the docker (e.g. copy it inside the `~/trento_lab_home` folder. 

**IMPORTANT**!** I ask you to download this specific version (2021.1.1) that I am sure it works, because the newer ones seem to be failing to load environment variables! 

1) To run Pycharm community type (if you are lazy you can create an alias...): 

```
$ pycharm_folder/bin/pycharm.sh
```

2) remember to run **pycharm-community** from the terminal otherwise it does not load the environment variables loaded inside the .bashrc.

3) launch one of the labs in locosim/robot_control or in locosim/robot_control/base_controllers  (e.g. base_controller_fixed.py)

4) the first time you run the code be sure you selected the appropriate interpreter /usr/binpython3.8

**IMPORTANT!** To be able to keep the plots **alive** at the end of the program and to have access to variables,  you need to "Edit Configurations..." and tick "Run with Python Console". Otherwise the plot will immediately close. 



### Running the Software from terminal

To run from a terminal we  use the interactive option that allows  when you close the program have access to variables:

```
$ python3 -i $LOCOSIM_DIR/robot_control/base_controllers/base_controller.py
```

to exit from python3 console type CTRL+Z



### **Support for Universal Robots** (only for advanced users, it requires ros noetic)

```
sudo apt install ros-noetic-pass-through-controllers
```

```
sudo apt install ros-noetic-moveit-core
```

```
sudo apt install ros-noetic-moveit-kinematics
```

```
sudo apt install ros-noetic-ur-msgs
```

```
sudo apt install ros-noetic-ur-robots-driver
```

```
sudo apt install ros-noetic-speed-scaling-state-controller
```

```
sudo apt install ros-noetic-speed-scaling-interface
```

```
sudo apt install ros-noetic-speed-scaling-state-controller
```

```
sudo apt install ros-noetic-speed-scaling-interface
```

```
sudo apt install ros-noetic-ur-client-library
```

```
sudo apt install ros-noetic-scaled-joint-trajectory-controller
```

```
sudo apt install ros-noetic-pass-through-controllers
```



**Support for Realsense camera (Simulation)**

This packages are needed if you want to see the PointCloud published by a realsense camera attached at the endeffector. To activate it, you should load the ur5 with the flag "vision_sensor:=true"

```
sudo apt-get install ros-noetic-openni2-launch
```

```
sudo apt-get install ros-noetic-openni2-camera
```

```
sudo apt install ros-noetic-realsense2-description
```
