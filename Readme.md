

# What is locosim?

Locosim is  didactic framework to learn/test basic controllers schemes on quadrupeds (HyQ/Solo robots are supported) and manipulators (UR5 robot is supported).

Locosim is composed by a **roscontrol** node called ros_impedance_controller written in C++ that interfaces the python ros node (where the controller is written) to a Gazebo simulator. All the didactic labs have a description, with exercises of increasing complexity, in the folder **lab_descriptions** inside robot_control submodule. For the controller plotting / logging utilities are available to evaluate the results and a config file (LX_conf.py) to change the controller parameters. 

# Usage with a Virtual Machine

Download the following [virtual machine](https://www.dropbox.com/sh/5trh0s5y1xzdjds/AACchznJb7606MbQKb6-fUiUa) (made for VirtualBox) for Ubuntu 20 and run the lab experiments that are present in: **robot_control/lab_exercises** you can find a detailed description of them in **robot_control/lab_exercises/lab_descriptions**. The virtual machine contains **both** the code and the required dependencies already installed.

##### **IMPORTANT NOTE:** 

Most of virtual machines including Virtualbox, do not have support for GPU. This means that if you run Gazebo Graphical User Interface (GUI) it can become very **slow**. A way to mitigate this is to avoid to start the  Gazebo GUI and only start the gzserver process that will compute the dynamics, you will keep the visualization in Rviz. This is referred to planners that employ BaseController or BaseControllerFixed classes. In the Python code where you start the simulator you need to pass this additional argument as follows:

```
additional_args = 'gui:=false'
p.startSimulator(..., additional_args =additional_args)
```



# Usage with Docker

You can alternatively use a docker image with Ubuntu 20 and all the required dependencies already installed (you will need only to clone the code and compile it), by following this  [wiki](https://github.com/mfocchi/lab-docker). Docker has been tested to work with windows machines, Linux and old MACs (not ARM processors). 



# Native Installation on LINUX

### SOFTWARE VERSIONS:

Locosim is compatible with Ubuntu 16/18/20. The installation instructions have been generalized accordingly. You need replace four strings (PYTHON_PREFIX, PYTHON_VERSION, PIP_PREFIX, ROS_VERSION) with the appropriate values according to your operating systems as follows:

| **Ubuntu 16**:               | Ubuntu 18:                   | **Ubuntu 20**:               |
| ---------------------------- | ---------------------------- | ---------------------------- |
| PYTHON_PREFIX = python       | PYTHON_PREFIX = python3      | PYTHON_PREFIX = python3      |
| PYTHON_VERSION = 2.7         | PYTHON_VERSION = 3.5         | PYTHON_VERSION = 3.8         |
| ROBOTPKG_PYTHON_VERSION=py27 | ROBOTPKG_PYTHON_VERSION=py35 | ROBOTPKG_PYTHON_VERSION=py38 |
| PIP_PREFIX = pip             | PIP_PREFIX = pip3            | PIP_PREFIX = pip3            |
| ROS_VERSION = kinetic        | ROS_VERSION = bionic         | ROS_VERSION = noetic         |



### Install ROS 

setup your source list:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up your keys:

```
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
```

install ROS:

```
sudo apt-get install ros-ROS_VERSION-desktop-full
```

install packages:

```
sudo apt-get install ros-ROS_VERSION-urdfdom-py
```

```
sudo apt-get install ros-ROS_VERSION-srdfdom
```

```
sudo apt-get install ros-ROS_VERSION-joint-state-publisher
```

```
sudo apt-get install ros-ROS_VERSION-joint-state-publisher-gui
```

```
sudo apt-get install ros-ROS_VERSION-joint-state-controller 
```

```
sudo apt-get install ros-ROS_VERSION-gazebo-msgs
```

```
sudo apt-get install ros-ROS_VERSION-control-toolbox
```

```
sudo apt-get install ros-ROS_VERSION-gazebo-ros
```

```
sudo apt-get install ros-ROS_VERSION-controller-manager
```



### Install robot urdfs 

```
sudo apt install robotpkg-PINOCCHIO_PYTHON_VERSION-example-robot-data
```



### Pinocchio stuff

**Add robotpkg as source repository to apt:**

```
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
```

```
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub $(lsb_release -sc) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
```

**Register the authentication certificate of robotpkg:**

```
sudo apt -qqy lsb-release gnupg2 curl
```

```
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -
```

You need to run at least once apt update to fetch the package descriptions:

```
sudo apt-get update
```

Now you can install Pinocchio and the required libraries:

```
sudo apt install robotpkg-PINOCCHIO_PYTHON_VERSION-eigenpy	
```

```
sudo apt install robotpkg-PINOCCHIO_PYTHON_VERSION-pinocchio
```

```
sudo apt-get install robotpkg-PINOCCHIO_PYTHON_VERSION-quadprog  
```

**NOTE:** If you have issues in installing robotpkg libraries you can try to install them through ROS as:

```
sudo apt-get install ros-ROS_VERSION-LIBNAME
```



###  Python

```
sudo apt-get install PYTHON_PREFIX-scipy
```

```
sudo apt-get install PYTHON_PREFIX-matplotlib=2.0.2
```

```
sudo apt-get install PYTHON_PREFIX-termcolor
```

```
PIP_PREFIX install cvxpy==1.2.0
```



### Download code and setup ROS workspace

```
mkdir -p ~/ros_ws/src
```

```
cd ~/ros_ws/src
```

```
catkin_init_workspace
```

This will create "/opt/ros/ROS_VERSION/setup.bash" that you add to your .bashrc with the following:

```
echo "source /opt/ros/ROS_VERSION/setup.bash" >> ~/.bashrc
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

Now you can clone the repository inside the ROS workspace you just created:

```
git clone https://github.com/mfocchi/locosim.git
```

**IMPORTANT NOTE!** you will not be able to checkout the submodules unless you generate and add your SSH key to your Github account, as explained here:

https://github.com/mfocchi/lab-docker/blob/master/install_docker.md#installing-git-and-ssh-key

remember to update its submodules  (robot_control and ros_impedance_controller) running this command in the locosim root:

```
git submodule update --init --recursive
```

now recompile again (then this step won't bee needed anymore if you just work in python unless you do not modify / create additional ROS packages)

```
cd ~/ros_ws/ 
```

```
 catkin_make install
```

the install step install the ros packages inside the "$HOME/ros_ws/install" folder rather than the devel folder. This folder will be added to the ROS_PACKAGE_PATH instead of the devel one.

Finally, run (you should do it any time you add a new ros package)

```
 rospack profile
```



### Configure environment variables 

```
gedit  ~/.bashrc
```

copy the following lines (at the end of the .bashrc), remember to replace the string PYTHON_VERSION with the appropriate version name as explained in [software versions](#software-versions) section:

```
source /opt/ros/ROS_VERSION/setup.bash
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

**IMPORTANT**! I ask you to download this specific version (2021.1.1) that I am sure it works, because the newer ones seem to be failing to load environment variables! 

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



### **Support for Universal Robots**

These packages are needed if you are willing to perform simulations with the Ur5 robot:

```
sudo apt install ros-ROS_VERSION-joint-trajectory-controller
```

If you want to do experiments with the real robot you need to install these additional packages:

```
sudo apt install ros-ROS_VERSION-ur-msgs
```

```
sudo apt install ros-ROS_VERSION-scaled-joint-trajectory-controller
```

plus clone the ur_robot_driver package wherever inside the ros_ws/src folder:

```
git clone git@github.com:mfocchi/universal_robots_ros_driver.git
```



**Support for Realsense camera (simulation)**

This packages are needed if you want to see the PointCloud published by a realsense camera attached at the endeffector. To activate it, you should load the xacro of the ur5 with the flag "vision_sensor:=true". 

```
sudo apt-get install ros-noetic-openni2-launch
```

```
sudo apt-get install ros-noetic-openni2-camera
```

```
sudo apt install ros-noetic-realsense2-description
```
