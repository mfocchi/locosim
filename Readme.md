# What is locosim?

Locosim is  didactic framework to learn/test basic controllers schemes on quadrupeds (HyQ/Solo robots are supported) and manipulators (UR5 robot is supported).

Locosim is composed by a **roscontrol** node called ros_impedance_controller written in C++ that interfaces the python ros node (where the controller is written) to a Gazebo simulator. All the didactic labs have a description, with exercises of increasing complexity, in the folder **lab_descriptions** inside robot_control submodule. For the controller plotting / logging utilities are available to evaluate the results and a config file (LX_conf.py) to change the controller parameters. 

# Usage with a Virtual Machine

Download the following [virtual machine](https://www.dropbox.com/sh/5trh0s5y1xzdjds/AACchznJb7606MbQKb6-fUiUa) (made for VirtualBox) and run the lab experiments that are present in  **robot_control/lab_descriptions**. Note that there are 2 Virtual machines available, one for Ubuntu 16 and one for Ubuntu 20. I strongly recommend to use the Ubuntu 20 one, because all the files base_controllerXX.py files are no longer compatible with Ubuntu 16. 



# Installation on Ubuntu 16 / Ubuntu 20

IMPORTANT NOTE: 

In the following commands, if you are installing for Ubuntu 16, you need to consider **kinetic** as  ROS version, **py27**, and **pip**, for Ubuntu 20, consider **noetic** as ROS version, **py38**  and **pip3**.

First clone the repository inside a ros workspace, then remember to update its submodules  (robot_control and ros_impedance_controller) running this command in the locosim root:

```
git submodule update --init --recursive
```

Finally you need to compile the C++ code running in your ros workspace 

```
catkin_make install
```

## Install Spyder python IDE

```
sudo pip install -Iv spyder==2.3.9
```

```
sudo apt install python-pyside
```



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

copy the following lines (at the end of the .bashrc!):

```
source /opt/ros/kinetic/setup.bash
source $HOME/ros_ws/install/setup.bash
export PATH=/opt/openrobots/bin:$PATH
export LOCOSIM_DIR=$HOME/ros_ws/src/locosim
export PYTHONPATH=/opt/openrobots/lib/python2.7/site-packages:$LOCOSIM_DIR/robot_control:$PYTHONPATH
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/opt/openrobots/share/
```

#### Compile the code

```
cd ~/ros_ws/ 
```

```
 catkin_make install 
```



### **Running the software** from terminal

You can execute a python script directly from the terminal using the following command (according to the version of python 2.7 or 3.X you have):

```
python/python3 script_name.py
```

The scripts are located or in the robot_control root or in the robot_control/base_controller folder

If you want to keep interacting with the interpreter after the execution of the script use the following command:

```
python3 -i script_name.py
```

Rather than running scripts from the terminal, it is more convenient to use a customized python editor. For this class we suggest you use the software "spyder/spyder3" or pycharm (recommended).



### **Running the software** from Python IDE: Pycharm  

With Pycharm you just need to download and unzip the program here https://www.jetbrains.com/pycharm/download/download-thanks.html?platform=linux&code=PCC and unzip it. To run type: 

```
pycharm_folder/bin/pycharm.sh
```

**IMPORTANT!** To be able to keep the plots **alive** at the end of the program and to have access to variables,  you need to "Edit Configurations..." and tick "Run with Python Console". Otherwise the plot will immediately close.



### **Running the software** from Python IDE: Spyder 

You should run Spyder from the terminal by simply typing:

```
spyder/spyder3
```

Once spyder is open, you can use "File->Open" to open a python script, and then click on the "Run file" button (green "play" shape) to execute the script. The first time that you run a script in spyder, you must set up the configuration options. In particular, you must choose the type of console between these 3 options:

1. current console
2. dedicated console
3. external system terminal

Typically option 1 (which is the default choice) does not work, so you should use either option 2 or 3. I typically use option 2, but option 3 is fine as well. If you have already run a file on spyder3 and you want to change the console to use, you can do it via the menu "*Run -> Configuration per file*".

Side note: depending on your OS version, option 2 and/or option 3 also allow you to check the option "*Interact with the Python console after execution*", which is useful to explore the value of the script variables after the execution has ended.

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
