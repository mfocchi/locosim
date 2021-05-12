



# Usage with a Virtual Machine

Download the following [virtual machine](https://www.dropbox.com/sh/5trh0s5y1xzdjds/AACchznJb7606MbQKb6-fUiUa) (made for VirtualBox) and run the lab experiments that are present in  **robot_control/lab_descriptions**



# Install on Ubuntu 16 / Ubuntu 18

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
source $HOME/ros_ws/install/setup.bash
export UR5_MODEL_DIR=/opt/openrobots/share/example-robot-data/robots
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python2.7/site-packages:$PYTHONPATH
export LOCOSIM_DIR=$HOME/ros_ws/src/locosim
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/opt/openrobots/share/
```

#### Compile the code

```
cd ~/ros_ws/ 
```

```
 catkin_make install 
```





