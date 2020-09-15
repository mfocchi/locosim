
# Install on Ubuntu 16 / Ubuntu 18
## Download submodules 
git submodule sync
git submodules update --init --recursive

## Install Spyder python IDE

sudo pip install -Iv spyder==2.3.9
sudo apt install python-pyside

### Pinocchio stuff

**add robotpkg repositories**

sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"
sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub $(lsb_release -sc) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list"

**Register the authentication certificate of robotpkg:**

sudo apt install curl
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -

sudo apt-get update
sudo apt install robotpkg-pinocchio
sudo apt install robotpkg-py27-pinocchio

### Install robot urdfs 

sudo apt-get install ros-kinetic-urdfdom-py
sudo apt install robotpkg-py27-example-robot-data

###  Python
sudo apt-get install python-scipy
sudo apt-get install python-matplotlib
sudo apt-get install robotpkg-py27-quadprog 

### Install ROS on ubuntu

sudo apt-get install ros-kinetic-desktop-full


### Rviz
sudo apt-get install ros-kinetic-joint_state_publisher

sudo apt-get install ros-kinetic-joint-state-controller 


### Setup ros workspace

1) mkdir -p ~/ros_ws/src
2) cd ~/ros_ws/
3) catkin_make
4) cd ~/ros_ws/src/ 
5) git clone https://github.com/mfocchi/locosim.git
6) git submodule update --init --recursive
7) cd ~/ros_ws/ 
8) catkin_make install  #this will create $HOME/ros_ws/install/setup.bash

#configure environment variables (add to .bashrc)
1) gedit  ~/.bashrc
2) copy the following lines:

source $HOME/ros_ws/install/setup.bash

export UR5_MODEL_DIR=/opt/openrobots/share/example-robot-data/robots

export PATH=/opt/openrobots/bin:$PATH

export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH

export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH

export PYTHONPATH=/opt/openrobots/lib/python2.7/site-packages:$PYTHONPATH

export LOCOSIM_DIR=$HOME/ros_ws/src/locosim

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/opt/openrobots/share/

3) step: source ~/.bashrc
4) sudo cp $LOCOSIM_DIR/robot_control/ur5_modified.urdf /opt/openrobots/share/example-robot-data/robots/ur_description/urdf/ur.urdf