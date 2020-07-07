

## Download submodules
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

#### update
sudo apt-get update

sudo apt-get remove robotpkg-omniorb (omniORBpy should be on version 3 on Ubuntu 16.04 see https://github.com/humanoid-path-planner/hpp-rbprm-corba/issues/46)
sudo apt-get install python-omniorb
sudo apt install robotpkg-pinocchio
sudo apt install robotpkg-py27-pinocchio

**install robot urdfs**
sudo apt install robotpkg-py27-example-robot-data

**gepetto viewer** 

sudo apt-get install ros-kinetic-urdfdom-py
sudo apt-get install robotpkg-urdfdom
sudo apt update && sudo apt install robotpkg-py27-qt4-gepetto-viewer-corba

###  Python
sudo apt-get install python-matplotlib
sudo apt-get install robotpkg-py27-quadprog 

### rviz
sudo apt-get install ros-kinetic-joint_state_publisher

### Setup ros workspace

1) mkdir -p ~/ros_ws/src
2) cd ~/ros_ws/
3) catkin_make
4) cd ~/ros_ws/src/ 
5) git clone git@github.com:mfocchi/locosim.git
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