

# What is Locosim?

Locosim is a didactic framework to learn/test basic controllers schemes on quadruped robots (HyQ/Solo/Aliengo/Go1 are supported) and manipulators (UR5  is supported). Locosim has been successfully tested on Ur5, Aliengo and Go1 robots and (to my knowledge) is the first Open-Source Easy-to-Use interface to the new Unitree Quadruped robot Go1. If you just bought a Go1 robot and you want to give a try, follow this [wiki](https://github.com/mfocchi/locosim/blob/develop/figs/go1_setup.md)!

Locosim is composed by a **roscontrol** node called **ros_impedance_controller** (written in C++) that interfaces a python ROS node (where the controller is written) to a Gazebo simulator. All the didactic labs have a description, with exercises of increasing complexity, in the folder **lab_descriptions** inside robot_control submodule. For each controller, plotting / logging utilities are available to evaluate the results together with a configuration file (LX_conf.py) to change the controller parameters. 

You have 3 ways to get the Locosim code: 1) with a virtual machine 2) with docker 3) by manual installation of dependencies.

**Note**: If you intend to use Locosim for your *research* please cite:

- M. Focchi, F. Roscia, C. Semini, **Locosim: an Open-Source Cross-Platform Robotics Framework**, Synergetic Cooperation between Robots and Humans. CLAWAR, 2023.  

  you can download a pre-print of the paper [here](https://iit-dlslab.github.io/papers/focchi23clawar.pdf). [View BibTeX](https://github.com/mfocchi/locosim/blob/develop/locosim.bib)

# Usage with a Virtual Machine

First install the free version (16.0) of the software VMWare Player (the "player" keyword is for free personal use in the VMware software). For Linux / Windows system you can find it  [here](https://www.vmware.com/products/workstation-player.html) for MAC  [here](https://customerconnect.vmware.com/en/evalcenter?p=fusion-player-personal-13) (you need to create an account, I am sorry...).

Then, download the following [virtual machine](https://www.dropbox.com/scl/fo/tjwfcjwnenqgtsakohdfy/h?dl=0&rlkey=lg1fdkn6k0thveg4efuwoh3kn) and run the file **VM ROBO.vmx** to open it. You  will have now a fully working Ubuntu 20 system with all the needed dependencies, code and Pycharm IDE already installed. Check README.txt for the password. The lab experiments that are present in: **robot_control/lab_exercises** you can find a detailed description of them in **robot_control/lab_exercises/lab_descriptions**. 

**IMPORTANT NOTE!** To be able to update the submodules you need to generate you SSH key inside the virtual machine a store it in your Github account as explained  [here](https://github.com/mfocchi/lab-docker/blob/master/install_docker.md#installing-git-and-ssh-key).

**COMPATIBILITY ISSUES:** This virtual machine works with x86-64 processors, but not for new MAC M1/M2 that employ ARM  processors.



# Usage with Docker

You can alternatively use a docker image that contains Ubuntu 20 and all the required dependencies already installed (you will need only to clone the code and compile it) by following this  [wiki](https://github.com/mfocchi/lab-docker). 

**WINDOWS:** follow this procedure: https://github.com/mfocchi/lab-docker/blob/master/install_docker_windows.md

**MAC:** follow this  [wiki](https://github.com/mfocchi/lab-docker), just replace **"sudo apt install package_name"** with **"brew install package_name"**.

**LINUX:** follow this  [wiki](https://github.com/mfocchi/lab-docker). 



# Native Installation 

**WINDOWS:** Install Ubuntu 20.4.06 LTS  following this procedure: https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support#1-overview

If you experiment any issue in using the Nvidia with  OpenGL rendering (the symptom is that you cannot visualize STL meshes in RVIZ) then you should update to the latest mesa-driver:

```
sudo add-apt-repository ppa:kisak/kisak-mesa
sudo apt update
sudo apt install mesa-utils
```

**MAC:** follow the next procedure, just replace **"sudo apt install package_name"** with **"brew install package_name"**.

**LINUX:** follow the next procedure.



### UBUNTU VERSIONS:

Locosim is compatible with Ubuntu 18/20. The installation instructions have been generalized accordingly. You need replace four strings (PYTHON_PREFIX, PYTHON_VERSION, PIP_PREFIX, ROS_VERSION) with the appropriate values according to your operating systems as follows:

| Ubuntu 18:                   | **Ubuntu 20**:               |
| ---------------------------- | ---------------------------- |
| PYTHON_PREFIX = python3      | PYTHON_PREFIX = python3      |
| PYTHON_VERSION = 3.5         | PYTHON_VERSION = 3.8         |
| ROBOTPKG_PYTHON_VERSION=py35 | ROBOTPKG_PYTHON_VERSION=py38 |
| PIP_PREFIX = pip3            | PIP_PREFIX = pip3            |
| ROS_VERSION = bionic         | ROS_VERSION = noetic         |

**NOTE:** ROS is no longer supported (only ROS2 Humble) on Ubuntu 22 hence is not possible to install Locosim on Ubuntu 22.

### Install ROS 

setup your source list:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up your keys:

```
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
```

install ROS main distro:

```
sudo apt-get install ros-ROS_VERSION-desktop-full
```

install ROS packages:

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

```
sudo apt install ros-ROS_VERSION-joint-trajectory-controller
```

### **Support for Realsense camera (simulation)**

This packages are needed if you want to see the PointCloud published by a realsense camera attached at the endeffector. To activate it, you should load the xacro of the ur5 with the flag "vision_sensor:=true". 

```
sudo apt-get install ros-ROS_VERSION-openni2-launch
```

```
sudo apt-get install ros-ROS_VERSION-openni2-camera
```

```
sudo apt install ros-ROS_VERSION-realsense2-description
```

### **Support to simulate Grasping**

Unfortunately grasping in Gazebo is still an open issue, I impelented grasping using this [plugin]( https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Installation) that creates a fixed link between the gripper and the object to be grasped. To activate the grasping plugin set gripper_sim parameter to True in your configuration file. The following dependencies are required:

```
sudo apt-get install ros-ROS_VERSION-eigen-conversions 
```

```
sudo apt-get install ros-ROS_VERSION-object-recognition-msgs
```

```
sudo apt install ros-ROS_VERSION-roslint
```

You can check which parameters have to be tuned looking to the following [wiki]( https://github-wiki-see.page/m/JenniferBuehler/gazebo-pkgs/wiki/The-Gazebo-grasp-fix-plugin) 



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
sudo apt install -qqy lsb-release gnupg2 curl
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
sudo apt-get install PYTHON_PREFIX-matplotlib
```

```
sudo apt-get install PYTHON_PREFIX-termcolor
```

```
sudo apt install python3-pip
```

```
PIP_PREFIX install cvxpy==1.2.0
```



### Download code and setup ROS workspace

Now that you installed all the dependencies you are ready to get the code, but first you need to create a ros workspace to out the code in:

```
mkdir -p ~/ros_ws/src
```

```
cd ~/ros_ws/src
```

Now you need to call the following line manually (next you will see that it will be done automatically in the .bashrc)

```
source /opt/ros/ROS_VERSION/setup.bash
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

remember to update its submodules  (robot_control and ros_impedance_controller) running this command in the locosim root:

```
git submodule update --init --recursive
```

**IMPORTANT NOTE!** you will not be able to checkout the submodules unless you generate and add your SSH key to your Github account, as explained [here](https://github.com/mfocchi/lab-docker/blob/master/install_docker.md#installing-git-and-ssh-key).

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

There are some additional utilities that I strongly suggest to install. You can find the list  [here](https://github.com/mfocchi/locosim/blob/develop/figs/utils.md).



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

the .bashrc is a file that is **automatically** sourced whenever you open a new terminal.

**NOTE**: people with some operating systems like ARCH LINUX, might need to add "export ROS_IP=127.0.0.1" to the .bashrc.

#### Compile/Install the code

Whenever you modify some of the ROS packages (e.g. the ones that contain the xacro fles inside the robot_description folder), you need to install them to be sure they are been updated in the ROS install folder. 

```
cd ~/ros_ws/ 
```

```
 catkin_make install 
```

**IMPORTANT!**

The first time you compile the code the install folder is not existing, therefore won't be added to the PYTHONPATH with the command **source $HOME/ros_ws/install/setup.bash**, and you won't be able to import the package ros_impedance_controller. Therefore, **only once**, after the first time that you compile, run again :

```
source .bashrc
```



### **Running the software** from Python IDE: Pycharm  

Now that you compiled the code you are ready to run the software! 

We recommend to use an IDE to run and edit the python files, like Pycharm community. 

1. To install it, enter in the $HOME folder of the docker and download it from here:

```
$ wget https://download.jetbrains.com/python/pycharm-community-2021.1.1.tar.gz
```

2. Then, unzip the program:

```
$tar -xf pycharm-community-2021.1.1.tar.gz
```

 and unzip it  *inside* the docker (e.g. copy it inside the `~/trento_lab_home` folder. 

**IMPORTANT**!** I ask you to download this specific version (2021.1.1) that I am sure it works, because the newer ones seem to be failing to load environment variables! 

3. To run Pycharm community type (if you are lazy you can create an alias...): 

```
$ pycharm-community-2021.1.1/bin/pycharm.sh
```

Running pycharm from the terminal enables to use the environment variables loaded inside the .bashrc.

4. click "Open File or Project" and open the folder robot_control. Then launch one of the labs in locosim/robot_control/lab_exercises or in locosim/robot_control/base_controllers  (e.g. ur5_generic.py)  right click on the code and selecting "Run File in Pyhton Console"

5. the first time you run the code you will be suggested to select the appropriate interpreter (/usr/binpython3.8). Following this procedure you will be sure that the run setting will be stored, next time that you start Pycharm.



### Running the Software from terminal

To run from a terminal we  use the interactive option that allows  when you close the program have access to variables:

```
$ python3 -i $LOCOSIM_DIR/robot_control/base_controllers/base_controller.py
```

to exit from python3 console type CTRL+Z



## Using the real robots: 

These packages are needed if you are willing to do experiments with the **real** robots

### **Universal Robot UR5**

The driver for the UR5 has already been included in Locosim but is not compiled by default, hence you need to:

1. remove file [CATKIN_IGNORE](https://github.com/mfocchi/universal_robots_ros_driver/blob/master/CATKIN_IGNORE) inside the **ur_driver** package 
2. remove file [CATKIN_IGNORE]( https://github.com/mfocchi/zed_wrapper/blob/af3750a31c1933d4f25b0cb9d5fc4de657d62001/CATKIN_IGNORE) inside the **zed_wrapper** package 
3. Install these additional packages:

```
sudo apt install ros-ROS_VERSION-ur-msgs
```

```
sudo apt install ros-ROS_VERSION-speed-scaling-interface
```

```
sudo apt install ros-ROS_VERSION-scaled-joint-trajectory-controller
```

```
sudo apt install ros-ROS_VERSION-industrial-robots-status-interface
```

```
sudo apt install ros-ROS_VERSION-speed-scaling-state-controller
```

```
sudo apt install ros-ROS_VERSION-ur-client-library
```

```
sudo apt install ros-ROS_VERSION-pass-through-controllers
```

4. recompile with **catkin_make install**.
5. add the following alias to your .bashrc

```
launch_robot='roslaunch ur_robot_driver ur5e_bringup.launch headless_mode:=true robot_ip:=192.168.0.100 kinematics_config:=$LOCOSIM_DIR/robot_hardware_interfaces/ur_driver/calibration_files/my_robot_calibration_X.yaml'
```

where X is {1,2}. For the robot with the soft gripper X = 2. 

4. Trigger the robot workbench power switch on  and press the on button on the UR5 Teach Pendant

5. Connect the Ethernet cable to the lab laptop and create a local LAN network where you set the IP of your machine to 192.168.0.101 (the robot IP will be 192.168.0.100, double check it using the UR5 "Teach Pendant"), see [network settings](https://github.com/mfocchi/locosim/blob/develop/network_settings.md) to setup the network in order to run together with a mobile platform.

6. Verify that you can ping the robot:

   ```
   ping 192.168.0.100
   ```

7. Where it says "Spegnimento" activate the robot pressing "Avvio" twice until you see all the 5 green lights and you hear the release of the brakes. Set the "Remote Control" in the upper right corner of the "Teach Pendant"

8. Run the **launch_robot** alias to start the **ur_driver**. If you want to start the driver without the ZED camera (e.g. you do not have CUDA installed), append **vision_sensor:=false** to the command. 

   ```
   launch_robot vision_sensor:=false
   ```

   Conversely, if you want to launch only the ZED camera alone and see the data in rviz:

   ```
   roslaunch zed_wrapper zed2.launch rviz:=true
   ```

9. Run the **ur5_generic.py** with the  [real_robot](https://github.com/mfocchi/robot_control/blob/2e88a9a1cc8b09753fa18e7ac936514dc1d27b8d/lab_exercises/lab_palopoli/params.py#L30) flag set to **True**. The robot will move to the home configuration defined  [here](https://github.com/mfocchi/robot_control/blob/babb5ff9ad09fec32c7ceaeef3d02715c6d067ab/lab_exercises/lab_palopoli/params.py#L26).

10. For the usage and calibration of the ZED camera follow this instructions [here](https://github.com/mfocchi/locosim/blob/develop/figs/zed_calibration.md).



**Universal Robots  + Gripper**

You have 2 kind of gripper available in the ur5 robot: a rigid 3 finger gripper and a soft 2 finger gripper.  Locosim seamlessly allows you to deal with both of them. By default, the 3 finger gripper is enabled, if you want to use the 2 finger one (soft_gripper)  you need to:

1. set the  [soft_gripper](https://github.com/mfocchi/robot_control/blob/fde3b27884819e1b2ea319fe5b2781a86d33a648/lab_exercises/lab_palopoli/params.py#L33) flag to True (in Simulation)

2. append **soft_gripper:=true** to the  **launch_robot** alias (on the real robot).

   

**Universal Robots  + ZED camera support**

If you are willing to use the real ZED Camera on the real robot, the zed_wrapper package is not compiled by default, hence you need to:

1. remove the file [CATKIN_IGNORE]( https://github.com/mfocchi/zed_wrapper/blob/af3750a31c1933d4f25b0cb9d5fc4de657d62001/CATKIN_IGNORE)  in the **zed_wrapper** package 

2. Install CUDA (locosim is compatible only with version 12): https://developer.nvidia.com/cuda-downloads

3. Install the  SDK library (locosim is compatible only with version 4.0.2) in: https://www.stereolabs.com/developers/release/

4. Recompile with catkin_make install

If you have issues remove the build/devel folder and recompile.



### Go1 Quadruped Robot

```
sudo apt-get install apt-get install liblcms2-2
```

```
sudo apt-get install apt-get install liblcms-bin
```

and follow this [wiki](https://github.com/mfocchi/locosim/blob/develop/go1_setup.md)!



### Tips and Tricks 

1) Most of virtual machines including Virtualbox, do not have support for GPU. This means that if you run Gazebo Graphical User Interface (GUI) it can become very **slow**. A way to mitigate this is to avoid to start the  Gazebo GUI and only start the gzserver process that will compute the dynamics, you will keep the visualization in Rviz. This is referred to planners that employ BaseController or BaseControllerFixed classes. In the Python code where you start the simulator you need to pass this additional argument as follows:

```
additional_args = 'gui:=false'
p.startSimulator(..., additional_args =additional_args)
```

2) Another annoying point is the default timeout to kill Gazebo that is by default very long. You can change it (e.g. to 0.1s) by setting the  _TIMEOUT_SIGINT = 0.1 and _TIMEOUT_SIGTERM = 0.1:

```
sudo gedit /opt/ros/ROS_VERSION/lib/PYTHON_PREFIX/dist-packages/roslaunch/nodeprocess.py
```

 this will cause ROS to send a `kill` signal much sooner.

3) if you get this annoying warning: 

```
Warning: TF_REPEATED_DATA ignoring data with redundant timestamp for frame...
```

a dirty hack to fix it is to clone this repository in your workspace:

```
git clone --branch throttle-tf-repeated-data-error git@github.com:BadgerTechnologies/geometry2.git
```

Note: you need a specific version of setuptools to be able to succesfully install the repo

```
pip3 install setuptools==45.2.0
```



