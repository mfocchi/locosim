# Using ROS in multiple machines with heterogeneous networks			

ROS has the important capability to allow the node to be running on different machines and networks. 

The only requirement is that the rosmaster node (i.e. roscore) runs in only one machine. You need to take care that in that specific machine (let's assume that it has an IP: 10.196.80.36) the environment variable  ROS_MASTER_URI is set as follows (you can put it in the .bashrc for permanent changes):

```
export ROS_MASTER_URI=http://localhost:11311
```

In all the other machines that variable should be set with the IP of the machine where the roscore is running, (e.g. in our example 10.196.80.36):

```
export ROS_IP=http://IP_OF_ROSCORE_MACHINE:11311
```

This tells everbody where the rosmaster is running. Additionally, for all the machines, you need to set the variable ROS_IP to the IP of that specific machine:

```
export ROS_MASTER_URI=IP_OF_LOCAL_MACHINE
```

this allows the machine to be uniquely recognized across the ROS network.  Note that the machines should not necessarily be on the same subnet network! they can be connected physically in different ways (e.g. LAN, WIFI) or on different subnetworks! For example let's assume we want to connect a mobile robot with an UR5 manipulator to design a handing-over task. 

Let's assume that, for some reason, we want to run the roscore on the computer **onboard** the mobile robot. Let's assume that the onboard computer is connected to a laptop computer via WIFI  and that the laptop computer, on his behalf, is physically connected to the UR5 robot via an ethernet cable. Let's assume that the WIFI connection is on a subnet 10.196.80.X while the ethernet one is on 192.168.0.X. In this case,  the settings should be as in the following Figure:

![network settings](https://github.com/mfocchi/lab-docker/blob/master/figs/network_settings.png)
