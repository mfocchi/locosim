

# Setup Unitree Go1 robot to work with Locosim

Locosim supports many robots in simulation: UR5, HyQ, Aliengo, Solo, Go1. Recently I managed to interface also two real platforms: Aliengo and Go1. Here I will show you the step to interface Go1 and make it run with Locosim.

1) connect your operator PC where you have installed Locosim to the Go1 robot switch via an Ethernet cable 

1) using your network configuration manager, set a fixed IP for your operator PC in the local network where the robot is connected 192.168.123.X for example:

```
192.168.123.15
```

with subnet mask 255.255.255.0 and gateway 192.168.123.1. The Main Control Board on the robot has the IP		

```
192.168.123.10
```

If you want to be able to sniff the packages coming from the Cameras on the robot you need to add this line to the .bashrc:

```
ROS_IP = 192.168.123.15
```

2) turn on the robot with the robot standing on its belly with all the legs close to the body and wait 1 minute. The robot will stand-up automatically.

3) press these buttons on the Joystick in sequence:

```
L2+A
```

locks the joints

```
L2+A
```

puts the robot in prone state (belly down)

```
L2+B
```

enters in damping state

```
L1+L2+Start		
```

Enters in external user command mode