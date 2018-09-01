# Overview
This is a high-level state machine that implements delivery mechanism onto the Powerbot.

This state machine was tested using:
- Ubuntu 16.04.3 Xenial
- ROS Kinetic
- Python 2.7.12
- Mozzila Firefox 54.0

# Setup
Install rosbridge
```
sudo apt-get install ros-kinetic-rosbridge-server
```

Install nodejs
```
sudo apt-get install nodejs
sudo apt-get install npm
```



# Running

Run the following in a terminal.
```
roslaunch powerbot_delivery powerbot_delivery.launch
```
This will load the configuration file onto the parameter server and run both GPS filter and the state machine.

# Mechanism
## GPS Filter

## State Machine
