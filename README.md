# Overview
This is a high-level state machine that implements delivery mechanism onto the Powerbot.

This state machine was tested using:
- Ubuntu 16.04.3 Xenial
- ROS Kinetic
- Python 2.7.12
- Mozzila Firefox 54.0

# Setup
## Installation
Install SMACH.
```
cd <workspace>/src
git clone https://github.com/ros/executive_smach
```

Install SMACH viewer.
```
cd <workspace>/src
https://github.com/ros-visualization/executive_smach_visualization
```

Install rosbridge.
```
sudo apt-get install ros-kinetic-rosbridge-server
```

Install nodejs and npm.
```
sudo apt-get install nodejs
sudo apt-get install npm
```

Install pip.
```
sudo apt-get install python-pip
```

Install utm.
```
pip install utm
```

Install lz4.
```
pip install lz4
```

## Configuration





# Running

Run the following in a terminal.
```
roslaunch powerbot_delivery powerbot_delivery.launch
```
This will load the configuration file onto the parameter server and run both GPS filter and the state machine.

# Mechanism
## GPS Filter

## State Machine
