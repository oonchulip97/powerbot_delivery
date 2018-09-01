# Overview
This is a high-level state machine that implements delivery mechanism onto the Powerbot.

This state machine was tested using:
- Ubuntu 16.04.3 Xenial
- ROS Kinetic
- Python 2.7.12
- Mozilla Firefox 54.0

# Setup
## Installation
Install SMACH to build hierarchical state machines.
```
cd <workspace>/src
git clone https://github.com/ros/executive_smach
```

Install SMACH viewer to show the state of hierarchical SMACH state machines.
```
cd <workspace>/src
https://github.com/ros-visualization/executive_smach_visualization
```

Install rosbridge to provide JSON API to ROS functionality.
```
sudo apt-get install ros-kinetic-rosbridge-server
```

Install roslibjs to communicate with rosbridge from the browser.
```
sudo apt-get install nodejs npm
cd <workspace>/src
git clone https://github.com/RobotWebTools/roslibjs.git
```

Install pip to install software packages written in Python. 
```
sudo apt-get install python-pip
```

Install utm to convert between UTM and WGS84 geocoordinates in Python.
```
pip install utm
```

Install lz4 to enable decompression of Firefox .jsonlz4 files.
```
pip install lz4
```

## Configuration
Within the powerbot_delivery_config.yaml file, modify the following parameters:

```
reload_path
```
This should be 

```
map_url
```
This should be the url when you open scripts/map.html in the browser.

There are other parameters within the configuration file that you should change as required.

# Running

Run the following in a terminal.
```
roslaunch powerbot_delivery powerbot_delivery.launch
```
This will load the configuration file onto the parameter server and run both the GPS filter and the state machine.

# Mechanism
## GPS Filter

## State Machine
