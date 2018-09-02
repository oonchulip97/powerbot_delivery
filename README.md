# Overview
This is a high-level state machine that implements delivery mechanism onto the Powerbot.

This state machine was tested using:
- Ubuntu 16.04.5 Xenial
- ROS Kinetic
- Python 2.7.12
- Mozilla Firefox 61.0.1

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
This should be the path to the recovery.jsonlz4 file located within .mozilla directory.

Typically, it is `/home/<user>/.mozilla/firefox/<alphanumeric characters>.default/sessionstore-backups/recovery.jsonlz4`

```
map_url
```
This should be the url when you open scripts/map.html in the browser.

Typically, it is `file:///home/<user>/<workspace>/src/powerbot_delivery/scripts/map.html`

There are other parameters within the configuration file that you should change as needed.

# Running

Run the following in a terminal.
```
roslaunch powerbot_delivery powerbot_delivery.launch
```
This will load the configuration file onto the parameter server and run both the GPS filter and the state machine.

# Mechanism
## GPS Filter
<img src="/images/GPS_filter_flow_diagram.jpg"  width="500">

A flow diagram of the GPS filter is shown above. In between the interval which the filter receives GPS signal, geocoordinates of the robot can be intrapolated using odometry and IMU. Predicted geocoordinates are fed to the state machine to indicate the current position of the robot. Once the filter receives GPS signal, geocoordinates of the robot can be updated. The raw GPS signal is averaged using moving average filter to reduce volatility. Complementary filter takes both predicted geocoordinates and averaged GPS signal as inputs and produces an estimated geocoordinates as output. The lower the covariance of the raw GPS signal, the more skewed the estimated geocoordinates towards the averaged GPS signal and away from the predicted geocoordinates is, and vice versa. Estimated geocoordinates are then fed to the state machine to indicate the current position of the robot.

## State Machine
 <img src="/images/state_machine_flow_diagram.jpg"  width="500">

A flow diagram of the state machine is shown above. After the user selects a delivery location, a path is computed by Google Maps' API with waypoints located at each intersection between the robot's initial location and the delivery location. The state machine will continuously check for the robot's arrival at a given waypoint. Only if the robot has not reached the waypoint yet, then the waypoint will be send to the low-level motor controller cyclically. However, if the robot has reached within a predefined tolerance of the waypoint, the state machine will check if there are more waypoints. If more waypoints exists, the current waypoint is updated with the next waypoint. Otherwise, if no more waypoints exist, it indicates that the robot has reached its goal, and the navigation is completed.
