## em3242_angle_sensor_ros 

ROS interface for the EM3242 absolute angle sensor (connected via an Arduino). 


# Installation

1. Install ROS. Instructions can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) 
if you have not already done this.  Tested with ros kinetic, desktop install

2. Setup your catkin workspace.  Instructions can be found [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) 
if you have not already done this. 
    
3. Download and install the em3242_angle_sensor python library from [here](http://github.com/willdickson/em3242_angle_sensor)

3. Then clone the git repository and run catkin make.

```bash
$cd ~/catkin_ws/src
$git clone https://github.com/willdickson/em3242_angle_sensor_ros.git
$cd ~/catkin_ws
$catkin_make

```

# Launching the em3242 angle sensor node

```bash
$rosluanch em3242_angle_sensor_ros em3242_angle_sensor.launch
```

# EM3242_AngleSensorData.msg

```
Header header
float64 angle
float64 cumulative_angle 

```


