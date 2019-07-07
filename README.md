# Home Service Robot

[//]: # (Image References)
[pickImg]: ./images/markeratpickuplocation.png "Marker at pickup location before Pick up"
[dropImg]: ./images/droplocation.png "Marker at dopoff location after robot reached"

The goal of this project is to program a robot than can autonomously map an environment and navigate to pick up and drop off virtual objects.
## Project setup

### Install xterm
```
sudo apt-get install xterm
```

### Create catkin workspace
```
$ mkdir -p ~/home_service/src
$ cd ~/home_service/src
$ catkin_init_workspace
$ cd ..
$ catkin_make
```
### Install Packages
```
$ cd ~/home_service/src
$ git clone https://github.com/ros-perception/slam_gmapping.git
$ git clone https://github.com/turtlebot/turtlebot.git
$ git clone https://github.com/turtlebot/turtlebot_interactions.git
$ git clone https://github.com/turtlebot/turtlebot_simulator.git
$ cd ~/home_service/
$ source devel/setup.bash
$ rosdep -i install gmapping
#All required rosdeps installed successfully
$ rosdep -i install turtlebot_teleop
#All required rosdeps installed successfully
$ rosdep -i install turtlebot_rviz_launchers
#All required rosdeps installed successfully
$ rosdep -i install turtlebot_gazebo
#All required rosdeps installed successfully
$ catkin_make
$ source devel/setup.bash
```
## Below is the catkin workspace structure
```
    ├──                                # Official ROS packages
    |
    ├── slam_gmapping                  # gmapping_demo.launch file
    │   ├── gmapping
    │   ├── ...
    ├── turtlebot                      # keyboard_teleop.launch file
    │   ├── turtlebot_teleop
    │   ├── ...
    ├── turtlebot_interactions         # view_navigation.launch file
    │   ├── turtlebot_rviz_launchers
    │   ├── ...
    ├── turtlebot_simulator            # turtlebot_world.launch file
    │   ├── turtlebot_gazebo
    │   ├── ...
    ├── map                            # map of the environment
    |   ├── ...
    ├── world                          # world files
    │   ├── ...
    ├── scripts                        # shell scripts files
    │   ├── home_service.sh
    |   ├── ...
    ├── rvizConfig                      # rviz configuration files
    │   ├── ...
    ├── pick_objects                    # pick_objects C++ node
    │   ├── src/pick_objects.cpp
    │   ├── ...
    ├── add_markers                     # add_marker C++ node
    │   ├── src/add_markers.cpp
    │   ├── ...
    └──
```
### Pick Objects

Created src/pick_objects.cpp node to pickup the virtual objects.

### Add Markers

Created src/add_markers.cpp node to display the virtual objects.

### Home Service

Created home_service.sh script which launches the turtelbot, amcl, rviz config file, pick objects and add_markers node. Below is the content of the shell script
```
#!/bin/sh
xterm  -e  " roslaunch -v turtlebot_gazebo turtlebot_world.launch " &
sleep 15
xterm  -e  " roslaunch -v turtlebot_gazebo amcl_demo.launch " & 
sleep 5
xterm  -e  " roslaunch -v turtlebot_rviz_launchers view_navigation.launch " &
sleep 15
xterm  -e  " rosrun add_markers add_markers " &
sleep 5
xterm  -e  " rosrun pick_objects pick_objects "
```

### Run

Run ./home_service.sh in Scripts directory to deploy the home service robot.

### Results  

#### Robot navigating towards virtual object

![alt text][pickImg]  

#### Robot successfully delivered the virtual object

![alt text][dropImg]  
