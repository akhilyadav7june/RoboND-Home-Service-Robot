# Home Service Robot

[//]: # (Image References)
[pickImg]: ./images/t1p1.png "Marker at pickup location before Pick up"
[dropImg]: ./images/t1p2.png "Marker at dopoff location after robot reached"

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
Needs to have run permissions.
```
$ chmod +x src/pick_objects.sh
```
Run pick_objects.sh to run two step navigation.

```
$ ~/catkin_ws/src/pick_objects.sh
```

### Add Markers
Needs to have run permissions.
```
$ chmod +x src/add_markers.sh
```
Run add_markers.sh to add and remove a marker on the pick up and drop off locations.

```
$ ~/catkin_ws/src/add_markers.sh
```
Note: The last version works but not as the Project part 11 Modeling Virtual Objects, since the logic has changed, the robot is required to reach to the marker to continue. 

### Home Service
Needs to have run permissions.
```
$ chmod +x src/home_service.sh
```

Run home_service.sh to pick and drop marker to specified locations.
```
$ ~/catkin_ws/src/home_service.sh
```
### Run

Run ./home_service.sh in Scripts directory to deploy the home service robot.

### Results  

#### Robot navigating towards virtual object

![alt text][pickImg]  

#### Robot successfully delivered the virtual object

![alt text][dropImg]  
