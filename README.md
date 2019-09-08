# Go Chase It Project
Project 2 of Udacity Robotics Software Engineer Nanodegree Program

## Overview 
In this project of the [Udacity Robotics Software Engineer] a differential drive robot is capable of chasing a white colored ball in a simulation world in Gazebo.

## Prerequisites/Dependencies
* Linux 16.04
* Gazebo >= 7.0  
* ROS Kinetic  
* make >= 4.1
* gcc/g++ >= 5.4

## Project Description  
Directory Structure  
```
.Project2                          # Go Chase It Project
├── my_robot                       # my_robot package                   
│   ├── launch                     # launch folder for launch files   
│   │   ├── robot_description.launch
│   │   ├── world.launch
│   ├── meshes                     # meshes folder for sensors
│   │   ├── hokuyo.dae
│   ├── urdf                       # urdf folder for xarco files
│   │   ├── my_robot.gazebo
│   │   ├── my_robot.xacro
│   ├── world                      # world folder for world files
│   │   ├── 1floorHouse.world
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info
├── ball_chaser                    # ball_chaser package                   
│   ├── launch                     # launch folder for launch files   
│   │   ├── ball_chaser.launch
│   ├── src                        # source folder for C++ scripts
│   │   ├── drive_bot.cpp
│   │   ├── process_images.cpp
│   ├── srv                        # service folder for ROS services
│   │   ├── DriveToTarget.srv
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info                  
└──           
```

## Run the project
* Create a catkin workspace
```bash
mkdir -p /home/workspace/catkin_ws/src
cd /home/workspace/catkin_ws/src
catkin_init_workspace
```
* Build the catkin package
```bash
cd /home/workspace/catkin_ws
catkin_make
```
* Launch the robot inside the world
```bash
cd /home/workspace/catkin_ws
source devel/setup.bash
roslaunch my_robot world.launch
```
* Launch drive_bot and process_image nodes on a new terminal
```bash
cd /home/workspace/catkin_ws
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```
* Move the white ball in front of the robot in the gazebo world and see the see the robot chasing the ball.