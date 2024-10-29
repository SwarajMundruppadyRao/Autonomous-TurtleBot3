# Autonomous Mobile Robot 
This repository contains the ENPM809Y Final Project submission, showcasing autonomous navigation using SLAM and action-client nodes in ROS2. Waypoints are generated based on the detected positions of batteries placed within the environment, as captured by cameras. The robot's current position and waypoint locations are determined by referencing the TF tree for precise navigation.

## Project Objectives  
The project involves the following tasks : 
- Creating a map of the environment
- Loading the Generated Map
- Reading ArUCo marker and retreiving the appropriate parameters
- Detect parts using logical cameras
- Writing an action Client to go through each part using the order described by the parameters


## Features : 
- Autonomous waypoint navigation
- Real-time position updates using TF tree
- Battery detection for waypoint generation
- Compatibility with ROS2 Humble and Gazebo simulation environment

## Dependencies :
| Dependency       | Recommended Version  |
|------------------|----------------------|
| ROS2             | Humble               |
| Gazebo           | Latest Compatible    |
| OpenCV           | Latest Compatible    |
| OpenCV Contrib   | Latest Compatible    |
| numpy            | < 2                  |


## Steps to replicate the results 

1. Create a directory 
2. 
   


