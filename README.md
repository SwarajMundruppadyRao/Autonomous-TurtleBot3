# Autonomous Mobile Robot 
This repository contains the ENPM809Y Final Project submission, showcasing autonomous navigation using SLAM and action-client nodes in ROS2. Waypoints are generated based on the detected positions of batteries placed within the environment, as captured by cameras. The robot's current position and waypoint locations are determined by referencing the TF tree for precise navigation.

![AutoTbot3](https://github.com/user-attachments/assets/97ce8b00-cfc0-480e-aa2c-45290c846825)


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
| Dependency       | Recommended Version  | Installation Command                                                                 |
|------------------|----------------------|--------------------------------------------------------------------------------------|
| ROS2             | Humble               | [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)   |
| Gazebo           | Latest Compatible    | Installed with ROS2 or `sudo apt install ros-humble-gazebo-ros-pkgs`                 |
| OpenCV           | Latest Compatible    | `sudo apt-get install libopencv-dev python3-opencv`                                  |
| OpenCV Contrib   | Latest Compatible    | Included with OpenCV installation                                                    |
| numpy            | < 2                  | `pip install "numpy<2"`                                                              |


## Steps to setup and run

1. Create a new workspace

    ```bash
    cd  
    mkdir -p ~/ros_ws/src
    cd ros_ws/src
    git clone https://github.com/SwarajMundruppadyRao/Autonomous-TurtleBot3.git
    cd ..
    ```

2. Make sure the folder structure looks like this 
    ``` bash 
    ros2_ws  
    ├── src
    │   ├── Autonomous-TurtleBot3
    │   ├── final_project
    │   ├── group5_final
    │   ├── mage_msgs
    │   ├── ros2_aruco
    │   ├── turtlebot3_navigation2
    │   ├── README.md
    ```

3. Install the package dependencies in the ROS workspace and colcon build 

    ```bash
    rosdep install --from-paths src -y --ignore-src
    colcon build --allow-overriding mage_msgs ros2_aruco_interfaces turtlebot3_navigation2
    ```

4. Set the TURTLEBOT3_MODEL env variable to waffle by adding this line to .bashrc and source bash

    ```bash
    echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
    source ~/.bashrc
    ```

5. Launch the TurtleBot3 Waflle model in Gazebo and RViz. Set the initial pose by selecting the ```2D Pose Estimate``` in RViz 

    ```bash
    ros2 launch final_project final_project.launch.py
    ```

6. Launch the ```tbot_nodes.launch.py``` to start the camera broadcaster nodes

    ```bash
    ros2 launch group5_final tbot_nodes.launch.py
    ```


7. Launch the ```tbot_pub.launch.py``` file to extract the waypoints

    ```bash
    ros2 launch group5_final tbot_pub.launch.py
    ```

8. To call the action to execute the waypoint navigation using following command 

    ```bash
    ros2 run group5_final tbot_follow_waypoints
    ```


The robot will fully navigate through the waypoints in the order mentioned in the parameter file by reading the aruco markers. 


## Citations

Group5, Autonomous TurtleBot3 [Source code]. GitHub. Available at: [https://github.com/SwarajMundruppadyRao/Autonomous-TurtleBot3](https://github.com/SwarajMundruppadyRao/Autonomous-TurtleBot3).

Special thanks to Professor Zeid Kootbally for his guidance, support, and for providing essential files and resources that contributed to the development of this project.


## Code Run Video 
https://github.com/user-attachments/assets/2e5c99c2-321c-4ee0-bf3e-03f4c865d637



