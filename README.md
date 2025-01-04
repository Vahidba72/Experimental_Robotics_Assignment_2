# ROSPlan based Robotic Planning System and Localization 


<p><strong>Description:</strong> This project integrates ROSPlan-based planning for a mobile robot  mobile robot endowed with a camera and a laser scanner in a Gazebo environment with 4 different waypoints. In each waypoint there exists one aruco marcer. The robot's goal is to navigate through the 4 different waypoints, detect the markers id in each waypoint, and then move to the next waypoint and repeat the task until all markers are found based on PDDL (Planning Domain Definition Language) actions. Finally the robot should go to the location which holds the marker with least id. The system uses a custom action interface to control the robot's behavior based on planning results. During its task, the robot needs to avoid obstacles such as walls and also the aruco markers when trying to go to a new location. Consequently, our system will utilize gmapping as the Simultaneous Localization and Mapping (SLAM) algorithm to map and determine the robot's position within the environment. Furthermore, the Move_base package will be employed for enabling autonomous navigation capabilities. </p>

Here you can see a demo of the implemented code:

---
## Table of Contents

1. [Overview](#overview)
2. [Usage](#usage)
3. [actions](#actions)
4. [Package Descriptions](#package-descriptions)
5. [Author](#author)  

---

## Overview
This project involves using ROSPlan for planning, which integrates domain and problem definitions in PDDL to perform automated actions for a robot. The robot can navigate to specific waypoints, detect markers, and return to the location where the marker with the least id was found.

### Key Features

**Waypoint Navigation:** Robot moves to predefined waypoints using MoveBase. We were provided with certain clues regarding the marker's position:
- WP0 is located at: x = -7.0, y = 1.5
- WP 1 is located at: x = -3.0, y = -8.0
- WP 2 is located at: x = 6.0, y = 2.0
- WP 3 is located at: x = 7.0, y = -5.0
- WP4 which is the robot's initial position is: x = 0, y = 2.75
  
**Marker Detection:** The robot searches for markers, updates the location with the lowest marker ID, and navigates accordingly.

**ROSPlan Integration:** Uses ROSPlan to create plans based on PDDL and dispatch actions for robot execution.

---

## Usage

1. At first clone this repository into your ROS workspace:

```bash
git clone https://github.com/Vahidba72/Experimental_Robotics_Assignment_2
```
2. Make sure the following packages are installed:

```bash
cd ~/<ros_workspace>/src
git clone https://github.com/CarmineD8/aruco_ros.git
```
3. After Installing the aruco ros package make sure to substitute the marker_publish.cpp file in the src folder of the aruco_ros package with the marker_publish.cpp file in this repository. This is because the default code is changed partially for the goals of this project.

4. SLAM_packages and ROSPlan should also be cloned:

```bash
git clone https://github.com/CarmineD8/SLAM_packages.git # Remember to switch to noetic branch
git clone https://github.com/KCL-Planning/ROSPlan.git
```

For ROSPlan follow the instruction in their readme to properly install the dependencies.

5. After cloning all the required packages and installing all the dependencies, run the `main.launch` file to spawn the robot in the gazebo and rviz environments and also run the gmapping and move_base algorithms for the localization purposes.

```bash
roslaunch assignment2_exprob main.launch
```
6. in a seperate terminal run the `plan_sim.launch` file to start the planning process and dispatch the plan:


```bash
roslaunch assignment2_exprob plan_sim.launch
```

7. In a seperate terminal, run the following lines of code to generate the plan, parse it, and then dispatch it:


```bash
rosservice call /rosplan_problem_interface/problem_generation_server

rosservice call /rosplan_planner_interface/planning_server

rosservice call /rosplan_parsing_interface/parse_plan

rosservice call /rosplan_plan_dispatcher/dispatch_plan
```
Keep in mind that you can see the generated plan by observing the following topic:

```bash
rostopic echo /rosplan_planner_interface/planner_output -p
```

---

## Actions

In the `MyDomain.pddl`, three main actions are defined:

- **go_to:** Moves the robot from a specified waypoint to a target waypoint (e.g., `wp1`, `wp2`, etc.).
- **detect:** The robot rotates to find the the aruco marker located at the specific waypoint.
- **check_markers:** This specific action only checks if all the waypoints are visited and all the markers are detected. 

---

## Package Descriptions

There are two main packages in this repository: `assignment2_exprob` and `my_rosplan_interface`. The `aruco_ros` package here only contains one cpp file which will is used as we explained before in the Usage section.

- **assignment2_exprob:** In this package the pddl planning is implemented. The domain and problem files are provided in the folder `PDDL/`, while all the necessary nodes are launched with the `main.launch` and `plan_sim.launch` launch files. The configurations of gmapping and move_base and all their parameters are also in this package. In particular, the yaml configuration can be found in the `param/` folder. The robot xacro and gazebo file `my_robot4.xacro` and `my_robot4.gazebo` are also in this package in the `urdf` folder.

- **my_rosplan_interface:** This package implements the action dispatch mechanism and associates each action with a specific robot's action. There is node called my_action which does this whole process. We handle 3 actions:
  - goto: when this action is dispatched, the my_action node sends the corresponding waypoint coordinates with an action client using move_base and the robot starts moving towards the target waypoint.
  - detect: when this action is dispatched, the my_action node checks a parameter `flag` which is true until the marker is not found. When the marker is found, this parameter will beccome false. The robot keeps rotating until it detects the marker and the marker's center is at the center of the image. When the marker is detected, its id is compared with a global varible `LeastID` and if the id is smaller than `LeastID`, it will be substituted and its location's label (e.g., `wp0`) is also stored and then the corresponding paramters `last_loc` and `least_id` in the `main.launch` file are set.
  - check_markers: when this action is dispatched, the robot gets the `last_loc` and `least_id` from the launch file and sends the goal to the action client and robot moves toward the locaion with the least id value. 
