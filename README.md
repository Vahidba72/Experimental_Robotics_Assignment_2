# ROSPlan based Robotic Planning System and Localization 


<p><strong>Description:</strong> This project integrates ROSPlan-based planning for a mobile robot  mobile robot endowed with a camera and a laser scanner in a Gazebo environment with 4 different waypoints. In each waypoint there exists one aruco marcer. The robot's goal is to navigate through the 4 different waypoints, detect the markers id in each waypoint, and then move to the next waypoint and repeat the task until all markers are found based on PDDL (Planning Domain Definition Language) actions. Finally the robot should go to the location which holds the marker with least id. The system uses a custom action interface to control the robot's behavior based on planning results. During its task, the robot needs to avoid obstacles such as walls and also the aruco markers when trying to go to a new location. Consequently, our system will utilize gmapping as the Simultaneous Localization and Mapping (SLAM) algorithm to map and determine the robot's position within the environment. Furthermore, the Move_base package will be employed for enabling autonomous navigation capabilities. </p>

---
## Table of Contents

1. [Overview](#overview)  
2. [Dependencies](#dependencies)
3. [installation](#Installation)
4. [usage](#Usage)  
5. [actions](#actions)  
6. [Code Overview](#code-overview)  
7. [Author](#author)  

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

## Dependencies
Make sure the following packages are installed:

```bash
cd ~/<ros_workspace>/src
git clone https://github.com/CarmineD8/aruco_ros.git
```
After Installing the aruco ros package make sure to substitute the marker_publish.cpp file in the src folder of the aruco_ros package with the marker_publish.cpp file in this repository. This is because the default code is changed partially for the goals of this project.

```bash
git clone https://github.com/CarmineD8/SLAM_packages.git # Remember to switch to noetic branch
git clone https://github.com/KCL-Planning/ROSPlan.git
```
<h2 id="installation">Installation</h2>
<p>To get started with this project, follow the steps below to set up your development environment:</p>

<ol>
    <li><strong>Clone the Repository:</strong>
        <pre><code>git clone https://github.com/your_username/rosplan-robotic-planner.git</code></pre>
    </li>
    <li><strong>Install Dependencies:</strong> Make sure you have ROS installed. This project was developed using ROS Noetic. Follow the official ROS installation guide: <a href="http://wiki.ros.org/noetic/Installation">ROS Noetic Installation Guide</a>.
    </li>
    <li><strong>Install the ROSPlan package:</strong> This project requires ROSPlan for planning. You can install it by running the following commands:
        <pre><code>sudo apt-get install ros-noetic-rosplan</code></pre>
    </li>
    <li><strong>Install additional dependencies:</strong> Navigate to the project folder and run:
        <pre><code>rosdep install --from-paths src --ignore-src -r -y</code></pre>
    </li>
    <li><strong>Build the Workspace:</strong>
        <pre><code>cd ~/catkin_ws
catkin_make
source devel/setup.bash</code></pre>
    </li>
</ol>

<hr>

<h2 id="usage">Usage</h2>
<p>Once the installation is complete, follow these steps to launch the system:</p>

<ol>
    <li><strong>Start ROS Core:</strong>
        <pre><code>roscore</code></pre>
    </li>
    <li><strong>Launch Simulation:</strong> In a new terminal, run the following to launch the simulation environment:
        <pre><code>roslaunch assignment2_exprob main.launch</code></pre>
    </li>
    <li><strong>Monitor the Robot's Actions:</strong> To track robot movements and actions, use the following command:
        <pre><code>rosrun rqt_graph rqt_graph</code></pre>
    </li>
    <li><strong>Run the Planning System:</strong> In another terminal, you can use the planning interface:
        <pre><code>roslaunch assignment2_exprob plan_sim.launch</code></pre>
    </li>
</ol>

<hr>

<h2 id="actions">Actions</h2>
<p>There are three main actions that the robot can perform based on the dispatched plan:</p>

<ul>
    <li><strong>go_to:</strong> Moves the robot to a specified waypoint (e.g., `wp1`, `wp2`, etc.).</li>
    <li><strong>detect:</strong> The robot rotates to find a marker. Once found, it updates the lowest marker ID and the associated location.</li>
    <li><strong>check_markers:</strong> The robot returns to the last location where the lowest marker ID was detected.</li>
</ul>

<hr>

<h2 id="contribution">Contribution</h2>
<p>Contributions are welcome! To contribute to this project:</p>
<ol>
    <li>Fork this repository.</li>
    <li>Clone your forked repository.</li>
    <li>Create a new branch for your feature or fix.</li>
    <li>Commit your changes with a clear description.</li>
    <li>Push your changes and create a pull request.</li>
</ol>

<h3>Code Style</h3>
Please ensure that your code follows the standard ROS C++ coding style. You can refer to the ROS C++ style guide here: <a href="https://github.com/ros/ros_cpp_style_guide">ROS C++ Style Guide</a>.

<hr>

<h2 id="license">License</h2>
<p>This project is licensed under the MIT License - see the <a href="LICENSE">LICENSE</a> file for details.</p>

</body>
</html>

