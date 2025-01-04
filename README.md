<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Project README</title>
</head>
<body>

<h1>Project Name: ROSPlan-based Robotic Planning System</h1>

<p><strong>Author:</strong> Vahid Bagherian</p>
<p><strong>Course:</strong> Master's in Robotic Engineering</p>
<p><strong>Description:</strong> This project integrates ROSPlan-based planning for a robot to navigate waypoints, detect markers, and return to previous locations based on PDDL (Planning Domain Definition Language) actions. The system uses a custom action interface to control the robot's behavior based on planning results.</p>

<hr>

<h2>Table of Contents</h2>
<ul>
    <li><a href="#overview">Overview</a></li>
    <li><a href="#installation">Installation</a></li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#actions">Actions</a></li>
    <li><a href="#contribution">Contribution</a></li>
    <li><a href="#license">License</a></li>
</ul>

<hr>

<h2 id="overview">Overview</h2>
<p>This project involves using ROSPlan for planning, which integrates domain and problem definitions in PDDL to perform automated actions for a robot. The robot can navigate to specific waypoints, detect markers, and return to the last location where a marker was found.</p>

<h3>Key Features</h3>
<ul>
    <li><strong>Waypoint Navigation:</strong> Robot moves to predefined waypoints using ROSMoveBase.</li>
    <li><strong>Marker Detection:</strong> The robot searches for markers, updates the location with the lowest marker ID, and navigates accordingly.</li>
    <li><strong>ROSPlan Integration:</strong> Uses ROSPlan to create plans based on PDDL and dispatch actions for robot execution.</li>
</ul>

<hr>

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
