This repository contains a Python program for navigating a robot around the screen to draw the A&M logo using ROS2 Humble. The program utilizes a controller implemented in Python 3.8, allowing the robot to vary its linear and angular velocity to move towards specified goal coordinates.

**Overview**
The program extracts the coordinates of the A&M logo using an online image ruler and stores them in a list. During runtime, it executes the movetogoal function every 0.1 seconds. This function calculates both the distance and angle between the robot’s current position and the goal coordinate it is currently moving toward. Linear and angular velocities are adjusted proportionally to the differences in magnitude of these distances and angles.
