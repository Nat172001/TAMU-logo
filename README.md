This repository contains a Python program for navigating a robot around the screen to draw the A&M logo using ROS2 Humble. The program utilizes a controller implemented in Python 3.8, allowing the robot to vary its linear and angular velocity to move towards specified goal coordinates.

**Overview**
The program extracts the coordinates of the A&M logo using an online image ruler and stores them in a list. During runtime, it executes the movetogoal function every 0.1 seconds. This function calculates both the distance and angle between the robot’s current position and the goal coordinate it is currently moving toward. Linear and angular velocities are adjusted proportionally to the differences in magnitude of these distances and angles.

<img src="https://raw.githubusercontent.com/Nat172001/TAMU-logo/main/TAMU_logo.png" width="400" height="400">

**Hyperparameters**
The program includes four tunable hyperparameters: <br/> 
**ε_D**: Tolerance for distance accuracy <br/>
**ε_A**: Tolerance for angle accuracy <br/>
**κ_v**: Linear velocity tuning parameter <br/>
**κ_w**: Angular velocity tuning parameter <br/>
These hyperparameters affect the accuracy of the drawn logo and the speed of the robot's movement.

**Class Structure**
The program defines a class named ToGoal, inheriting from the rclpy package’s Node class. In the __init__ function, it sets up a publisher of type Twist over the topic named /cmd_vel and creates a subscription to the robot’s pose. The functions described above and the hyperparameters are defined as methods and attributes in the class.

**Conclusion**
This program provides a framework for controlling a robot to draw the A&M logo accurately on the screen using ROS2 Humble. Users can tune hyperparameters to optimize performance based on their requirements.
