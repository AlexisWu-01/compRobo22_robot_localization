> # Robot Locolization
> Author: Alexis Wu <br>
> Created in Oct. 2022
- All source code can be found in https://github.com/AlexisWu-01/compRobo22_robot_localization

---


- What was the goal of your project?
- How did you solve the problem? (Note: this doesn’t have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).
- Describe a design decision you had to make when working on your project and what you ultimately did (and why)? These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.
What if any challenges did you face along the way?
- What would you do to improve your project if you had more time?
- Did you learn any interesting lessons for future robotic programming projects? These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.

## Introduction
The goal in this project is to implement a [particle filter](https://en.wikipedia.org/wiki/Particle_filter) on a map where we use a probablity based method to infer the position of the robot on a known map based on:
 - sensor measurement: laser scan
 - motor information: odometry

I started by understanding from a high level about how the particle filter works. The particle filter algorithm could be broken down into: 
- Randomly generate n particles on the map
- Calculate the weight of each particle based on how well its laser scan lines up with that of the actual robot
- Keeps the movement of the particles consistent with the odometry information from actual robot movement.
- If the movement (translational or rotational) is beyond our threshold, we recheck the sensor measurement and resample based on different weights: particles with larger weight are more likely to be selected.
- We recognize the most important particle as our estimated robot position.

Here is a diagram of a general idea of how our program works:
![Work map](https://github.com/AlexisWu-01/compRobo22_robot_localization/blob/main/demo_resources/work_map.png)

# Implementation 
Here is a recording of how our particle filter works:

<a href="url"><img src="[http://url.to/image.png](https://github.com/AlexisWu-01/compRobo22_robot_localization/blob/main/demo_resources/pf_demo.gif)" width="200" ></a>

The red arrows are our particles and the grey neato was our guessed position. If the red laser scan lines up with the steady map in the background, we know our estimated robot position and orientation is correct.

## 1. Particle Cloud Initialization
Each particle class contains 4 properties: `x`, `y`, `theta` (as orientation), and `w` (weight).
There are two ways to initialize the particle cloud:
1. Initialize all particles around the actual robot position with some variations. This is more computationally efficient and is easier to start with.
2. Initialize the particle randomly on the map. This is called the robot kidnapping problem. We need to have more particles and potentially improved algorithm to make it fast enough.

We sticked with 
