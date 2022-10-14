> # Robot Locolization
> Author: Alexis Wu <br>
> Oct. 2022
- All source code can be found in https://github.com/AlexisWu-01/compRobo22_robot_localization

---


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
Here is a recording of how the particle filter performs:

<a href="url"><img src="https://github.com/AlexisWu-01/compRobo22_robot_localization/blob/main/demo_resources/pf_demo.gif" width="2000" ></a>


The red arrows are our particles and the grey neato was our guessed position. If the red laser scan lines up with the steady map in the background, we know our estimated robot position and orientation is correct.

## 1. Particle Cloud Initialization and Weight Normalization
**Particle Cloud Initialization**
Each particle class contains 4 properties: `x`, `y`, `theta` (as orientation), and `w` (weight).
There are two ways to initialize the particle cloud:
1. Initialize all particles around the actual robot position with some variations. This is more computationally efficient and is easier to start with.
2. Initialize the particle randomly on the map. This is called the robot kidnapping problem. We need to have more particles and potentially improved algorithm to make it fast enough.

The above methods only differs in the positions and orientations. They both assign equal weight to each of the particles as we do not know which is better. We sticked with the first option but increased the variance as we developed the algorithm so we could still have a more spreadout cloud.

**Weight Normalization**
We need to always keep the sum of all weights to 1, because the probability is at most 1. Therefore, we need to sum the weight for each of the particle and then divide each of them by the sum.

## 2. Update Particle Position Based on Motor Info
The motor info we have from the neato was odometry was updated based on the wheel encoders which tells us the distance travelled and angle rotated. 

Because each of the neato acts at its own confidence, it needs to perform the same movement on its own coordinate frame as the robot moves in odometry frame. 

We perform matrix manipulation in this case:
1. Represent old robot position in odometry fram, represent new robot position in odometry frame. (Both using translational and rotational matrices)
2. Then we use the inverse of the first position in odometry and multiply it with the the second position in odometry to represent the new position to in the frame of old position.
3. Finally we multiply the transformation matrix with another matrix to transfer the particle frame to map frame.

During the process, we add noise to position and orientation of the updated particles to prevent [particle deprivation](https://homes.cs.washington.edu/~todorov/courses/cseP590/16_ParticleFilter.pdf) problem where there are no particles in the vicinity of the correct state and then we will never be able to find the correct position.

## 3. Update Particle Weight Based on Sensor Info
Here we calculate how well the sensor data -- laser scan of particles aligns with the actual laser scan from the robot. 
We would have a list of data of angles `theta` from robot frame and the corresponding distances `r`. To calculate each (x,y) position on map frame, we need to transfer the actual angle based on map frame by adding the angle of the scan to particle orientation. Now with the positions of obstacles "should be" as if the particle is at the robot position, we use [occupancy field](https://en.wikipedia.org/wiki/Occupancy_grid_mapping) which returns the distance between the given point and the closest obstacle to it. In this case we were hoping for a 0 value from occupancy field which means the scan data at particle position aligns well with actual robot scan.

### Likelihood Calculation - Gaussian Distribution
 For assigning weight to particles based on how well their scan aligns with the actual robot scan. From the `occupancy_field`, we could read the error distance `d` for each scan. The likelihood of each scan is represented by $$p(x) = \frac{1}{\sigma \sqrt{2 \pi}} e^{-\frac{1}{2}(\frac{0-d}{\sigma})^2}$$
![Gaussian Distribution]](https://github.com/AlexisWu-01/compRobo22_robot_localization/blob/main/demo_resources/gaussian_distribution.png)
 In the diagram above we could clearly see that the highest possibility is where the input value x equals the mean value. In our case, we want to set the ideal distance from occupancy_field to 0. which is why we have `(0-d)` in the equation. The $\sigma$ stands for standard deviation, we could arbitrarily design its value to determin how "spreadout" our distribution is. Therefore, we would to give the particle a higher weight when the error distance is closer to 0. 
 To make this bonus more distinctive between the particles, we first make it cube then add the result to weight for a more acute distribution curve. 
