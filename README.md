> # Robot Locolization
> Author: Alexis Wu <br>
> Oct. 2022
- All source code can be found in https://github.com/AlexisWu-01/compRobo22_robot_localization

---

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
![Gaussian Distribution](https://github.com/AlexisWu-01/compRobo22_robot_localization/blob/main/demo_resources/gaussian_distribution.png)

 In the diagram above we could clearly see that the highest possibility is where the input value x equals the mean value. In our case, we want to set the ideal distance from occupancy_field to 0. which is why we have `(0-d)` in the equation. The $\sigma$ stands for standard deviation, we could arbitrarily design its value to determin how "spreadout" our distribution is. Therefore, we would to give the particle a higher weight when the error distance is closer to 0. 
 To make this bonus more distinctive between the particles, we first make it cube then add the result to weight for a more acute distribution curve. 


## 4. Resample Particles
After each update of particle cloud is performed, we need to resample the particles based on their weight to randomly abandon the particles with lower weights. We used a technique called low variance resampling where we make sure the particles with higher possibilities are guaranteened to be selected rather than a completely random process. 
We also reduce the number of particles if any particle has a confidence level greater than 40% to speed up our computation once we have a very likely estimation of the robot pose.

## 5. Update robot pose
`self.robot_pose` represents the best estimation of robot position. We chose 1 single particle with the highest possibility/ weight rather than calculating the mean of all particles to prevent the effect of outliers.

# Challenges 
## 1. Coordinates Transformation
There were multiple frames to work with and generally we needed to transform everything to work on the map frame eventually. This was extremely hard when it came to `update_particles_with_odom`. The most intuitive way for implementation was to break down the robot movement ( rotate for an angle until it face towards the new x,y in map frame, then goes straight for $\sqrt{x^2+y^2}$ then rotate the particle to its current orientation) in its own frame and perform the same steps in each of the particle frame. 

However, this simpler implementation was not computationally efficient. And therefore we introduced matrix manipution. The map was hard to understand so I had to draw a lot of diagrams and ask for explanation.

There should be more and harder frame transformations in the process but luckily the scaffolder code handled it for us.

## 2. Debugging
With all new ros2 platform and unstable rviz2, debugging was extremely hard: the terminals would still run, but with warnings I could not notice or interprate or got covered by the print statments for debugging purpose (which did not work that well). Also due to the long break, I could only complete every function without debugging and testing each of them separately. The compleete system should take way longer than doing it along the way.

However, I am optimistic about future debugging as I not only learned many techniques that are extremely useful during the office hours but also gaining more familarity on the tools as we use it.

# Key Takeaways 
## 1. Likelihood Field

The concepts and the maths behind occupancy field and likelihood field did not make sense to me. Therefore I went through the lectures on youtube and read through related documents to gain a complete understanding of this. I believe this would be beneficial for future probablity related modellings.

## 2. Resampling
I did not find out the resampling method already exists in the helper function and went through the same process as above to deeply understand the underlying methods and be able to implement this algorithm from sketch.

# Lessons Learned and Future Improvements

#### Plan Ahead in Project Development
I did not debug at every step of the way as I should have. As a result, I had to spend tremendous amount of time on debugging and visualizing after I have the whole program completed. Balancing among algorithm, code, debugging and test would be my priority in future project development.

#### Ros Implementation vs. Algorithm Concepts
The scaffold code saved a tremendous time on trying to find which could be a simple answer as we did in warmup project. However, with the current code I sort of naturally chose the MVP implementation wise as all I need was to follow the TODOs. I was able to explore the algorithm and mathematical aspects of a particle filter. But I would really like to work more on ros implementation just to learn more about its features and how it works.

#### Computation Speed
As we can see from the gif from implementation section, the particle filter works but was not able to follow the neato movement realtime. A great optimization would be using np matrix instead of the for loops in implementation or potentially use multithread programming. 

#### Generalized Use
Due to debugging and computation speed limits, I only tested the particle filter on gazebo map with particles initialized related to robot_pose. Once the program speed improves, I would like to test the particle filter on larger maps with unknown start position (robot kidnapping problem) and finally tune the variables like noises. 
