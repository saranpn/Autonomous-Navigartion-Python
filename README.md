# Autonomous-Driving

This course is based on the data (LiDAR and Encoder) collected from a robot with caterpillar tracks navigated through a controlled environment. 
  
## Overview

### Projects
<table style="width:100%">
  <tr>
    <th>
      <p align="center">
           <a href="https://https://github.com/saranpn/Autonomou-Navigartion-Python/tree/master/Motion%20and%20Sensor%20Model"><img src="./Motion%20and%20Sensor%20Model/robot_motion_model.gif" alt="Overview" width="100%" height="100%"></a>
           <br><a href="https://github.com/saranpn/Autonomou-Navigartion-Python/tree/master/Motion%20and%20Sensor%20Model" name="p1_code">Motion and Sensor Model </a>
        </p>
    </th>
    <th>
           <p align="center">
           <a href="https://github.com/saranpn/Autonomou-Navigartion-Python/tree/master/Feature%20based%20and%20Featureless%20localization"><img src="./Feature%20based%20and%20Featureless%20localization/featureless_icp.gif" alt="Overview" width="100%" height="100%"></a>
           <br><a href="https://github.com/saranpn/Autonomou-Navigartion-Python/tree/master/Feature%20based%20and%20Featureless%20localization" name="p1_code">Feature based and Featureless localization </a>
        </p>
    </th>
    <th>
      <p align="center">
           <a href="https://github.com/saranpn/Autonomou-Navigartion-Python/tree/master/Bayes%20Filter"><img src="./Bayes%20Filter/img_03_KF.gif/img_03_KF.gif" alt="Overview" width="100%" height="100%"></a>
           <br><a href="https://github.com/saranpn/Autonomou-Navigartion-Python/tree/master/Bayes%20Filter" name="p1_code">Bayes Filter </a>        
        </p>
    </th>
  </tr>
  <tr>
    <th>
      <p align="center">
           <a href="https://github.com/saranpn/Autonomous-Driving/tree/master/Unit_D"><img src="./Unit_D/kalman_prediction_and_correction.gif" alt="Overview" width="100%" height="100%"></a>
           <br><a href="https://github.com/saranpn/Autonomous-Driving/tree/master/Unit_D" name="p1_code">Unit_D: Extended Kalman Filter </a>
        </p>
    </th>
    <th>
           <a href="https://github.com/saranpn/Autonomous-Driving/tree/master/Unit_F"><img src="./Unit_F/ekf_slam.gif" alt="Overview" width="100%" height="100%"></a>
           <br><a href="https://github.com/saranpn/Autonomous-Driving/tree/master/Unit_F" name="p1_code">Unit_F: EKF SLAM </a>
        </p>
    </th>
  </tr>
</table>
--- 

## Table of Contents

#### [1. Model](Unit_A)
 - **Summary:** Developed a motion model of the robot, transformed the motor ticks of the robot into a real world trajectory and corrected the calibration error with the sensor model, developed using the LiDAR data by detecting the location of the landmarks in scanner's coordinate system.
 - **Keywords:** Motion model, motor control, LiDAR and encoder data.

#### [2. Localization](Unit_B)
 - **Summary** _Feature Based Localization:_ Assigned the detected landmarks to the respective landmarks in the map, developed the mathematics for similarity transformation and applied as a direct solution, and corrected the pose of the robot based on the transform. _Featureless localization:_ Assigned the scan points to the walls of the arena and applied Iterative Closest Point to find the optimal transformation.
 - **Keywords:** Similarity transformation, ICP
 
#### [3. Bayes Filter](Unit_C)
 - **Summary:** Developed a histogram filter and a 1-D Kalman filter which are recursive Bayes filters, to estimate the true position of the Robot, which is uncertain about its movement and has measurement noise.
 - **Keywords:** Histogram filter, 1-D Kalman filter.
 
#### [4. Kalman Filter](Unit_D)
 - **Summary:** Augmented the Kalman filter developed in Unit 3 to _n_ dimensions and implemented Exteded Kalman filter to localize our robot which has non linear states and control commands.
 - **Keywords:** Sensor fusion, Extended Kalman filter.
 
#### [5. EKF SLAM](Unit_E)
 - **Summary:** What if the robot is not given the map of the environment? This is the common problem in the field of autonomous navigation. We have to estimate both the position and heading of the robot and the position of the landmarks. These 2 processes cannot be separated from each other and must be done simultaneously. In this part of the project, I implement an Extended Kalman Filter Simultaeous Localization and Mapping algorithm, as a solution to this problem.
 - **Keywords:** Sensor fusion, EKF SLAM
