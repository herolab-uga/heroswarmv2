# heroswarmv2 - Fully-Capable Swarm robot design
This repository contains the open-sourced codes and hardware designs of the Swarm Robot (heroswarmv2) designed at the Heterogeneous Robotics Research Lab (HeRoLab) at the University of Georgia.

<p align="center">
<img src="https://github.com/herolab-uga/heroswarmv2/blob/main/images/rendered.png" width="400">
<img src="https://github.com/herolab-uga/heroswarmv2/blob/main/images/heroswarmv2-white.png" width="330">
<img src="https://github.com/herolab-uga/heroswarmv2/blob/main/images/heroswarmv2-gray.png" width="250">
</p>


## Capability
The robot is fully-capable in the following sense.

    ### Sensing: Multiple sensors such as proximity, RGB, sound, Inertial (IMU) altitude, and humidity and pressure measurements to capture local information; 
    ### Communication: Explicit data communication modalities such as Wi-Fi and Bluetooth; 
    ### Computing: High C1-level \cite{trenkwalder2019computational} computing through a Raspberry Pi Zero-based computing module that can support ROS and advanced programming; 
    ### Motion: Multi-level motor control with onboard wheel odometry aided by a microcontroller; 
    ### Power: Dedicated power management with different recharging variants such as inductive wireless charging or magnetic coupling. 
    

# Citation
A preprint of the paper focusing on the robot architecture is available at https://arxiv.org/abs/2211.03014

Cite the paper as "Starks M, Gupta A, Oruganti S, Parasuraman R. HeRoSwarm: Fully-Capable Miniature Swarm Robot Hardware Design With Open-Source ROS Support. In Proceedings of 2023 IEEE/SICE International Symposium on System Integration (SII) 2023. IEEE." URL: https://ieeexplore.ieee.org/document/10039174 

# Implementation Examples

Running a Conventional Multi-Robot Rendezvous Algorithm with HeRoSwarmV2 robots. Here, the robots are meeting into one place in a distributed manner.

<p align="center">
<img src="https://github.com/herolab-uga/heroswarmv2/blob/main/images/rendezvous_gif.gif" width="600">
</p>



Running a Conventional Multi-Robot Formation Control Algorithm with HeRoSwarmV2 robots. Here, the robots are organizing into a square-like pattern with 9 robots.

<p align="center">
<img src="https://github.com/herolab-uga/heroswarmv2/blob/main/images/formation_gif.gif" width="600">
</p>



More Demonstrations:

[![HeRoSwarm - Rendezvous Demo](https://img.youtube.com/vi/odDSfJcQ4XM/0.jpg)](https://www.youtube.com/watch?v=odDSfJcQ4XM)

[![HeRoSwarm - Formation Demo](https://img.youtube.com/vi/DiLuNy2J9R0/0.jpg)](https://www.youtube.com/watch?v=DiLuNy2J9R0)

[![HeRoSwarm - Distribution/Coverage Demo](https://img.youtube.com/vi/bHFGVdpAJ7c/0.jpg)](https://www.youtube.com/watch?v=bHFGVdpAJ7c)

[![HeRoSwarm - Pursuit-Evasion Demo](https://img.youtube.com/vi/HexOJGLDnEQ/0.jpg)](https://www.youtube.com/watch?v=HexOJGLDnEQ)


# Instructions

Go to the [Wiki page](https://github.com/herolab-uga/heroswarmv2/wiki) for mode details.



## Core contributors

* **Michael Starks** - undergraduate student researcher at HeRoLab (https://github.com/MichaelStarks)

* **Aryan Gupta** -- undergraduate student at Georgia Inst. of Technology (previously a high school intern at HeRoLab) (https://github.com/rtlaryan)

* **Sanjay Sarma O V** -- College of Engineering, University of Georgia (https://github.com/sanjayovs)

* PI: Prof. Ramviyas Parasuraman - HeRoLab UGA - (http://hero.uga.edu)


## Heterogeneous Robotics (HeRoLab)

This project is a part of the HeRoSwarm project at the Heterogeneous Robotics Research Lab (HeRoLab) of the University of Georgia.

Please contact hero at uga . edu for any queries

http://hero.uga.edu/

<p align="center">
<img src="https://hero.uga.edu/wp-content/uploads/2021/04/herolab_newlogo_whitebg.png" width="300">
</p>

