# Comparative Analysis of Visual & Lidar Based SLAM Algorithms in ROS
This is for 16833-Robot Localization and Mapping course (Spring21) project code. It includes the launch file, command line, and other repo we used for the project


# RTAB-Map
For the RTAB-map, it will require installation of RTAB-Map from the website. The installation options include sudo installed and build from source. Either method will work for for the launch file. Link: https://github.com/introlab/rtabmap_ros 

For specific parameters and their purposes, check the link: http://wiki.ros.org/rtabmap_ros
For the experiment test with L515 camera, check the command_line.txt file in the RTAB-Map folder

# ORB-SLAM2
**ORB-SLAM2 Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2)).
The original implementation can be found [here](https://github.com/raulmur/ORB_SLAM2.git).

# ORB-SLAM2 ROS node
This is the ROS implementation of the ORB-SLAM2 real-time SLAM library for **Monocular**, **Stereo** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time. This implementation removes the Pangolin dependency, and the original viewer. All data I/O is handled via ROS topics. For visualization you can use RViz.

Repo to build ORB-SLAM2 in ROS: https://github.com/appliedAI-Initiative/orb_slam_2_ros

Cartographer ROS Integration
============================

`Cartographer`_ is a system that provides real-time simultaneous localization
and mapping (`SLAM`_) in 2D and 3D across multiple platforms and sensor
configurations. This project provides Cartographer's ROS integration.

Build Cartographer: https://google-cartographer-ros.readthedocs.io/en/latest/

.. _Cartographer: https://github.com/cartographer-project/cartographer

.. _SLAM: https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping


# Evaluation 
Link to repo : http://github.com/MichaelGrupp/evo

***Python package for the evaluation of odometry and SLAM***

| Linux / macOS / Windows / ROS |
| :---: |
| [![Build Status](https://dev.azure.com/michl2222/michl2222/_apis/build/status/MichaelGrupp.evo?branchName=master)](https://dev.azure.com/michl2222/michl2222/_build/latest?definitionId=1&branchName=master) |

This package provides executables and a small library for handling, evaluating and comparing the trajectory output of odometry and SLAM algorithms.

Supported trajectory formats:

* 'TUM' trajectory files
* 'KITTI' pose files
* 'EuRoC MAV' (.csv groundtruth and TUM trajectory file)
* ROS bagfile with `geometry_msgs/PoseStamped`, `geometry_msgs/TransformStamped`, `geometry_msgs/PoseWithCovarianceStamped` or `nav_msgs/Odometry` topics or [TF messages](https://github.com/MichaelGrupp/evo/wiki/Formats#bag---ros-bagfile)

For our project we evaluated the Absolute Pose Error.

### Run a metric on trajectories

  For example, here we calculate the absolute pose error for the estimated trajectories from  RGBD RTAB-MAP using `evo_ape` (`CARLA_00_gt.txt` is the reference (ground truth)) and plot the graph using the TUM format:

  *First trajectory (RGBD RTAB-MAP):*

  ```
  evo_ape tum CARLA_00_gt.txt CARLA_00_RgbdRTAB.txt -va --plot --plot_mode xy
  ```
  <img src="https://user-images.githubusercontent.com/21180916/118181877-4a0b7380-b406-11eb-8328-5dc7f0fc338e.png" width="450">  <img src="https://user-images.githubusercontent.com/21180916/118181905-52fc4500-b406-11eb-9bd3-7e5c44aeb315.png" width="450">
  
  
  # ROS/ROS2 bridge for CARLA simulator

[![Actions Status](https://github.com/carla-simulator/ros-bridge/workflows/CI/badge.svg)](https://github.com/carla-simulator/ros-bridge)
[![Build Status](https://travis-ci.com/carla-simulator/ros-bridge.svg?branch=master)](https://travis-ci.com/carla-simulator/ros-bridge)
[![GitHub](https://img.shields.io/github/license/carla-simulator/ros-bridge)](https://github.com/carla-simulator/ros-bridge/blob/master/LICENSE)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/carla-simulator/ros-bridge)](https://github.com/carla-simulator/ros-bridge/releases/latest)

 This ROS package is a bridge that enables two-way communication between ROS and CARLA. The information from the CARLA server is translated to ROS topics. In the same way, the messages sent between nodes in ROS get translated to commands to be applied in CARLA.



## Features

- Provide Sensor Data (Lidar, Semantic lidar, Cameras (depth, segmentation, rgb, dvs), GNSS, Radar, IMU)
- Provide Object Data (Transforms (via [tf](http://wiki.ros.org/tf)), Traffic light status, Visualization markers, Collision, Lane invasion)
- Control AD Agents (Steer/Throttle/Brake)
- Control CARLA (Play/pause simulation, Set simulation parameters)

## Getting started and documentation

Installation instructions and further documentation of the ROS bridge and additional packages are found [__here__](https://carla.readthedocs.io/projects/ros-bridge/en/latest/).

Clone repo and build: https://github.com/carla-simulator/ros-bridge


CARLA Simulator
===============

Instructions for Buildng in Linux: https://carla.readthedocs.io/en/latest/build_linux/

CARLA is an open-source simulator for autonomous driving research. CARLA has been developed from the ground up to support development, training, and
validation of autonomous driving systems. In addition to open-source code and protocols, CARLA provides open digital assets (urban layouts, buildings,
vehicles) that were created for this purpose and can be used freely. The simulation platform supports flexible specification of sensor suites and
environmental conditions.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![GitHub tag (latest SemVer)](https://img.shields.io/github/tag/carla-simulator/scenario_runner.svg)
[![Build Status](https://travis-ci.com/carla-simulator/scenario_runner.svg?branch=master)](https://travis-ci.com/carla/scenario_runner)

ScenarioRunner for CARLA
========================
This repository contains traffic scenario definition and an execution engine
for CARLA. It also allows the execution of a simulation of the CARLA Challenge.
You can use this system to prepare your agent for the CARLA Challenge.

Scenarios can be defined through a Python interface, and with the newest version
the scenario_runner also the upcoming [OpenSCENARIO](http://www.openscenario.org/) standard is supported.

Repo: https://github.com/carla-simulator/scenario_runner
