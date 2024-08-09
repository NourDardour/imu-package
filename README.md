# IMU Emulator Package

## Overview
This repository contains an IMU (Inertial Measurement Unit) emulator package designed for use in navigation systems. The IMU emulator simulates acceleration and gyroscopic data, providing the Absolute Navigation system with mocked inputs. It integrates with ROS (Robot Operating System) to receive trajectory data and publish formatted IMU data.

## Features
- **ROS Integration**: Subscribes to trajectory data from a designated ROS topic.
- **Data Emulation**: Generates simulated acceleration and gyroscopic data based on the latest trajectory information.
- **User-Configurable Frequency**: Control the rate of IMU data transmission (100Hz, 200Hz, or 500Hz).
- **Failure Simulation**: Prepares the system to simulate various IMU failures for robust testing.

## Requirements
- **ROS**: Ensure that you have ROS installed on your system (e.g., ROS Noetic).
- **C++ Compiler**: Necessary for building the C++ source code.

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/NourDardour/imu-package.git
   cd imu_package
   ```
