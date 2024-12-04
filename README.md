# PyBullet Racecar Simulation

This repository contains a PyBullet-based simulation of a racecar navigating a dynamically generated obstacle field. The simulation integrates LiDAR-based obstacle detection and provides a framework for controlling the car to reach a target destination.

## Features
- **Dynamic Environment Setup:**
  - Loads a racecar and plane model.
  - Generates obstacles at random positions.
- **LiDAR Integration:**
  - Simulates a LiDAR sensor with 100 rays.
  - Visualizes raycast hits and misses.
- **Racecar Control:**
  - Controls wheel velocity and steering.
  - Synchronizes wheel movements using PyBullet constraints.
- **Real-Time Simulation Loop:**
  - Updates LiDAR and control logic at specified frequencies.
  - Allows for path planning and navigation tasks.

## Tasks
1. **Car Control:**
   - Implement a function to control the car using wheel velocity and steering angle.
2. **Path Planning:**
   - Navigate the car from its starting position `(0, 0)` to the target destination `(11, 11)` while avoiding obstacles.

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/DBEST-EZRA/Navigate-Mobile-Robot.git
   cd pybullet-racecar-simulation
