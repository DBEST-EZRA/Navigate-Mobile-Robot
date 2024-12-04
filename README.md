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

# PyBullet Simulation: Car Navigation with Lidar (main_test)

This project simulates a mobile robot navigating a plane with obstacles using PyBullet. The car is equipped with a Lidar system for obstacle detection and is programmed to navigate through a defined environment.

## Features

1. **Environment Setup**:
   - A plane is loaded as the base environment.
   - A racecar model (`racecar_differential.urdf`) is used for the mobile robot.
   - Gravity and simulation settings are configured.

2. **Obstacles**:
   - Eight obstacles (cubes) are placed at predefined positions on the plane.
   - Positions can be randomized using the `random_obstacles()` function.

3. **Lidar System**:
   - A 30-ray Lidar system is implemented to detect obstacles.
   - Lidar operates at 20Hz, updating ray information to determine obstacle positions.

4. **Control Mechanism**:
   - The car's movement is controlled via velocity commands for the wheels and positional commands for the steering.
   - Placeholder tasks are included for implementing navigation algorithms:
     - **Task 1**: Controlling the car's velocity and steering.
     - **Task 2**: Designing a pathfinding algorithm from `(0, 0)` to `(11, 11)`.

5. **Simulation**:
   - Uses PyBullet's real-time and stepped simulation modes.
   - Visualizes Lidar rays as debug lines, showing hits and misses with different colors.

## Prerequisites

- Python 3.x
- PyBullet library

Install PyBullet using pip:

```bash
pip install pybullet

