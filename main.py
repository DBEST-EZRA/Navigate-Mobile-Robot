import pybullet as p
import time
import math
import numpy as np
import random


############################################### Environment Setup ####################################################
p.connect(p.GUI)

p.resetSimulation()

p.setGravity(0, 0, -10)
useRealTimeSim = 0

p.setRealTimeSimulation(useRealTimeSim)

# Load plane and car
track = p.loadURDF("data/plane/plane.urdf")
car = p.loadURDF("f10_racecar/racecar_differential.urdf", [0, 0, 0])

# Function to generate random obstacle positions
def random_obstacles():
    np.random.seed()
    xy_position_float = np.random.rand(2)
    x_position_range = np.random.randint(1, 10)
    y_position_range = np.random.randint(1, 10)

    xy_position = [xy_position_float[0] + x_position_range, xy_position_float[1] + y_position_range]
    position = np.append(xy_position, 0.2)
    return position


# Load obstacles
cube_name_list = ['cube_black', 'cube_green', 'cube']
total_cubes_number = 10
cube_list = [random.choice(cube_name_list) for _ in range(total_cubes_number)]

# Stable obstacle positions
p.loadURDF('data/cube_black/marble_cube.urdf', (5, 5, 0.2))
p.loadURDF('data/cube/marble_cube.urdf', (3, 3, 0.2))

# Random obstacle positions
for i in range(total_cubes_number):
    position = random_obstacles()
    p.loadURDF(f'data/{cube_list[i]}/marble_cube.urdf', position)

# Initialize car joints
wheels = [8, 15]
steering = [0, 2]
hokuyo_joint = 4

for wheel in range(p.getNumJoints(car)):
    p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

# Lidar settings
numRays = 100
rayLen = 8
rayStartLen = 0.25
rayFrom = []
rayTo = []
rayIds = []
rayHitColor = [1, 0, 0]
rayMissColor = [0, 1, 0]

for i in range(numRays):
    angle = -0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays
    rayFrom.append([rayStartLen * math.sin(angle), rayStartLen * math.cos(angle), 0])
    rayTo.append([rayLen * math.sin(angle), rayLen * math.cos(angle), 0])
    rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint))


############################################### Navigation Functions ##################################################

# Control the car's velocity and steering
def control_car(car, target_velocity, steering_angle):
    for wheel in wheels:
        p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=target_velocity, force=20)
    for steer in steering:
        p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steering_angle)

# Navigate the car to the target position
def navigate_to_target(car, target_position):
    while True:
        # Get current car position
        car_position, car_orientation = p.getBasePositionAndOrientation(car)
        car_position = np.array(car_position[:2])
        distance_to_target = np.linalg.norm(target_position - car_position)

        # Check if the car has reached the target
        if distance_to_target < 0.5:
            print("Target reached!")
            break

        # Lidar data processing
        results = p.rayTestBatch(rayFrom, rayTo, parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
        steering_angle = 0  # Default steering
        target_velocity = 5  # Default velocity

        for result in results:
            hit_fraction = result[2]
            if hit_fraction < 0.2:  # Obstacle is close
                steering_angle += 0.1  # Adjust to avoid obstacle
                target_velocity = 3  # Reduce speed near obstacles

        # Adjust steering angle to target
        direction_to_target = target_position - car_position
        target_angle = math.atan2(direction_to_target[1], direction_to_target[0])
        steering_angle += target_angle

        # Control the car
        control_car(car, target_velocity, steering_angle)

        # Step simulation
        if not useRealTimeSim:
            p.stepSimulation()
        time.sleep(0.01)


############################################### Simulation Loop #######################################################

# Define target position
target_position = np.array([11, 11])

# Navigate to the target
navigate_to_target(car, target_position)

# Keep simulation running after reaching the target
while True:
    time.sleep(1)
