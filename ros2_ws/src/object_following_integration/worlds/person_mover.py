#!/usr/bin/env python3
"""
Controller to make person move randomly within floor boundaries
Floor is 10x10, so boundaries are -5 to +5 in x and y
"""
from controller import Robot, Supervisor
import random
import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Floor boundaries (10x10 floor centered at origin)
MIN_X = -4.0
MAX_X = 4.0
MIN_Y = -4.0
MAX_Y = 4.0

# Movement parameters
SPEED = 0.5  # m/s
TURN_PROBABILITY = 0.02  # Probability of changing direction each step
MIN_DISTANCE = 0.5  # Minimum distance to travel before turning

# Get the person node
person_node = robot.getFromDef("MOVING_PERSON")
if person_node is None:
    print("ERROR: MOVING_PERSON not found!")
    exit(1)

# Initial position
x = 2.0
y = 0.0
z = 0.9

# Random initial direction
angle = random.uniform(0, 2 * math.pi)
distance_traveled = 0.0

while robot.step(timestep) != -1:
    dt = timestep / 1000.0  # Convert to seconds
    
    # Check if we should turn (random or hit boundary)
    should_turn = False
    
    # Random turn
    if random.random() < TURN_PROBABILITY:
        should_turn = True
    
    # Check boundaries
    next_x = x + SPEED * math.cos(angle) * dt
    next_y = y + SPEED * math.sin(angle) * dt
    
    if next_x < MIN_X or next_x > MAX_X or next_y < MIN_Y or next_y > MAX_Y:
        should_turn = True
        # Bounce off boundary
        if next_x < MIN_X or next_x > MAX_X:
            angle = math.pi - angle
        if next_y < MIN_Y or next_y > MAX_Y:
            angle = -angle
    
    # Turn if needed
    if should_turn:
        # Random new direction
        angle = random.uniform(0, 2 * math.pi)
        distance_traveled = 0.0
    
    # Move in current direction
    x += SPEED * math.cos(angle) * dt
    y += SPEED * math.sin(angle) * dt
    distance_traveled += SPEED * dt
    
    # Keep within boundaries
    x = max(MIN_X, min(MAX_X, x))
    y = max(MIN_Y, min(MAX_Y, y))
    
    # Update position
    person_node.getField("translation").setSFVec3f([x, y, z])
    
    # Reset distance counter periodically
    if distance_traveled > MIN_DISTANCE:
        distance_traveled = 0.0
