#!/usr/bin/env python3
"""
Controller to make person move randomly within floor boundaries
Floor is 15x15, so boundaries are -7.5 to +7.5 in x and y
This controller works with the Pedestrian's own node
"""
from controller import Robot, Supervisor
import random
import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Floor boundaries (15x15 floor centered at origin)
MIN_X = -7.0
MAX_X = 7.0
MIN_Y = -7.0
MAX_Y = 7.0

# Movement parameters
SPEED = 0.5  # m/s
TURN_PROBABILITY = 0.02  # Probability of changing direction each step
MIN_DISTANCE = 0.5  # Minimum distance to travel before turning

# Get the person node - try to get the robot's own node first
# In Webots, when a controller is assigned to a Pedestrian, we can get the node
person_node = robot.getSelf()
if person_node is None:
    # Fallback: try to get by DEF name
    person_node = robot.getFromDef("MOVING_PERSON")
    if person_node is None:
        # Try alternative DEF names
        person_node = robot.getFromDef("MOVING_PERSON_1")
        if person_node is None:
            person_node = robot.getFromDef("MOVING_PERSON_2")

if person_node is None:
    print("ERROR: Could not find person node!")
    exit(1)

# Get initial position from the node
initial_pos = person_node.getField("translation").getSFVec3f()
x = initial_pos[0]
y = initial_pos[1]
z = initial_pos[2]

# Random initial direction
angle = random.uniform(0, 2 * math.pi)
distance_traveled = 0.0

print(f"Person Mover Controller: Starting for node at ({x}, {y}, {z})")

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
