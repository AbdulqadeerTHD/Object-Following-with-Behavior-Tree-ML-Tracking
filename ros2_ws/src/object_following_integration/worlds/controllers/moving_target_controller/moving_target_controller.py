#!/usr/bin/env python3
"""
Supervisor controller to move the blue target box back and forth
This simulates a person walking back and forth
Speed: 0.2 m/s
Movement: Forward 1.0m, pause 2s, backward to start, repeat
"""
from controller import Supervisor
import time

# Initialize supervisor
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Get the moving target node - try by DEF name first, then by name
target_node = robot.getFromDef("MOVING_TARGET")
if target_node is None:
    # Try by name "TargetPerson"
    root = robot.getRoot()
    children = root.getField("children")
    for i in range(children.getCount()):
        child = children.getMFNode(i)
        if child.getTypeName() == "Solid":
            name_field = child.getField("name")
            if name_field:
                name = name_field.getSFString()
                if name == "TargetPerson" or name == "moving_target":
                    target_node = child
                    print(f"Found target by name: {name}")
                    break

if target_node is None:
    print("ERROR: Could not find MOVING_TARGET or TargetPerson node!")
    print("Searching all nodes...")
    root = robot.getRoot()
    children = root.getField("children")
    for i in range(children.getCount()):
        child = children.getMFNode(i)
        if child.getTypeName() == "Solid":
            name_field = child.getField("name")
            if name_field:
                print(f"Found Solid: {name_field.getSFString()}")
    robot.simulationQuit(1)
    exit(1)

# Get translation field
translation_field = target_node.getField("translation")

# Initial position
initial_x = 1.5
current_x = initial_x
target_x = initial_x + 1.0  # Move forward 1.0m
speed = 0.2  # m/s
direction = 1  # 1 = forward, -1 = backward
pause_time = 0
pause_duration = 2000  # 2 seconds in milliseconds (100 timesteps at 20ms = 2000ms)

print("Moving Target Controller: Starting...")
print(f"Initial position: {initial_x}")
print(f"Target position: {target_x}")
print(f"Speed: {speed} m/s")
print(f"Movement: Forward 1.0m, pause 2s, backward, repeat")

# Main loop
while robot.step(timestep) != -1:
    # If paused, wait
    if pause_time > 0:
        pause_time -= timestep
        continue
    
    # Calculate movement
    distance_per_step = (speed * timestep) / 1000.0  # Convert to meters per step
    
    # Move in current direction
    current_x += direction * distance_per_step
    
    # Check if reached target
    if direction == 1 and current_x >= target_x:
        # Reached forward position, pause then reverse
        current_x = target_x
        direction = -1
        pause_time = pause_duration
        print(f"Reached forward position: {current_x}, pausing 2s then reversing...")
    elif direction == -1 and current_x <= initial_x:
        # Reached backward position, pause then go forward
        current_x = initial_x
        direction = 1
        pause_time = pause_duration
        print(f"Reached backward position: {current_x}, pausing 2s then going forward...")
    
    # Update position (keep Y=0.0, Z=0.8 - on ground, height/2 = 1.6/2 = 0.8)
    translation_field.setSFVec3f([current_x, 0.0, 0.8])

print("Moving Target Controller: Exiting...")

