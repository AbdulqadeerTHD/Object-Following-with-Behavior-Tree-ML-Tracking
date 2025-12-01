#!/usr/bin/env python3
"""
Supervisor controller to automatically enable camera device
This runs as the Ros2Supervisor to enable the camera after Webots loads
"""
from controller import Supervisor
import time

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

print("Camera Enabler Supervisor: Starting...")

# Wait for everything to initialize
for i in range(100):  # Wait 2 seconds
    robot.step(timestep)

# Try multiple ways to find and enable the camera
camera_enabled = False

# Method 1: Find robot by DEF name
robot_node = robot.getFromDef("TurtleBot3Burger")
if robot_node is None:
    # Method 2: Find robot by name
    robot_node = robot.getFromDef("TurtleBot3Burger")
    if robot_node is None:
        # Method 3: Get root and search
        root = robot.getRoot()
        children = root.getField("children")
        for i in range(children.getCount()):
            child = children.getMFNode(i)
            if child.getTypeName() == "TurtleBot3Burger":
                robot_node = child
                break

if robot_node:
    print(f"Found robot node: {robot_node.getTypeName()}")
    
    # Try to find camera in extensionSlot
    extension_slot = robot_node.getField("extensionSlot")
    if extension_slot:
        print(f"Found extensionSlot with {extension_slot.getCount()} items")
        for i in range(extension_slot.getCount()):
            item = extension_slot.getMFNode(i)
            if item.getTypeName() == "Camera":
                print(f"Found Camera at index {i}")
                enabled_field = item.getField("enabled")
                if enabled_field:
                    enabled_field.setSFBool(True)
                    print("Camera enabled successfully!")
                    camera_enabled = True
                    break
            elif item.getTypeName() == "Solid":
                # Check children of Solid
                children_field = item.getField("children")
                if children_field:
                    for j in range(children_field.getCount()):
                        child = children_field.getMFNode(j)
                        if child.getTypeName() == "Camera":
                            print(f"Found Camera in Solid at index {i}, child {j}")
                            enabled_field = child.getField("enabled")
                            if enabled_field:
                                enabled_field.setSFBool(True)
                                print("Camera enabled successfully!")
                                camera_enabled = True
                                break
else:
    print("ERROR: Could not find TurtleBot3Burger node")

if not camera_enabled:
    print("WARNING: Could not automatically enable camera. Please enable manually in Webots GUI.")
    print("   Steps: Select robot → camera → set enabled=TRUE")

# Keep running
while robot.step(timestep) != -1:
    pass
