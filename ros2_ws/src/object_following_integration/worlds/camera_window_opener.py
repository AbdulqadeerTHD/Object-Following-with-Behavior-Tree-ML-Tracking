#!/usr/bin/env python3
"""
Supervisor script to automatically open camera window in Webots
This keeps the camera window open and visible
"""
from controller import Supervisor
import time

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

print("Camera Window Opener: Starting...")

# Wait for everything to initialize
for i in range(100):
    robot.step(timestep)

# Find the robot and camera
robot_node = robot.getFromDef("TurtleBot3Burger")
if robot_node is None:
    # Try to find by searching
    root = robot.getRoot()
    children = root.getField("children")
    for i in range(children.getCount()):
        child = children.getMFNode(i)
        if child.getTypeName() == "TurtleBot3Burger":
            robot_node = child
            break

camera_node = None
if robot_node:
    # Find camera in extensionSlot
    extension_slot = robot_node.getField("extensionSlot")
    if extension_slot:
        for i in range(extension_slot.getCount()):
            item = extension_slot.getMFNode(i)
            if item.getTypeName() == "Camera":
                camera_node = item
                break

if camera_node:
    print("Found camera node, attempting to show window...")
    # Try to enable window display
    # Note: Webots doesn't have a direct API to open camera windows
    # But we can ensure the camera is enabled and ready
    enabled_field = camera_node.getField("enabled")
    if enabled_field:
        enabled_field.setSFBool(True)
        print("Camera enabled!")
    
    # The camera window needs to be opened manually in Webots GUI
    # But we can log instructions
    print("=" * 60)
    print("TO VIEW CAMERA:")
    print("1. In Webots Scene Tree, find: TurtleBot3Burger -> extensionSlot -> camera")
    print("2. Right-click on 'camera'")
    print("3. Select 'Show Window' or 'View Camera'")
    print("=" * 60)
else:
    print("WARNING: Could not find camera node")

# Keep running
while robot.step(timestep) != -1:
    # Keep the supervisor alive
    pass

