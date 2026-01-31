# Autonomous Person-Following Robot System

**ROS2-Based Modular Architecture with Deep Learning Detection and Behavior-Driven Control**

## Repository Access

The complete implementation of this project, including source code, configuration files, trained model artifacts, and simulation assets, is maintained in a publicly accessible GitHub repository.


This repository serves as the primary reference for system structure, package organization, and execution workflow. All components required for system setup and operation are provided within this repository.

---

## Abstract

This project presents an autonomous person-following robot developed using a modular ROS2-based architecture. The system integrates deep learning-based person detection using YOLOv8, multi-object tracking through the SORT algorithm, and a behavior-driven state machine to support intelligent autonomous navigation.

Obstacle avoidance is implemented using LiDAR-based reactive control, enabling the robot to safely navigate structured indoor environments. The system operates within a Webots simulation environment and follows a three-layer design paradigm that separates perception, decision-making, and control into independent and extensible subsystems.

---
## System Overview

The system is organized into three primary ROS2 nodes:

1. **Vision Node (Perception Layer)**  
   Performs real-time person detection using a custom-trained YOLOv8 model and maintains consistent person identities using the SORT tracking algorithm.

2. **Behavior Manager Node (Decision Layer)**  
   Implements a state-based control structure (SEARCH → FOLLOW → WAIT) responsible for target selection, re-identification, and high-level behavior transitions.

3. **Controller Node (Control Layer)**  
   Converts behavior commands into velocity outputs and enforces safety constraints using LiDAR-based obstacle detection and avoidance strategies.

These subsystems communicate through standardized ROS2 topics to maintain a continuous perception–decision–action pipeline.

### ROS2 Topics

#### Published Topics

- **`/detected_persons`** (`std_msgs/String`)  
  Publisher: `vision_yolo_node`  
  Format: JSON array of person detections  
  Fields: `id` (int), `x_center` (float), `area` (float)  
  Frequency: ~10–30 Hz (depends on camera frame rate)

- **`/behavior_cmd`** (`std_msgs/String`)  
  Publisher: `behavior_manager_node`  
  Format: String command or JSON for FOLLOW  
  Commands: `"ROTATE"`, `"STOP"`, or `{"cmd": "FOLLOW", "x": float, "area": float}`  
  Frequency: 10 Hz

- **`/cmd_vel`** (`geometry_msgs/Twist`)  
  Publisher: `controller_node`  
  Format: Standard ROS2 velocity command  
  Fields: `linear.x` (m/s), `angular.z` (rad/s)  
  Frequency: 10 Hz

#### Subscribed Topics

- **`/camera/image_raw`** (`sensor_msgs/Image`)  
  Subscriber: `vision_yolo_node`  
  Source: Robot camera (Webots simulation)  
  Format: RGB image (640×480)

- **`/scan`** (`sensor_msgs/LaserScan`)  
  Subscriber: `controller_node`  
  Source: LiDAR sensor  
  Used for: Obstacle detection and avoidance

---

## System Requirements

The system is designed for operation in a Linux-based robotics development environment supporting ROS2 middleware and simulation-based robotic modeling.

A typical runtime environment includes:
<<<<<<< HEAD
- A ROS2-compatible operating system and middleware distribution (e.g., ROS2 Humble)  
- The Webots robotics simulator for virtual robot and sensor modeling  
- A Python runtime environment for perception, tracking, and control modules  
- Standard numerical and computer vision libraries for machine learning inference and sensor processing  
- Sufficient computational resources to support real-time perception and control workloads  
=======

- A ROS2-compatible operating system and middleware distribution (e.g., ROS2 Humble)
- The Webots robotics simulator for virtual robot and sensor modeling
- A Python runtime environment (Python 3.10 or newer) for perception, tracking, and control modules
- Standard numerical and computer vision libraries for machine learning inference and sensor processing
- Sufficient computational resources to support real-time perception and control workloads

---
## File Structure

```
ros2_ws/
├── src/
│   ├── object_following_vision/          # Vision and detection package
│   │   └── object_following_vision/
│   │       └── vision_yolo_node.py        # YOLOv8 + SORT tracking
│   ├── object_following_bt/              # Behavior tree package
│   │   └── object_following_bt/
│   │       └── behavior_manager_node.py  # State machine logic
│   ├── object_following_control/         # Control package
│   │   └── object_following_control/
│   │       └── controller_node.py        # Motor control + obstacle avoidance
│   └── object_following_integration/     # Integration package
│       └── launch/
│           └── launch_3node_architecture.launch.py
├── runs/
│   └── detect/
│       └── person_obstacle_detector/     # Trained YOLOv8 model
│           └── weights/
│               └── best.pt
├── sort/                                 # SORT tracking library
│   └── sort.py
└── training_data/                        # Training dataset
    ├── person/
    └── obstacles/
```


## Installation and Environment Preparation

System setup follows a standard ROS2 workspace workflow and assumes that the development environment provides compatible versions of ROS2, Webots, and Python-based machine learning libraries.

The general preparation process includes:
- Initializing a ROS2-compatible workspace structure  
- Integrating the project repository into the workspace source directory  
- Resolving external software and library dependencies  
- Building the workspace using a supported ROS2 build system  
- Configuring runtime environment variables to enable package discovery and execution  

Additional configuration may be required depending on the specific simulation environment, ROS2 distribution, and host system configuration.

---

## System Build Procedure

Once the workspace is prepared and dependencies are resolved, the system packages are compiled using the standard ROS2 build workflow. The build process generates the necessary runtime artifacts and establishes inter-package communication paths required for system execution.

Following a successful build, the runtime environment is configured to ensure that the system packages and launch resources are discoverable by the ROS2 middleware.
=
---

## Quick Setup and Build

The following steps prepare the workspace and build the packages. Ensure ROS2 Humble (or a compatible distribution) and Webots are installed before proceeding.

### 1. Clone the repository

```bash
git clone https://github.com/AbdulqadeerTHD/Object-Following-with-Behavior-Tree-ML-Tracking.git
cd Object-Following-with-Behavior-Tree-ML-Tracking
```

### 2. Install Python dependencies

From the repository root, install the required Python packages (e.g., Ultralytics YOLOv8, NumPy, SORT dependencies):

```bash
pip3 install -r requirements.txt
```

If a `requirements.txt` is located inside `ros2_ws/`, run the same command from that directory instead.

### 3. Build the ROS2 workspace

Navigate to the workspace and build the packages using the standard ROS2 build workflow:

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

### 4. Verify nodes and topics (optional)

After launching the system, you can inspect active nodes and topics in another terminal (with the workspace sourced):

```bash
ros2 node list
ros2 topic list
ros2 topic info /detected_persons
ros2 topic info /behavior_cmd
ros2 topic info /cmd_vel
```
---

## Simulation and Execution

System execution is coordinated through a centralized launch interface that initializes the Webots simulation environment, establishes ROS2 communication services, and activates the application-level nodes.

A typical execution workflow includes:

- Launching the Webots simulation environment and robotic platform model  
- Initializing ROS2 middleware services and topic interfaces  
- Activating the perception subsystem for camera-based detection and multi-object tracking  
- Enabling the behavior management subsystem for state-based decision-making  
- Running the control subsystem for motion execution and obstacle safety enforcement  

These components operate concurrently and communicate through standardized ROS2 messaging interfaces to maintain continuous autonomous operation.

- Launching the Webots simulation environment and robotic platform model
- Initializing ROS2 middleware services and topic interfaces
- Activating the perception subsystem for camera-based detection and multi-object tracking
- Enabling the behavior management subsystem for state-based decision-making
- Running the control subsystem for motion execution and obstacle safety enforcement

The primary launch configuration is located within the **object_following_integration** package (see File Structure). Ensure the workspace is built and sourced before running the simulation. Webots must be installed and available in the environment.
---

## Configuration and Adaptation

System behavior is governed through a combination of configuration files, launch parameters, and internal node-level settings. These mechanisms influence:
<<<<<<< HEAD
- Sensor input sources and update rates within the simulation environment  
- Detection confidence thresholds and tracking persistence parameters  
- State transition timing and behavioral completion criteria  
- Motion control gains, velocity limits, and obstacle detection thresholds  


- Sensor input sources and update rates within the simulation environment
- Detection confidence thresholds and tracking persistence parameters
- State transition timing and behavioral completion criteria
- Motion control gains, velocity limits, and obstacle detection thresholds
This design allows the system to be adapted to different experimental conditions and simulation scenarios without requiring architectural changes.

---

## Model Training Overview

The perception subsystem uses a custom-trained YOLOv8 model for person detection. Training artifacts and model weights are maintained within the repository.

The perception subsystem uses a custom-trained YOLOv8 model for person detection. Training artifacts and model weights are maintained within the repository (see File Structure: `runs/detect/person_obstacle_detector/weights/`).
Training is conducted using a staged fine-tuning pipeline based on a lightweight YOLOv8 base model. The process supports independent training of person detection, obstacle detection, and combined detection models, depending on experimental requirements.

---

## Dependencies

The system relies on the following primary software components:

- ROS2 Humble  
- Webots Robotics Simulator  
- Python 3.10 or newer  
- Ultralytics YOLOv8  
- NumPy  
- cv_bridge  
- SORT tracking library  


- ROS2 Humble
- Webots Robotics Simulator
- Python 3.10 or newer
- Ultralytics YOLOv8
- NumPy
- cv_bridge (ROS2)
- SORT tracking library (included in `ros2_ws/sort/`)
---

## Deployment Considerations

Although the system is validated primarily within a simulated environment, the modular software architecture is designed to support transition to physical robotic platforms. Deployment on real hardware may require additional platform-specific configuration, including sensor driver integration, specific data training as per real Environment, parameter tuning, and safety validation procedures.

---
## Execution Reference

This section provides a formal reference for system initialization and execution within a compatible ROS2 and Webots development environment.

### Repository Integration

The project repository is intended to be placed within the source directory of a ROS2 workspace. A typical integration process consists of obtaining the repository from the public version control system and positioning it within the workspace structure used for package discovery and build operations.

### Build Procedure

Following repository integration, the workspace is compiled using a ROS2-compatible build system. This process generates the runtime artifacts required for inter-package communication and system execution. The runtime environment is then configured to ensure the availability of the built packages and associated launch resources.

### System Launch

Although the system is validated primarily within a simulated environment, the modular software architecture is designed to support transition to physical robotic platforms. Deployment on real hardware may require additional platform-specific configuration, including sensor driver integration, environment-specific data training, parameter tuning, and safety validation procedures.

---

## Execution Reference

### Repository integration

The project repository is intended to be cloned and used as (or placed within) a ROS2 workspace. The `ros2_ws` directory contains the source packages, and the build is performed from within `ros2_ws` as described in **Quick Setup and Build**.

### Build procedure

Following repository integration and dependency installation, the workspace is compiled using `colcon build` from the `ros2_ws` directory. The runtime environment is then configured with `source install/setup.bash` so that packages and launch files are discoverable.

### System launch

System operation is initiated through the launch file provided in the **object_following_integration** package. It coordinates initialization of the Webots simulation environment, activation of ROS2 middleware services, and startup of the perception, behavior management, and control nodes. Invoke it from a terminal in which the workspace has been built and sourced.

---
System operation is initiated through a centralized launch interface that coordinates:
- Initialization of the Webots simulation environment
- Activation of ROS2 middleware services
- Startup of perception, behavior management, and control nodes

The primary launch configuration for this system is located within the system integration package and is designed to be invoked from the workspace environment after successful compilation.

---
