# Autonomous Person-Following Robot System  
**ROS2-Based Modular Architecture with Deep Learning Detection and Behavior-Driven Control**

## Repository Access

The complete implementation of this project, including source code, configuration files, trained model artifacts, and simulation assets, is maintained in a publicly accessible GitHub repository.

**Repository Link:**  
https://github.com/AbdulqadeerTHD/Object-Following-with-Behavior-Tree-ML-Tracking

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

---

## System Requirements

The system is designed for operation in a Linux-based robotics development environment supporting ROS2 middleware and simulation-based robotic modeling.

A typical runtime environment includes:
- A ROS2-compatible operating system and middleware distribution (e.g., ROS2 Humble)  
- The Webots robotics simulator for virtual robot and sensor modeling  
- A Python runtime environment for perception, tracking, and control modules  
- Standard numerical and computer vision libraries for machine learning inference and sensor processing  
- Sufficient computational resources to support real-time perception and control workloads  

---

## Workspace Structure

The project follows a modular ROS2 workspace layout in which system functionality is divided into independent packages representing perception, behavior management, control, and system integration.

A typical workspace structure is as follows:

ros2_ws/
├── src/
│ ├── object_following_vision/
│ ├── object_following_bt/
│ ├── object_following_control/
│ └── object_following_integration/
├── runs/
│ └── detect/
│ └── person_obstacle_detector/
│ └── weights/
│ └── best.pt
└── training_data/
├── person/
├── obstacles/
└── combined_dataset/


---

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

---

## Configuration and Adaptation

System behavior is governed through a combination of configuration files, launch parameters, and internal node-level settings. These mechanisms influence:
- Sensor input sources and update rates within the simulation environment  
- Detection confidence thresholds and tracking persistence parameters  
- State transition timing and behavioral completion criteria  
- Motion control gains, velocity limits, and obstacle detection thresholds  

This design allows the system to be adapted to different experimental conditions and simulation scenarios without requiring architectural changes.

---

## Model Training Overview

The perception subsystem uses a custom-trained YOLOv8 model for person detection. Training artifacts and model weights are maintained within the repository.

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

---

## Reproducibility and Documentation

This repository includes structured documentation describing:
- System architecture and package organization  
- Workspace layout and build workflow  
- Execution sequence and runtime behavior  
- Configuration and adaptation guidelines  

This documentation is intended to support independent system setup and execution within a compatible development environment by following the information provided in this repository.

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

System operation is initiated through a centralized launch interface that coordinates:
- Initialization of the Webots simulation environment
- Activation of ROS2 middleware services
- Startup of perception, behavior management, and control nodes

The primary launch configuration for this system is located within the system integration package and is designed to be invoked from the workspace environment after successful compilation.

---


