# Quickstart Guide: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: Module 2 - The Digital Twin (Gazebo & Unity)
**Date**: 2025-12-17
**Status**: Draft

## Prerequisites

Before starting with the digital twin module, ensure you have:

1. **ROS 2 Humble Hawksbill** installed on Ubuntu 20.04+ (or WSL2 if on Windows)
2. **Gazebo Garden** or **Gazebo Harmonic** installed
3. **Unity 2022.3 LTS** installed (Windows or macOS)
4. **Python 3.10+** with pip
5. **Node.js 18+** for the frontend components
6. **Completed Module 1** (ROS 2 fundamentals) concepts

## Setup Instructions

### 1. Environment Setup

First, clone the repository and navigate to the project directory:

```bash
cd C:\Users\saad\Desktop\Hackathon-I
```

### 2. Backend Setup

Navigate to the backend directory and install Python dependencies:

```bash
cd backend
pip install -r requirements.txt  # if requirements.txt exists
# Or install required packages:
pip install rclpy pydantic fastapi uvicorn
```

### 3. Gazebo Setup

Ensure Gazebo is properly installed and test it:

```bash
# On Linux/WSL:
gz sim --version  # Should show Gazebo version

# For ROS 2 integration:
source /opt/ros/humble/setup.bash
ros2 pkg list | grep gazebo  # Should show gazebo-related packages
```

### 4. Unity Setup

1. Open Unity Hub
2. Install Unity 2022.3 LTS
3. Install the Unity Robotics Package through the Package Manager
4. Import the Unity ROS TCP Connector

### 5. Frontend Setup

Navigate to the frontend directory and install dependencies:

```bash
cd frontend_ai_book
npm install
```

## Running the Digital Twin

### 1. Start the Backend Services

```bash
cd backend
# Source ROS environment
source /opt/ros/humble/setup.bash
# Run the simulation backend
python -m src.main  # or the appropriate entry point
```

### 2. Start the Gazebo Simulation

```bash
# In a new terminal
source /opt/ros/humble/setup.bash
# Launch the digital twin simulation
ros2 launch digital_twin_gazebo.launch.py  # placeholder name
```

### 3. Run the Unity Environment

1. Open the Unity project in the Unity editor
2. Load the Digital Twin scene
3. Press Play to start the visualization

### 4. Start the Frontend

```bash
cd frontend_ai_book
npm start
```

## Core Components

### Chapter 1: Physics Simulation with Gazebo

**Objective**: Learn to simulate humanoid robot dynamics using Gazebo

**Key Activities**:
1. Import humanoid robot URDF into Gazebo
2. Configure physics parameters (gravity, friction, damping)
3. Test joint constraints and collision detection
4. Apply external forces and observe reactions

**Test Command**:
```bash
# Verify physics simulation is working
ros2 run digital_twin_gazebo test_physics.py
```

### Chapter 2: Digital Environments & Human-Robot Interaction in Unity

**Objective**: Create high-fidelity virtual environments for robot interaction

**Key Activities**:
1. Import robot model into Unity scene
2. Create interactive environment objects
3. Implement human-robot interaction constraints
4. Record and playback robot trajectories

**Test Command**:
```bash
# Verify Unity integration
npm run test unity-integration
```

### Chapter 3: Sensor Simulation

**Objective**: Simulate robot sensors to generate synthetic training data

**Key Activities**:
1. Attach virtual sensors to robot model
2. Configure sensor parameters (LiDAR, depth camera, IMU)
3. Generate synthetic sensor data
4. Validate sensor accuracy vs. ground truth

**Test Command**:
```bash
# Verify sensor simulation
ros2 run digital_twin_sensors test_sensors.py
```

## Integration with Chatbot

The digital twin integrates with the existing chatbot system to provide guided learning experiences:

1. **Content Queries**: Ask the chatbot about simulation concepts
2. **Troubleshooting**: Get help with setup and configuration issues
3. **Scenario Guidance**: Receive step-by-step instructions for simulation tasks

Example query: "Explain how to configure joint limits in Gazebo for a humanoid robot"

## Troubleshooting

### Common Issues

1. **Gazebo won't start**:
   - Ensure ROS 2 environment is sourced
   - Check if display is properly configured (X11 forwarding for WSL2)

2. **Unity-ROS communication fails**:
   - Verify ROS TCP Connector settings
   - Check that both ROS_MASTER_URI and ROS_IP are properly set

3. **Simulation runs slowly**:
   - Reduce physics update rate in Gazebo
   - Lower rendering quality in Unity

### Verification Commands

```bash
# Check all required services are running
ros2 node list
ros2 topic list

# Verify sensor data is being generated
ros2 topic echo /robot/sensors/lidar_scan
```

## Next Steps

After completing the setup:

1. Work through Chapter 1: Physics Simulation with Gazebo
2. Proceed to Chapter 2: Digital Environments in Unity
3. Complete Chapter 3: Sensor Simulation
4. Integrate all components for full digital twin experience
5. Test sim-to-real transfer capabilities