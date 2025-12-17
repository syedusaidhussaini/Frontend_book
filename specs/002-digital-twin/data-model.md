# Data Model: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: Module 2 - The Digital Twin (Gazebo & Unity)
**Date**: 2025-12-17
**Status**: Design

## Core Entities

### 1. RobotModel
**Description**: Represents a humanoid robot in simulation with physical and kinematic properties

**Fields**:
- `id` (string): Unique identifier for the robot model
- `name` (string): Display name of the robot
- `urdf_path` (string): Path to URDF file describing robot structure
- `links` (array[Link]): List of rigid body links in the robot
- `joints` (array[Joint]): List of joints connecting the links
- `actuators` (array[Actuator]): List of motor actuators
- `sensors` (array[Sensor]): List of sensors attached to the robot
- `physical_properties` (PhysicalProperties): Mass, friction, and other physical parameters

**Relationships**:
- One-to-many with Joint (robot has many joints)
- One-to-many with Link (robot has many links)
- One-to-many with Sensor (robot has many sensors)

### 2. Link
**Description**: A rigid body component of the robot

**Fields**:
- `id` (string): Unique identifier for the link
- `name` (string): Name of the link
- `mass` (float): Mass of the link in kg
- `inertia` (Inertia): Inertia tensor of the link
- `visual_mesh` (string): Path to visual mesh file
- `collision_mesh` (string): Path to collision mesh file
- `position` (Vector3): Position in 3D space
- `orientation` (Quaternion): Orientation as quaternion

### 3. Joint
**Description**: Connection between two links with specific degrees of freedom

**Fields**:
- `id` (string): Unique identifier for the joint
- `name` (string): Name of the joint
- `type` (string): Joint type (revolute, prismatic, fixed, etc.)
- `parent_link` (string): ID of parent link
- `child_link` (string): ID of child link
- `axis` (Vector3): Joint axis of rotation/translation
- `limits` (JointLimits): Position, velocity, and effort limits
- `dynamics` (JointDynamics): Damping, friction, spring parameters

### 4. Sensor
**Description**: Virtual sensor attached to robot for data generation

**Fields**:
- `id` (string): Unique identifier for the sensor
- `name` (string): Name of the sensor
- `type` (string): Sensor type (lidar, depth_camera, imu, etc.)
- `parent_link` (string): Link to which sensor is attached
- `position` (Vector3): Position relative to parent link
- `orientation` (Quaternion): Orientation relative to parent link
- `parameters` (SensorParameters): Type-specific parameters
- `data_stream` (string): ROS topic for sensor data

### 5. PhysicsSimulator
**Description**: Manages dynamics simulation in Gazebo

**Fields**:
- `id` (string): Unique identifier for the simulator instance
- `engine` (string): Physics engine name (e.g., Bullet, ODE)
- `gravity` (Vector3): Gravity vector (default: [0, 0, -9.81])
- `time_step` (float): Simulation time step in seconds
- `real_time_factor` (float): Real-time factor (1.0 = real-time)
- `collision_detection` (CollisionSettings): Collision detection parameters
- `contact_properties` (ContactProperties): Contact model parameters

### 6. SceneEnvironment
**Description**: Virtual environment containing robot, objects, and lighting

**Fields**:
- `id` (string): Unique identifier for the scene
- `name` (string): Display name of the scene
- `objects` (array[SceneObject]): List of objects in the scene
- `lighting` (LightingSettings): Lighting configuration
- `physics_properties` (PhysicsProperties): Environment physics settings
- `environment_map` (string): Path to environment map file

### 7. SceneObject
**Description**: An object in the simulation environment

**Fields**:
- `id` (string): Unique identifier for the object
- `name` (string): Name of the object
- `type` (string): Object type (static, dynamic, interactive)
- `mesh_path` (string): Path to mesh file
- `position` (Vector3): Position in 3D space
- `orientation` (Quaternion): Orientation as quaternion
- `physical_properties` (PhysicalProperties): Mass, friction, etc.
- `interaction_properties` (InteractionProperties): Interaction settings

### 8. TrajectoryData
**Description**: Time-series recording of robot states and sensor readings

**Fields**:
- `id` (string): Unique identifier for the trajectory
- `robot_id` (string): ID of the robot being recorded
- `start_time` (datetime): Start time of the recording
- `end_time` (datetime): End time of the recording
- `joint_states` (array[JointState]): Time-series joint state data
- `sensor_data` (array[SensorData]): Time-series sensor data
- `control_commands` (array[ControlCommand]): Time-series control commands
- `interaction_forces` (array[InteractionForce]): Time-series interaction forces

## Data Relationships

```
RobotModel 1 ---- * Joint
RobotModel 1 ---- * Link
RobotModel 1 ---- * Sensor
PhysicsSimulator 1 ---- * RobotModel
SceneEnvironment 1 ---- * SceneObject
TrajectoryData * ---- 1 RobotModel
TrajectoryData * ---- * JointState
TrajectoryData * ---- * SensorData
```

## State Transitions

### RobotModel States
- `DESIGN`: Model is being designed (URDF creation)
- `VALIDATED`: Model has passed validation checks
- `SIMULATING`: Model is active in simulation
- `RECORDING`: Model trajectory is being recorded
- `EXPORTED`: Model has been exported for use

### Simulation States
- `STOPPED`: Simulation is not running
- `PAUSED`: Simulation is paused
- `RUNNING`: Simulation is actively running
- `ERROR`: Simulation encountered an error

## Validation Rules

1. **RobotModel Validation**:
   - URDF file must exist and be valid
   - All links must have defined mass > 0
   - All joints must connect valid parent-child links
   - Joint limits must be within physical constraints

2. **Joint Validation**:
   - Parent and child links must exist in the robot
   - Joint axis must be normalized
   - Joint limits must be physically realistic

3. **Sensor Validation**:
   - Parent link must exist in the robot
   - Sensor parameters must match sensor type
   - Sensor position must be within robot bounds

4. **Trajectory Validation**:
   - Time stamps must be sequential
   - Joint states must match robot configuration
   - Sensor data must be consistent with simulation

## Serialization Formats

### Robot Model (URDF)
```xml
<robot name="humanoid_robot">
  <link name="base_link">
    <inertial>...</inertial>
    <visual>...</visual>
    <collision>...</collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

### Sensor Configuration (YAML)
```yaml
sensors:
  lidar_1:
    type: "ray"
    parent_link: "head_link"
    parameters:
      scan:
        range_min: 0.1
        range_max: 10.0
        resolution: 0.01
      noise:
        type: "gaussian"
        mean: 0.0
        std: 0.01
```

### Trajectory Data (JSON)
```json
{
  "id": "trajectory_001",
  "robot_id": "humanoid_01",
  "timestamps": [0.0, 0.1, 0.2],
  "joint_positions": [
    {"joint1": 0.0, "joint2": 0.5},
    {"joint1": 0.1, "joint2": 0.6},
    {"joint1": 0.2, "joint2": 0.7}
  ],
  "sensor_readings": [...]
}
```