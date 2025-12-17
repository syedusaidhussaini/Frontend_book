"""
Data models for the Digital Twin Module

Defines Pydantic models for robot representations, physics simulation,
and sensor data structures.
"""

from typing import Dict, List, Optional, Union
from pydantic import BaseModel, Field
from enum import Enum
import math


class JointType(str, Enum):
    REVOLUTE = "revolute"
    PRISMATIC = "prismatic"
    FIXED = "fixed"
    CONTINUOUS = "continuous"
    PLANAR = "planar"
    FLOATING = "floating"


class Vector3(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class Quaternion(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


class JointLimits(BaseModel):
    position_min: float = Field(default=-math.pi, description="Minimum joint position")
    position_max: float = Field(default=math.pi, description="Maximum joint position")
    velocity_max: float = Field(default=10.0, description="Maximum joint velocity")
    effort_max: float = Field(default=100.0, description="Maximum joint effort")


class JointDynamics(BaseModel):
    damping: float = Field(default=0.1, description="Joint damping coefficient")
    friction: float = Field(default=0.01, description="Joint friction coefficient")
    spring_stiffness: float = Field(default=0.0, description="Spring stiffness coefficient")
    spring_damping: float = Field(default=0.0, description="Spring damping coefficient")


class JointInfo(BaseModel):
    name: str = Field(..., description="Unique name of the joint")
    joint_type: JointType = Field(..., description="Type of joint")
    parent_link: str = Field(..., description="Parent link name")
    child_link: str = Field(..., description="Child link name")
    axis: Vector3 = Field(default_factory=Vector3, description="Joint axis of rotation/translation")
    limits: JointLimits = Field(default_factory=JointLimits, description="Joint limits")
    dynamics: JointDynamics = Field(default_factory=JointDynamics, description="Joint dynamics parameters")


class LinkInfo(BaseModel):
    name: str = Field(..., description="Unique name of the link")
    mass: float = Field(default=1.0, description="Mass of the link in kg")
    position: Vector3 = Field(default_factory=Vector3, description="Position in 3D space")
    orientation: Quaternion = Field(default_factory=Quaternion, description="Orientation as quaternion")
    visual_mesh: Optional[str] = Field(default=None, description="Path to visual mesh file")
    collision_mesh: Optional[str] = Field(default=None, description="Path to collision mesh file")


class SensorType(str, Enum):
    LIDAR = "lidar"
    DEPTH_CAMERA = "depth_camera"
    IMU = "imu"
    CAMERA = "camera"
    FORCE_TORQUE = "force_torque"


class SensorParameters(BaseModel):
    # LiDAR parameters
    scan_range_min: Optional[float] = Field(default=0.1, description="Minimum scan range for LiDAR")
    scan_range_max: Optional[float] = Field(default=10.0, description="Maximum scan range for LiDAR")
    scan_resolution: Optional[float] = Field(default=0.01, description="Angular resolution for LiDAR")
    scan_field_of_view: Optional[float] = Field(default=2 * math.pi, description="Field of view for LiDAR")

    # Depth camera parameters
    camera_resolution_x: Optional[int] = Field(default=640, description="Horizontal resolution for depth camera")
    camera_resolution_y: Optional[int] = Field(default=480, description="Vertical resolution for depth camera")
    camera_field_of_view: Optional[float] = Field(default=math.pi/2, description="Field of view for depth camera")

    # IMU parameters
    imu_noise_density: Optional[float] = Field(default=0.01, description="Noise density for IMU")
    imu_random_walk: Optional[float] = Field(default=0.001, description="Random walk for IMU")


class SensorInfo(BaseModel):
    id: str = Field(..., description="Unique identifier for the sensor")
    name: str = Field(..., description="Display name of the sensor")
    type: SensorType = Field(..., description="Type of sensor")
    parent_link: str = Field(..., description="Link to which sensor is attached")
    position: Vector3 = Field(default_factory=Vector3, description="Position relative to parent link")
    orientation: Quaternion = Field(default_factory=Quaternion, description="Orientation relative to parent link")
    parameters: SensorParameters = Field(default_factory=SensorParameters, description="Sensor-specific parameters")


class RobotModel(BaseModel):
    id: str = Field(..., description="Unique identifier for the robot model")
    name: str = Field(..., description="Display name of the robot")
    urdf_path: str = Field(..., description="Path to URDF file describing robot structure")
    links: List[LinkInfo] = Field(default_factory=list, description="List of links in the robot")
    joints: List[JointInfo] = Field(default_factory=list, description="List of joints connecting the links")
    sensors: List[SensorInfo] = Field(default_factory=list, description="List of sensors attached to the robot")
    physical_properties: Dict[str, float] = Field(default_factory=dict, description="Physical properties (mass, friction, etc.)")


class PhysicsSimulatorConfig(BaseModel):
    gravity: Vector3 = Field(default_factory=lambda: Vector3(x=0, y=0, z=-9.81), description="Gravity vector")
    time_step: float = Field(default=0.001, description="Simulation time step in seconds")
    real_time_factor: float = Field(default=1.0, description="Real-time factor (1.0 = real-time)")
    collision_detection_enabled: bool = Field(default=True, description="Whether collision detection is enabled")
    contact_surface_params: Dict[str, float] = Field(default_factory=dict, description="Contact surface parameters")


class SimulationState(str, Enum):
    STOPPED = "stopped"
    PAUSED = "paused"
    RUNNING = "running"
    ERROR = "error"


class SimulationControl(BaseModel):
    command: str = Field(..., description="Control command (start, stop, pause, reset)")
    robot_model_id: Optional[str] = Field(default=None, description="ID of robot model to load")


class RobotState(BaseModel):
    joint_positions: Dict[str, float] = Field(default_factory=dict, description="Current joint positions")
    joint_velocities: Dict[str, float] = Field(default_factory=dict, description="Current joint velocities")
    joint_efforts: Dict[str, float] = Field(default_factory=dict, description="Current joint efforts")
    link_positions: Dict[str, Vector3] = Field(default_factory=dict, description="Current link positions")
    link_orientations: Dict[str, Quaternion] = Field(default_factory=dict, description="Current link orientations")
    timestamp: float = Field(..., description="Timestamp of the state")


class SimulationStatus(BaseModel):
    state: SimulationState = Field(..., description="Current simulation state")
    robot_model_id: Optional[str] = Field(default=None, description="ID of loaded robot model")
    physics_config: PhysicsSimulatorConfig = Field(default_factory=PhysicsSimulatorConfig, description="Current physics configuration")
    robot_state: Optional[RobotState] = Field(default=None, description="Current robot state")
    performance_metrics: Dict[str, float] = Field(default_factory=dict, description="Performance metrics (FPS, update rates, etc.)")


class LiDARData(BaseModel):
    header_timestamp: float = Field(..., description="Timestamp of the scan")
    angle_min: float = Field(..., description="Start angle of the scan [rad]")
    angle_max: float = Field(..., description="End angle of the scan [rad]")
    angle_increment: float = Field(..., description="Angular distance between measurements [rad]")
    time_increment: float = Field(..., description="Time between measurements [seconds]")
    scan_time: float = Field(..., description="Time between scans [seconds]")
    range_min: float = Field(..., description="Minimum range value [m]")
    range_max: float = Field(..., description="Maximum range value [m]")
    ranges: List[float] = Field(..., description="Range data [m]")
    intensities: Optional[List[float]] = Field(default=None, description="Intensity data")


class DepthImageData(BaseModel):
    width: int = Field(..., description="Image width in pixels")
    height: int = Field(..., description="Image height in pixels")
    timestamp: float = Field(..., description="Timestamp of the image")
    depth_data: List[List[float]] = Field(..., description="2D array of depth values")
    camera_info: Dict[str, Union[float, int]] = Field(default_factory=dict, description="Camera intrinsic parameters")


class IMUData(BaseModel):
    timestamp: float = Field(..., description="Timestamp of the measurement")
    orientation: Quaternion = Field(default_factory=Quaternion, description="Orientation estimate")
    angular_velocity: Vector3 = Field(default_factory=Vector3, description="Angular velocity")
    linear_acceleration: Vector3 = Field(default_factory=Vector3, description="Linear acceleration")
    orientation_covariance: List[float] = Field(default=[-1.0] + [0.0] * 8, description="Orientation covariance (9 elements)")
    angular_velocity_covariance: List[float] = Field(default=[-1.0] + [0.0] * 8, description="Angular velocity covariance (9 elements)")
    linear_acceleration_covariance: List[float] = Field(default=[-1.0] + [0.0] * 8, description="Linear acceleration covariance (9 elements)")


class SensorData(BaseModel):
    sensor_id: str = Field(..., description="ID of the sensor that generated the data")
    sensor_type: SensorType = Field(..., description="Type of sensor")
    timestamp: float = Field(..., description="Timestamp of the sensor data")
    data: Union[LiDARData, DepthImageData, IMUData] = Field(..., description="Actual sensor data")


class TrajectoryPoint(BaseModel):
    joint_positions: Dict[str, float] = Field(default_factory=dict, description="Joint positions at this point")
    joint_velocities: Dict[str, float] = Field(default_factory=dict, description="Joint velocities at this point")
    joint_accelerations: Optional[Dict[str, float]] = Field(default=None, description="Joint accelerations at this point")
    time_from_start: float = Field(..., description="Time from the start of the trajectory")


class TrajectoryData(BaseModel):
    id: str = Field(..., description="Unique identifier for the trajectory")
    robot_id: str = Field(..., description="ID of the robot being recorded")
    start_time: float = Field(..., description="Start time of the recording")
    end_time: float = Field(..., description="End time of the recording")
    trajectory_points: List[TrajectoryPoint] = Field(..., description="Time-series trajectory data")
    control_commands: List[Dict] = Field(default_factory=list, description="Time-series control commands")
    sensor_data: List[SensorData] = Field(default_factory=list, description="Time-series sensor data")