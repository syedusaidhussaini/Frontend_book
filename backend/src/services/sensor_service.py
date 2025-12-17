"""
Sensor Simulation Service for Digital Twin Module

Handles simulation of various robot sensors including LiDAR, depth cameras,
and IMUs in both Gazebo and Unity environments.
"""

import logging
import random
import time
from typing import Dict, Any, Optional, List
from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Image, Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
import numpy as np

from src.models.robot_model import (
    SensorType, SensorParameters, SensorData,
    LiDARData, DepthImageData, IMUData
)

logger = logging.getLogger(__name__)


class SensorSimulatorNode(Node):
    """
    ROS 2 node that simulates various robot sensors
    """

    def __init__(self, node_name: str = "sensor_simulator_node"):
        super().__init__(node_name)

        # Publishers for different sensor types
        self.lidar_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)

        # Timer for sensor data generation
        self.sensor_timer = self.create_timer(0.1, self.generate_sensor_data)  # 10 Hz

        # Internal state
        self.robot_position = [0.0, 0.0, 0.0]  # x, y, z
        self.robot_orientation = [0.0, 0.0, 0.0, 1.0]  # x, y, z, w (quaternion)
        self.robot_velocity = [0.0, 0.0, 0.0]  # linear velocity
        self.robot_angular_velocity = [0.0, 0.0, 0.0]  # angular velocity

        logger.info("Sensor Simulator Node initialized")

    def generate_sensor_data(self):
        """Generate and publish sensor data at regular intervals"""
        # Generate LiDAR data
        self._generate_lidar_data()

        # Generate IMU data
        self._generate_imu_data()

    def _generate_lidar_data(self):
        """Generate simulated LiDAR scan data"""
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        # Set LiDAR parameters
        msg.angle_min = -np.pi / 2  # -90 degrees
        msg.angle_max = np.pi / 2   # 90 degrees
        msg.angle_increment = np.pi / 180  # 1 degree
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0

        # Generate ranges with some obstacles
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        ranges = []

        for i in range(num_readings):
            angle = msg.angle_min + i * msg.angle_increment

            # Simulate some obstacles at various distances
            distance = msg.range_max  # Default: no obstacle

            # Add some random obstacles
            if random.random() < 0.3:  # 30% chance of obstacle
                distance = random.uniform(msg.range_min, msg.range_max * 0.7)

            # Add some noise to make it more realistic
            noise = random.gauss(0, 0.02)  # 2cm standard deviation
            noisy_distance = max(msg.range_min, min(msg.range_max, distance + noise))
            ranges.append(noisy_distance)

        msg.ranges = ranges
        msg.intensities = [100.0] * len(ranges)  # Constant intensity

        self.lidar_publisher.publish(msg)

    def _generate_imu_data(self):
        """Generate simulated IMU data"""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_frame'

        # Set orientation (with some small random variations)
        msg.orientation.x = self.robot_orientation[0] + random.gauss(0, 0.01)
        msg.orientation.y = self.robot_orientation[1] + random.gauss(0, 0.01)
        msg.orientation.z = self.robot_orientation[2] + random.gauss(0, 0.01)
        msg.orientation.w = self.robot_orientation[3] + random.gauss(0, 0.01)

        # Set angular velocity (with noise)
        msg.angular_velocity.x = self.robot_angular_velocity[0] + random.gauss(0, 0.01)
        msg.angular_velocity.y = self.robot_angular_velocity[1] + random.gauss(0, 0.01)
        msg.angular_velocity.z = self.robot_angular_velocity[2] + random.gauss(0, 0.01)

        # Set linear acceleration (include gravity)
        # Gravity in base frame (assuming z is up)
        gravity = [0.0, 0.0, 9.81]
        msg.linear_acceleration.x = self.robot_velocity[0] + gravity[0] + random.gauss(0, 0.1)
        msg.linear_acceleration.y = self.robot_velocity[1] + gravity[1] + random.gauss(0, 0.1)
        msg.linear_acceleration.z = self.robot_velocity[2] + gravity[2] + random.gauss(0, 0.1)

        # Set covariance matrices (diagonal values only, -1 means "covariance unknown")
        identity_cov = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
        msg.orientation_covariance = identity_cov
        msg.angular_velocity_covariance = identity_cov
        msg.linear_acceleration_covariance = identity_cov

        self.imu_publisher.publish(msg)

    def update_robot_state(self, position: List[float], orientation: List[float],
                          velocity: List[float], angular_velocity: List[float]):
        """Update the robot state used for sensor simulation"""
        self.robot_position = position
        self.robot_orientation = orientation
        self.robot_velocity = velocity
        self.robot_angular_velocity = angular_velocity


class SensorSimulationService:
    """
    Service class that manages sensor simulation operations
    """

    def __init__(self):
        self.node: Optional[SensorSimulatorNode] = None
        self._initialized = False
        logger.info("Initializing Sensor Simulation Service")

    def initialize_ros(self):
        """Initialize ROS 2 context"""
        if not rclpy.ok():
            rclpy.init()

        self.node = SensorSimulatorNode()
        self._initialized = True
        logger.info("ROS 2 context initialized for sensor simulation")

    def start_sensor_simulation(self) -> bool:
        """Start sensor simulation"""
        if not self._initialized:
            self.initialize_ros()

        logger.info("Sensor simulation started")
        return True

    def stop_sensor_simulation(self):
        """Stop sensor simulation"""
        if self.node:
            logger.info("Sensor simulation stopped")

    def update_robot_for_sensors(self, position: List[float], orientation: List[float],
                                velocity: List[float], angular_velocity: List[float]):
        """Update robot state that affects sensor readings"""
        if self.node:
            self.node.update_robot_state(position, orientation, velocity, angular_velocity)

    def generate_lidar_scan(self, sensor_params: SensorParameters) -> LiDARData:
        """Generate a single LiDAR scan with specified parameters"""
        timestamp = time.time()

        # Use parameters from sensor config
        angle_min = sensor_params.scan_field_of_view / -2.0
        angle_max = sensor_params.scan_field_of_view / 2.0
        angle_increment = sensor_params.scan_resolution
        range_min = sensor_params.scan_range_min or 0.1
        range_max = sensor_params.scan_range_max or 10.0

        num_readings = int((angle_max - angle_min) / angle_increment) + 1
        ranges = []

        for i in range(num_readings):
            angle = angle_min + i * angle_increment

            # Simulate environment with obstacles
            distance = range_max  # Default: no obstacle

            # Add some obstacles based on simulated environment
            if random.random() < 0.25:  # 25% chance of obstacle
                distance = random.uniform(range_min, range_max * 0.8)

            # Add sensor noise
            noise = random.gauss(0, 0.02)  # 2cm standard deviation
            noisy_distance = max(range_min, min(range_max, distance + noise))
            ranges.append(noisy_distance)

        return LiDARData(
            header_timestamp=timestamp,
            angle_min=angle_min,
            angle_max=angle_max,
            angle_increment=angle_increment,
            time_increment=0.0,
            scan_time=0.1,
            range_min=range_min,
            range_max=range_max,
            ranges=ranges
        )

    def generate_imu_data(self, sensor_params: SensorParameters) -> IMUData:
        """Generate IMU data with specified parameters"""
        timestamp = time.time()

        # Simulate realistic IMU readings
        # Orientation (with small random variations)
        orientation = Quaternion(
            x=random.gauss(0, sensor_params.imu_noise_density or 0.01),
            y=random.gauss(0, sensor_params.imu_noise_density or 0.01),
            z=random.gauss(0, sensor_params.imu_noise_density or 0.01),
            w=1.0  # Default to no rotation
        )

        # Angular velocity (with noise)
        angular_velocity = Vector3(
            x=random.gauss(0, sensor_params.imu_noise_density or 0.01),
            y=random.gauss(0, sensor_params.imu_noise_density or 0.01),
            z=random.gauss(0, sensor_params.imu_noise_density or 0.01)
        )

        # Linear acceleration (with gravity and noise)
        linear_acceleration = Vector3(
            x=random.gauss(0, sensor_params.imu_noise_density or 0.1),
            y=random.gauss(0, sensor_params.imu_noise_density or 0.1),
            z=9.81 + random.gauss(0, sensor_params.imu_noise_density or 0.1)  # Gravity
        )

        return IMUData(
            timestamp=timestamp,
            orientation=orientation,
            angular_velocity=angular_velocity,
            linear_acceleration=linear_acceleration
        )

    def shutdown(self):
        """Shutdown the sensor simulation service"""
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self._initialized = False
        logger.info("Sensor Simulation Service shutdown")


# Depth camera simulation would typically require more complex 3D rendering
# For now, we'll create a basic implementation that generates simulated data
class DepthCameraSimulator:
    """
    Simulates depth camera data generation
    """

    @staticmethod
    def generate_depth_image(width: int = 640, height: int = 480,
                           fov: float = np.pi/2,
                           min_depth: float = 0.1,
                           max_depth: float = 10.0) -> DepthImageData:
        """
        Generate a simulated depth image
        """
        timestamp = time.time()

        # Create a simple depth map with some objects
        depth_data = []
        for y in range(height):
            row = []
            for x in range(width):
                # Simulate a scene with some objects at different depths
                # This is a simplified simulation - in reality, this would use 3D rendering
                normalized_x = (x - width/2) / (width/2)
                normalized_y = (y - height/2) / (height/2)

                # Create a simple scene with a ground plane and some objects
                if abs(normalized_y) > 0.8:  # Ground
                    depth = max_depth * 0.7
                elif abs(normalized_x) < 0.2 and abs(normalized_y) < 0.2:  # Center object
                    depth = min_depth + (max_depth - min_depth) * 0.3
                else:  # Background
                    depth = max_depth

                # Add some noise
                noise = random.gauss(0, 0.02)
                noisy_depth = max(min_depth, min(max_depth, depth + noise))
                row.append(noisy_depth)
            depth_data.append(row)

        camera_info = {
            "width": width,
            "height": height,
            "fov": fov,
            "min_depth": min_depth,
            "max_depth": max_depth,
            "fx": width / (2 * np.tan(fov/2)),  # Focal length x
            "fy": height / (2 * np.tan(fov/2)), # Focal length y
            "cx": width / 2,  # Principal point x
            "cy": height / 2  # Principal point y
        }

        return DepthImageData(
            width=width,
            height=height,
            timestamp=timestamp,
            depth_data=depth_data,
            camera_info=camera_info
        )