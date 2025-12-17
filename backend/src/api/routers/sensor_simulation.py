"""
API Router for Sensor Simulation in Digital Twin Module

Handles REST API endpoints for simulating various robot sensors
including LiDAR, depth cameras, and IMUs.
"""

from fastapi import APIRouter, HTTPException, Depends
from typing import Dict, Any, Optional
import logging
import random
import time

from src.models.robot_model import (
    SensorType, SensorParameters, SensorData,
    LiDARData, DepthImageData, IMUData, RobotState
)
from src.services.sensor_service import SensorSimulationService, DepthCameraSimulator

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/sensor", tags=["sensor_simulation"])

# Global sensor service instance
sensor_service: Optional[SensorSimulationService] = None


def get_sensor_service():
    """Get or create the sensor simulation service instance"""
    global sensor_service
    if sensor_service is None:
        sensor_service = SensorSimulationService()
    return sensor_service


@router.post("/start", response_model=bool)
async def start_sensor_simulation(
    service: SensorSimulationService = Depends(get_sensor_service)
):
    """Start the sensor simulation system"""
    try:
        success = service.start_sensor_simulation()
        return success
    except Exception as e:
        logger.error(f"Error starting sensor simulation: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/stop", response_model=bool)
async def stop_sensor_simulation(
    service: SensorSimulationService = Depends(get_sensor_service)
):
    """Stop the sensor simulation system"""
    try:
        service.stop_sensor_simulation()
        return True
    except Exception as e:
        logger.error(f"Error stopping sensor simulation: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/lidar/generate", response_model=LiDARData)
async def generate_lidar_scan(
    sensor_params: Optional[SensorParameters] = None,
    service: SensorSimulationService = Depends(get_sensor_service)
):
    """Generate a LiDAR scan with specified parameters"""
    try:
        if sensor_params is None:
            sensor_params = SensorParameters()

        lidar_data = service.generate_lidar_scan(sensor_params)
        return lidar_data
    except Exception as e:
        logger.error(f"Error generating LiDAR scan: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/imu/generate", response_model=IMUData)
async def generate_imu_data(
    sensor_params: Optional[SensorParameters] = None,
    service: SensorSimulationService = Depends(get_sensor_service)
):
    """Generate IMU data with specified parameters"""
    try:
        if sensor_params is None:
            sensor_params = SensorParameters()

        imu_data = service.generate_imu_data(sensor_params)
        return imu_data
    except Exception as e:
        logger.error(f"Error generating IMU data: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/depth_camera/generate", response_model=DepthImageData)
async def generate_depth_image(
    width: int = 640,
    height: int = 480,
    fov: float = 1.047,  # Default ~60 degrees in radians
    min_depth: float = 0.1,
    max_depth: float = 10.0
):
    """Generate a depth camera image"""
    try:
        depth_image = DepthCameraSimulator.generate_depth_image(
            width=width,
            height=height,
            fov=fov,
            min_depth=min_depth,
            max_depth=max_depth
        )
        return depth_image
    except Exception as e:
        logger.error(f"Error generating depth image: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/data_stream/{sensor_type}", response_model=SensorData)
async def get_sensor_data_stream(
    sensor_type: SensorType,
    service: SensorSimulationService = Depends(get_sensor_service)
):
    """Get a sample of sensor data from the specified sensor type"""
    try:
        # Generate appropriate sensor data based on type
        timestamp = time.time()

        if sensor_type == SensorType.LIDAR:
            lidar_data = service.generate_lidar_scan(SensorParameters())
            sensor_data = SensorData(
                sensor_id=f"lidar_{int(timestamp)}",
                sensor_type=sensor_type,
                timestamp=timestamp,
                data=lidar_data
            )
        elif sensor_type == SensorType.IMU:
            imu_data = service.generate_imu_data(SensorParameters())
            sensor_data = SensorData(
                sensor_id=f"imu_{int(timestamp)}",
                sensor_type=sensor_type,
                timestamp=timestamp,
                data=imu_data
            )
        elif sensor_type == SensorType.DEPTH_CAMERA:
            depth_data = DepthCameraSimulator.generate_depth_image()
            sensor_data = SensorData(
                sensor_id=f"depth_camera_{int(timestamp)}",
                sensor_type=sensor_type,
                timestamp=timestamp,
                data=depth_data
            )
        else:
            raise HTTPException(status_code=400, detail=f"Sensor type {sensor_type} not supported")

        return sensor_data
    except Exception as e:
        logger.error(f"Error getting sensor data stream: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/robot_state/update", response_model=bool)
async def update_robot_state_for_sensors(
    robot_state: RobotState,
    service: SensorSimulationService = Depends(get_sensor_service)
):
    """Update robot state that affects sensor readings"""
    try:
        # Convert robot state to the format expected by the sensor service
        # For now, we'll use the robot position and some defaults
        position = [0.0, 0.0, 0.0]  # Default position
        orientation = [0.0, 0.0, 0.0, 1.0]  # Default orientation (quaternion)
        velocity = [0.0, 0.0, 0.0]  # Default velocity
        angular_velocity = [0.0, 0.0, 0.0]  # Default angular velocity

        # In a real implementation, we would derive these from the robot state
        # For now, we'll just call the update method with defaults
        service.update_robot_for_sensors(position, orientation, velocity, angular_velocity)
        return True
    except Exception as e:
        logger.error(f"Error updating robot state for sensors: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/capabilities", response_model=Dict[str, Any])
async def get_sensor_capabilities():
    """Get information about available sensor simulation capabilities"""
    try:
        capabilities = {
            "supported_sensors": [sensor_type.value for sensor_type in SensorType],
            "lidar_simulation": {
                "parameterizable": True,
                "features": ["range", "field_of_view", "resolution", "noise_model"]
            },
            "imu_simulation": {
                "parameterizable": True,
                "features": ["noise_density", "random_walk", "bias"]
            },
            "depth_camera_simulation": {
                "parameterizable": True,
                "features": ["resolution", "field_of_view", "depth_range"]
            },
            "real_time_simulation": False,  # Would be True when connected to actual sim
            "data_formats": ["sensor_msgs/LaserScan", "sensor_msgs/Imu", "sensor_msgs/Image"]
        }
        return capabilities
    except Exception as e:
        logger.error(f"Error getting sensor capabilities: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.on_event("shutdown")
async def shutdown_event():
    """Clean up sensor service on application shutdown"""
    global sensor_service
    if sensor_service:
        sensor_service.shutdown()
        sensor_service = None