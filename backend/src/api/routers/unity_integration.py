"""
API Router for Unity Integration in Digital Twin Module

Handles REST API endpoints for Unity environment integration,
including robot state synchronization and visualization data.
"""

from fastapi import APIRouter, HTTPException, Depends
from typing import Dict, Any, Optional, List
import logging
import json
import time

from src.models.robot_model import (
    RobotState, RobotModel, SimulationStatus,
    Vector3, Quaternion, JointInfo
)
from src.services.gazebo_service import GazeboSimulationService
from src.services.simulation_coordination_service import get_coordination_service

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/unity", tags=["unity_integration"])

# Global services
gazebo_service: Optional[GazeboSimulationService] = None


def get_gazebo_service():
    """Get or create the Gazebo simulation service instance"""
    global gazebo_service
    if gazebo_service is None:
        gazebo_service = GazeboSimulationService()
    return gazebo_service


def get_coordination_status():
    """Get the coordination status from the coordination service"""
    coordination_svc = get_coordination_service()
    return coordination_svc.get_synchronization_status()


@router.post("/connect", response_model=bool)
async def connect_unity(
    connection_info: Dict[str, Any],
    service: GazeboSimulationService = Depends(get_gazebo_service)
):
    """Establish connection with Unity environment"""
    try:
        logger.info(f"Unity connection attempt from: {connection_info.get('client_id', 'unknown')}")
        # In a real implementation, this would register the Unity client
        # For now, we just acknowledge the connection
        return True
    except Exception as e:
        logger.error(f"Error connecting Unity: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/robot/state", response_model=Dict[str, Any])
async def get_robot_state_for_unity(
    service: GazeboSimulationService = Depends(get_gazebo_service)
):
    """Get robot state formatted for Unity consumption"""
    try:
        state_data = service.get_current_state()
        if state_data:
            # Format data for Unity (using Unity coordinate system)
            unity_formatted = {
                "joint_positions": state_data.get("joint_positions", {}),
                "joint_velocities": state_data.get("joint_velocities", {}),
                "joint_efforts": state_data.get("joint_efforts", {}),
                "timestamp": state_data.get("timestamp", 0.0),
                "simulation_state": state_data.get("simulation_state", "stopped"),
                # Convert to Unity coordinate system (if needed)
                "robot_pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},  # Placeholder
                    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}  # Placeholder
                }
            }
            return unity_formatted
        else:
            return {
                "joint_positions": {},
                "joint_velocities": {},
                "joint_efforts": {},
                "timestamp": 0.0,
                "simulation_state": "stopped",
                "robot_pose": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            }
    except Exception as e:
        logger.error(f"Error getting robot state for Unity: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/robot/state/update", response_model=bool)
async def update_robot_state_from_unity(
    robot_state: RobotState,
    service: GazeboSimulationService = Depends(get_gazebo_service)
):
    """Update robot state from Unity environment (for synchronized simulation)"""
    try:
        # In a real implementation, this would send the state to Gazebo
        # For now, we'll just log the update
        logger.info(f"Received robot state update from Unity: {len(robot_state.joint_positions)} joints")

        # Send commands to the simulated robot based on Unity state
        if robot_state.joint_positions:
            service.send_robot_command(robot_state.joint_positions)

        return True
    except Exception as e:
        logger.error(f"Error updating robot state from Unity: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/environment/objects", response_model=List[Dict[str, Any]])
async def get_environment_objects(
    service: GazeboSimulationService = Depends(get_gazebo_service)
):
    """Get environment objects for Unity visualization"""
    try:
        # In a real implementation, this would get objects from Gazebo
        # For now, return placeholder objects
        objects = [
            {
                "id": "ground_plane",
                "type": "plane",
                "position": {"x": 0.0, "y": -1.0, "z": 0.0},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                "scale": {"x": 10.0, "y": 1.0, "z": 10.0},
                "visual": {"color": [0.5, 0.5, 0.5, 1.0]}
            },
            {
                "id": "obstacle_1",
                "type": "box",
                "position": {"x": 2.0, "y": 0.0, "z": 0.0},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                "scale": {"x": 1.0, "y": 1.0, "z": 1.0},
                "visual": {"color": [1.0, 0.0, 0.0, 1.0]}
            }
        ]
        return objects
    except Exception as e:
        logger.error(f"Error getting environment objects: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/visualization/config", response_model=bool)
async def set_visualization_config(
    config: Dict[str, Any],
    service: GazeboSimulationService = Depends(get_gazebo_service)
):
    """Set visualization configuration for Unity environment"""
    try:
        logger.info(f"Setting visualization config: {config}")
        # In a real implementation, this would configure how data is visualized
        # For now, just acknowledge the configuration
        return True
    except Exception as e:
        logger.error(f"Error setting visualization config: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/synchronization/status", response_model=Dict[str, Any])
async def get_synchronization_status(
    service: GazeboSimulationService = Depends(get_gazebo_service)
):
    """Get synchronization status between Gazebo and Unity"""
    try:
        # Get simulation status
        state_data = service.get_current_state()

        sync_status = {
            "gazebo_connected": True,  # Placeholder
            "unity_connected": True,   # Would be determined by actual connection state
            "last_sync_time": time.time(),
            "sync_frequency": 60,  # Hz - Unity typical frame rate
            "data_lag": 0.0,  # Seconds
            "simulation_state": state_data.get("simulation_state", "stopped") if state_data else "stopped"
        }
        return sync_status
    except Exception as e:
        logger.error(f"Error getting synchronization status: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/scene/reset", response_model=bool)
async def reset_unity_scene(
    service: GazeboSimulationService = Depends(get_gazebo_service)
):
    """Reset the Unity scene to match Gazebo simulation state"""
    try:
        logger.info("Resetting Unity scene to match simulation")
        # In a real implementation, this would reset both Unity and Gazebo to initial state
        return True
    except Exception as e:
        logger.error(f"Error resetting Unity scene: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/capabilities", response_model=Dict[str, Any])
async def get_unity_capabilities():
    """Get information about Unity integration capabilities"""
    try:
        capabilities = {
            "supported_features": [
                "robot_visualization",
                "environment_rendering",
                "human_avatar_interaction",
                "trajectory_playback",
                "sensor_visualization"
            ],
            "coordinate_system": "left_handed",  # Unity uses left-handed coordinate system
            "rendering_quality": "high",
            "max_frame_rate": 60,
            "supported_robot_formats": ["urdf_converted", "fbx", "obj", "gltf"],
            "integration_protocol": "ros_tcp_connector"
        }
        return capabilities
    except Exception as e:
        logger.error(f"Error getting Unity capabilities: {e}")
        raise HTTPException(status_code=500, detail=str(e))