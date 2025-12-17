"""
API Router for Digital Twin Simulation

Handles REST API endpoints for Gazebo physics simulation,
robot control, and simulation management.
"""

from fastapi import APIRouter, HTTPException, Depends
from typing import Dict, Any, Optional
import logging

from src.models.robot_model import (
    RobotModel, SimulationControl, SimulationStatus,
    RobotState, PhysicsSimulatorConfig, TrajectoryData
)
from src.services.gazebo_service import GazeboSimulationService

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/simulation", tags=["simulation"])

# Global simulation service instance
# In production, this would be managed differently (dependency injection, etc.)
simulation_service: Optional[GazeboSimulationService] = None


def get_simulation_service():
    """Get or create the simulation service instance"""
    global simulation_service
    if simulation_service is None:
        simulation_service = GazeboSimulationService()
    return simulation_service


@router.post("/start", response_model=bool)
async def start_simulation(
    control: SimulationControl,
    service: GazeboSimulationService = Depends(get_simulation_service)
):
    """Start the Gazebo simulation with specified robot model"""
    try:
        success = service.start_simulation(control.robot_model_id or "")
        if not success:
            raise HTTPException(status_code=500, detail="Failed to start simulation")
        return success
    except Exception as e:
        logger.error(f"Error starting simulation: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/stop", response_model=bool)
async def stop_simulation(
    service: GazeboSimulationService = Depends(get_simulation_service)
):
    """Stop the Gazebo simulation"""
    try:
        service.stop_simulation()
        return True
    except Exception as e:
        logger.error(f"Error stopping simulation: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/control", response_model=bool)
async def send_simulation_control(
    control: SimulationControl,
    service: GazeboSimulationService = Depends(get_simulation_service)
):
    """Send control commands to the simulation (start, stop, pause, reset)"""
    try:
        if control.command == "start":
            if control.robot_model_id:
                success = service.start_simulation(control.robot_model_id)
                if not success:
                    raise HTTPException(status_code=500, detail="Failed to start simulation")
                return success
            else:
                raise HTTPException(status_code=400, detail="robot_model_id required for start command")
        elif control.command == "stop":
            service.stop_simulation()
            return True
        elif control.command == "pause":
            # Add pause functionality to service
            return True
        elif control.command == "reset":
            # Add reset functionality to service
            return True
        else:
            raise HTTPException(status_code=400, detail=f"Unknown command: {control.command}")
    except Exception as e:
        logger.error(f"Error sending simulation control: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/status", response_model=SimulationStatus)
async def get_simulation_status(
    service: GazeboSimulationService = Depends(get_simulation_service)
):
    """Get the current status of the simulation"""
    try:
        # For now, return a mock status
        # In a real implementation, this would get actual data from the service
        state_data = service.get_current_state()
        if state_data:
            return SimulationStatus(
                state=state_data.get("simulation_state", "stopped"),
                robot_model_id=state_data.get("robot_model_id"),
                robot_state=RobotState(
                    joint_positions=state_data.get("joint_positions", {}),
                    joint_velocities=state_data.get("joint_velocities", {}),
                    joint_efforts=state_data.get("joint_efforts", {}),
                    timestamp=state_data.get("timestamp", 0.0)
                ),
                performance_metrics=state_data.get("performance_metrics", {})
            )
        else:
            return SimulationStatus(
                state="stopped",
                performance_metrics={}
            )
    except Exception as e:
        logger.error(f"Error getting simulation status: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/robot/command", response_model=bool)
async def send_robot_command(
    joint_positions: Dict[str, float],
    service: GazeboSimulationService = Depends(get_simulation_service)
):
    """Send joint position commands to the simulated robot"""
    try:
        service.send_robot_command(joint_positions)
        return True
    except Exception as e:
        logger.error(f"Error sending robot command: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/physics/config", response_model=bool)
async def set_physics_config(
    config: PhysicsSimulatorConfig,
    service: GazeboSimulationService = Depends(get_simulation_service)
):
    """Set physics simulation parameters"""
    try:
        gravity_dict = {
            "x": config.gravity.x,
            "y": config.gravity.y,
            "z": config.gravity.z
        }
        service.set_physics_params(
            gravity=gravity_dict,
            time_step=config.time_step,
            real_time_factor=config.real_time_factor
        )
        return True
    except Exception as e:
        logger.error(f"Error setting physics config: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/robot/state", response_model=RobotState)
async def get_robot_state(
    service: GazeboSimulationService = Depends(get_simulation_service)
):
    """Get the current state of the robot"""
    try:
        state_data = service.get_current_state()
        if state_data:
            return RobotState(
                joint_positions=state_data.get("joint_positions", {}),
                joint_velocities=state_data.get("joint_velocities", {}),
                joint_efforts=state_data.get("joint_efforts", {}),
                timestamp=state_data.get("timestamp", 0.0)
            )
        else:
            return RobotState(
                joint_positions={},
                joint_velocities={},
                joint_efforts={},
                timestamp=0.0
            )
    except Exception as e:
        logger.error(f"Error getting robot state: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/trajectory/record", response_model=str)
async def start_trajectory_recording(
    service: GazeboSimulationService = Depends(get_simulation_service)
):
    """Start recording robot trajectory"""
    try:
        # In a real implementation, this would start recording
        # For now, return a mock trajectory ID
        return "trajectory_001"
    except Exception as e:
        logger.error(f"Error starting trajectory recording: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/trajectory/{trajectory_id}/stop", response_model=TrajectoryData)
async def stop_trajectory_recording(
    trajectory_id: str,
    service: GazeboSimulationService = Depends(get_simulation_service)
):
    """Stop recording and return trajectory data"""
    try:
        # In a real implementation, this would stop recording and return actual data
        # For now, return mock trajectory data
        return TrajectoryData(
            id=trajectory_id,
            robot_id="mock_robot",
            start_time=0.0,
            end_time=1.0,
            trajectory_points=[]
        )
    except Exception as e:
        logger.error(f"Error stopping trajectory recording: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.on_event("shutdown")
async def shutdown_event():
    """Clean up simulation service on application shutdown"""
    global simulation_service
    if simulation_service:
        simulation_service.shutdown()
        simulation_service = None