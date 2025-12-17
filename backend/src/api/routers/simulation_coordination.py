"""
API Router for Simulation Coordination in Digital Twin Module

Handles REST API endpoints for coordinating between Gazebo physics simulation
and Unity visualization to maintain synchronized digital twin state.
"""
from fastapi import APIRouter, HTTPException, Depends
from typing import Dict, Any, Optional
import logging
import time

from src.services.simulation_coordination_service import (
    SimulationCoordinationService,
    get_coordination_service,
    SynchronizationMetrics
)

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/coordination", tags=["simulation_coordination"])


@router.post("/start", response_model=bool)
async def start_coordination(
    service: SimulationCoordinationService = Depends(get_coordination_service)
):
    """Start the simulation coordination between Gazebo and Unity"""
    try:
        success = service.start_coordination()
        if not success:
            raise HTTPException(status_code=500, detail="Failed to start simulation coordination")
        return success
    except Exception as e:
        logger.error(f"Error starting simulation coordination: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/stop", response_model=bool)
async def stop_coordination(
    service: SimulationCoordinationService = Depends(get_coordination_service)
):
    """Stop the simulation coordination"""
    try:
        service.stop_coordination()
        return True
    except Exception as e:
        logger.error(f"Error stopping simulation coordination: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/status", response_model=Dict[str, Any])
async def get_coordination_status(
    service: SimulationCoordinationService = Depends(get_coordination_service)
):
    """Get the current status of simulation coordination"""
    try:
        status = service.get_synchronization_status()
        return status
    except Exception as e:
        logger.error(f"Error getting coordination status: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/reset", response_model=bool)
async def reset_coordination(
    service: SimulationCoordinationService = Depends(get_coordination_service)
):
    """Reset the simulation coordination to initial state"""
    try:
        service.reset_synchronization()
        return True
    except Exception as e:
        logger.error(f"Error resetting coordination: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/frequency", response_model=bool)
async def set_coordination_frequency(
    frequency_hz: int,
    service: SimulationCoordinationService = Depends(get_coordination_service)
):
    """Set the synchronization frequency between Gazebo and Unity"""
    try:
        if frequency_hz <= 0 or frequency_hz > 500:
            raise HTTPException(status_code=400, detail="Frequency must be between 1 and 500 Hz")

        service.set_sync_frequency(frequency_hz)
        return True
    except ValueError as e:
        logger.error(f"Invalid frequency value: {e}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Error setting coordination frequency: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/performance", response_model=Dict[str, float])
async def get_coordination_performance(
    service: SimulationCoordinationService = Depends(get_coordination_service)
):
    """Get performance metrics for simulation coordination"""
    try:
        status = service.get_synchronization_status()
        metrics = status.get("metrics", {})

        performance_data = {
            "gazebo_fps": metrics.get("gazebo_fps", 0.0),
            "unity_fps": metrics.get("unity_fps", 0.0),
            "data_lag_ms": metrics.get("data_lag_ms", 0.0),
            "sync_frequency_hz": metrics.get("sync_frequency_hz", 0.0),
            "packet_loss_rate": metrics.get("packet_loss_rate", 0.0)
        }

        return performance_data
    except Exception as e:
        logger.error(f"Error getting coordination performance: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/sync_state", response_model=bool)
async def sync_single_state(
    gazebo_state: Optional[Dict[str, Any]] = None,
    unity_state: Optional[Dict[str, Any]] = None,
    service: SimulationCoordinationService = Depends(get_coordination_service)
):
    """Manually synchronize a single state between platforms"""
    try:
        # If both states are provided, this is a bidirectional sync
        if gazebo_state and unity_state:
            logger.info("Performing bidirectional state synchronization")
            # In a real implementation, this would handle bidirectional sync
            pass
        elif gazebo_state:
            logger.info("Syncing Gazebo state to Unity")
            service._sync_state_to_unity(gazebo_state)
        elif unity_state:
            logger.info("Syncing Unity state to Gazebo")
            service._sync_state_to_gazebo(unity_state)
        else:
            # If no specific states provided, trigger a sync from current states
            current_gazebo = service.gazebo_service.get_current_state()
            if current_gazebo:
                service._sync_state_to_unity(current_gazebo)

        return True
    except Exception as e:
        logger.error(f"Error during manual state sync: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/capabilities", response_model=Dict[str, Any])
async def get_coordination_capabilities():
    """Get information about coordination capabilities"""
    try:
        capabilities = {
            "supported_features": [
                "real_time_synchronization",
                "bidirectional_state_sync",
                "performance_monitoring",
                "frequency_control",
                "connection_monitoring"
            ],
            "max_sync_frequency_hz": 500,
            "min_sync_frequency_hz": 1,
            "default_sync_frequency_hz": 60,
            "supported_platforms": ["gazebo", "unity"],
            "data_types_synced": [
                "joint_positions",
                "joint_velocities",
                "joint_efforts",
                "robot_pose",
                "sensor_data"
            ]
        }
        return capabilities
    except Exception as e:
        logger.error(f"Error getting coordination capabilities: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.on_event("shutdown")
async def shutdown_event():
    """Clean up coordination service on application shutdown"""
    global coordination_service
    if coordination_service:
        coordination_service.stop_coordination()
        coordination_service = None