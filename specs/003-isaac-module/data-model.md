# Data Model: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)

**Feature**: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)
**Spec**: [spec.md](spec.md)
**Date**: 2025-12-17

## Overview

This document defines the data models required for implementing Module 3 focusing on the NVIDIA Isaac ecosystem. The models cover Isaac Sim synthetic data generation, Isaac ROS perception pipelines, and Nav2 humanoid navigation systems.

## Core Entities

### IsaacSimScene
Represents a photorealistic simulation environment in Isaac Sim

```json
{
  "id": "string",
  "name": "string",
  "description": "string",
  "created_at": "datetime",
  "updated_at": "datetime",
  "environment_config": {
    "lighting": {
      "type": "string",
      "intensity": "float",
      "color": "string",
      "position": {"x": "float", "y": "float", "z": "float"}
    },
    "materials": [
      {
        "name": "string",
        "albedo": "string",
        "roughness": "float",
        "metallic": "float",
        "normal_map": "string"
      }
    ],
    "physics_properties": {
      "gravity": {"x": "float", "y": "float", "z": "float"},
      "friction": "float",
      "bounce": "float"
    }
  },
  "objects": [
    {
      "name": "string",
      "type": "string",
      "position": {"x": "float", "y": "float", "z": "float"},
      "rotation": {"x": "float", "y": "float", "z": "float", "w": "float"},
      "scale": {"x": "float", "y": "float", "z": "float"}
    }
  ],
  "sensors_config": {
    "cameras": [
      {
        "name": "string",
        "resolution": {"width": "int", "height": "int"},
        "fov": "float",
        "position": {"x": "float", "y": "float", "z": "float"}
      }
    ],
    "lidars": [
      {
        "name": "string",
        "range_min": "float",
        "range_max": "float",
        "fov_horizontal": "float",
        "fov_vertical": "float",
        "rotation_frequency": "float"
      }
    ],
    "imus": [
      {
        "name": "string",
        "position": {"x": "float", "y": "float", "z": "float"},
        "noise_density": "float",
        "random_walk": "float"
      }
    ]
  }
}
```

### SyntheticDataSample
Represents a generated synthetic data sample with sensor data and annotations

```json
{
  "id": "string",
  "scene_id": "string",
  "timestamp": "datetime",
  "sensor_data": {
    "camera": {
      "rgb_image_path": "string",
      "depth_image_path": "string",
      "camera_info": {
        "width": "int",
        "height": "int",
        "fx": "float",
        "fy": "float",
        "cx": "float",
        "cy": "float"
      }
    },
    "lidar": {
      "point_cloud_path": "string",
      "ranges": ["float"],
      "intensities": ["float"],
      "angle_min": "float",
      "angle_max": "float",
      "angle_increment": "float"
    },
    "imu": {
      "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"},
      "angular_velocity": {"x": "float", "y": "float", "z": "float"},
      "linear_acceleration": {"x": "float", "y": "float", "z": "float"}
    }
  },
  "annotations": {
    "object_detections": [
      {
        "class": "string",
        "confidence": "float",
        "bbox": {"x": "int", "y": "int", "width": "int", "height": "int"},
        "mask_path": "string"
      }
    ],
    "segmentation": {
      "semantic_map_path": "string",
      "instance_map_path": "string"
    },
    "ground_truth": {
      "object_poses": [
        {
          "object_id": "string",
          "position": {"x": "float", "y": "float", "z": "float"},
          "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
        }
      ]
    }
  },
  "domain_randomization_params": {
    "lighting_variation": "float",
    "material_variation": "float",
    "weather_conditions": "string",
    "occlusion_probability": "float"
  }
}
```

### PerceptionPipeline
Represents a hardware-accelerated perception pipeline using Isaac ROS

```json
{
  "id": "string",
  "name": "string",
  "description": "string",
  "created_at": "datetime",
  "updated_at": "datetime",
  "configuration": {
    "input_topics": [
      {
        "topic_name": "string",
        "sensor_type": "string",
        "message_type": "string"
      }
    ],
    "processing_nodes": [
      {
        "node_name": "string",
        "algorithm": "string",
        "parameters": {
          "key": "value"
        },
        "gpu_acceleration": "boolean"
      }
    ],
    "output_topics": [
      {
        "topic_name": "string",
        "message_type": "string",
        "description": "string"
      }
    ]
  },
  "performance_metrics": {
    "fps": "float",
    "processing_time_ms": "float",
    "gpu_utilization": "float",
    "memory_usage_mb": "float"
  },
  "calibration": {
    "camera_intrinsics": {
      "fx": "float",
      "fy": "float",
      "cx": "float",
      "cy": "float",
      "distortion_coeffs": ["float"]
    },
    "extrinsics": {
      "translation": {"x": "float", "y": "float", "z": "float"},
      "rotation": {"x": "float", "y": "float", "z": "float", "w": "float"}
    }
  }
}
```

### NavigationGoal
Represents a navigation goal for humanoid robot navigation using Nav2

```json
{
  "id": "string",
  "robot_id": "string",
  "goal_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "tolerance": {
    "xy_goal_tolerance": "float",
    "yaw_goal_tolerance": "float"
  },
  "constraints": {
    "max_linear_speed": "float",
    "max_angular_speed": "float",
    "balance_margins": {
      "min_com_height": "float",
      "max_com_deviation": "float"
    }
  },
  "recovery_behaviors": [
    {
      "behavior_name": "string",
      "parameters": {
        "key": "value"
      }
    }
  ],
  "navigation_result": {
    "status": "string",
    "completion_time": "float",
    "path_length": "float",
    "balance_metrics": {
      "balance_maintained_percentage": "float",
      "max_com_deviation": "float"
    }
  }
}
```

### BipedalPath
Represents a navigation path specifically designed for bipedal locomotion

```json
{
  "id": "string",
  "navigation_goal_id": "string",
  "waypoints": [
    {
      "position": {"x": "float", "y": "float", "z": "float"},
      "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"},
      "step_type": "string",
      "support_foot": "string",
      "time_from_start": "float"
    }
  ],
  "terrain_analysis": {
    "traversability": "float",
    "step_locations": [
      {
        "position": {"x": "float", "y": "float", "z": "float"},
        "foot_placement": "string",
        "validity": "boolean"
      }
    ],
    "obstacle_clearance": "float"
  },
  "balance_constraints": {
    "zmp_trajectory": [
      {"x": "float", "y": "float", "time": "float"}
    ],
    "com_trajectory": [
      {"x": "float", "y": "float", "z": "float", "time": "float"}
    ]
  },
  "execution_plan": {
    "step_timing": "float",
    "swing_height": "float",
    "stance_width": "float"
  }
}
```

## API Contract Models

### IsaacSimControlRequest
Request model for controlling Isaac Sim environment

```json
{
  "command": "string",
  "scene_id": "string",
  "parameters": {
    "key": "value"
  }
}
```

### PerceptionPipelineRequest
Request model for perception pipeline operations

```json
{
  "pipeline_id": "string",
  "operation": "string",
  "configuration": {
    "key": "value"
  }
}
```

### NavigationRequest
Request model for humanoid navigation

```json
{
  "robot_id": "string",
  "goal_pose": {
    "position": {"x": "float", "y": "float", "z": "float"},
    "orientation": {"x": "float", "y": "float", "z": "float", "w": "float"}
  },
  "navigation_parameters": {
    "key": "value"
  }
}
```

## Validation Rules

### IsaacSimScene Validation
- `name` must be 3-50 characters
- `environment_config.lighting.position` values must be within simulation bounds
- `sensors_config` must include at least one sensor
- `materials` must reference valid material assets

### SyntheticDataSample Validation
- `timestamp` must be within valid range
- `sensor_data` must include at least one sensor modality
- `annotations` must be consistent with sensor data
- `domain_randomization_params` values must be within valid ranges

### PerceptionPipeline Validation
- `configuration.processing_nodes` must form valid processing graph
- `input_topics` and `output_topics` must use valid ROS 2 message types
- `calibration` must be complete for multi-sensor fusion

### NavigationGoal Validation
- `goal_pose` must be in valid navigation space
- `tolerance` values must be positive
- `constraints` must be within robot capabilities

## State Transitions

### IsaacSimScene States
- `CREATED` → `CONFIGURED` → `ACTIVE` → `RENDERING` → `COMPLETE`
- `ACTIVE` → `PAUSED` → `ACTIVE`
- Any state → `ERROR`

### PerceptionPipeline States
- `CREATED` → `CONFIGURED` → `RUNNING` → `STOPPED`
- `RUNNING` → `PAUSED` → `RUNNING`
- Any state → `ERROR`

### NavigationGoal States
- `CREATED` → `PLANNING` → `EXECUTING` → `COMPLETED` / `FAILED`
- `EXECUTING` → `RECOVERING` → `EXECUTING` / `FAILED`
- Any state → `CANCELED`

## Relationships

- `IsaacSimScene` 1 → * `SyntheticDataSample` (generated samples from scene)
- `PerceptionPipeline` 1 → * `SyntheticDataSample` (processed by pipeline)
- `NavigationGoal` 1 → 1 `BipedalPath` (navigation path for goal)
- `BipedalPath` 1 → * `NavigationGoal` (execution results)