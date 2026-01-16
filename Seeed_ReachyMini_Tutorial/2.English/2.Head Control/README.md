# Head Control - Reachy Mini SDK

## Overview

The Reachy Mini head has 7 degrees of freedom (DOF) controlled by a Stewart platform mechanism. This allows for precise positioning and smooth movements.

## Head Pose Control

### Get Current Head Pose

```python
# Get current 4x4 head pose matrix
pose = reachy_mini.get_current_head_pose()
print(pose)
# Output: 4x4 homogeneous transformation matrix
```

### Set Target Head Pose

```python
import numpy as np

# Create a target pose (x, y, z, roll, pitch, yaw)
from reachy_mini.utils import create_head_pose

target_pose = create_head_pose(
    x=0.0, y=0.0, z=0.1,  # 10cm forward
    roll=0.0, pitch=0.1, yaw=0.0,  # Look down slightly
    mm=False, degrees=True
)

# Set immediately
reachy_mini.set_target_head_pose(target_pose)

# Or with smooth transition
reachy_mini.goto_target(
    head=target_pose,
    duration=1.0,  # 1 second
    method="MIN_JERK"
)
```

## Look At Features

### Look at Image Coordinates

```python
# Look at a specific pixel in the camera image
joint_positions = reachy_mini.look_at_image(
    u=320,  # pixel x coordinate
    v=240,  # pixel y coordinate
    duration=0.5,
    perform_movement=True
)
```

### Look at World Coordinates

```python
# Look at a 3D point in world space
joint_positions = reachy_mini.look_at_world(
    x=0.5,  # meters
    y=0.0,
    z=0.3,
    duration=1.0,
    perform_movement=True
)
```

## Joint Position Control

### Get Joint Positions

```python
# Get all joint positions
head_joints, antenna_joints = reachy_mini.get_current_joint_positions()

print(f"Head joints (7 DOF): {head_joints}")
print(f"Antenna joints (2 DOF): {antenna_joints}")
```

### Head Joint Structure

```
Head: 7 DOF
- [0] body_yaw: Body rotation around vertical axis
- [1-6] stewart_platform: 6 joints for Stewart platform control
  Controls: x, y, z position + roll, pitch, yaw orientation
```

## Quick Set Target

### Set Target Pose (Convenience Method)

```python
# Set target with individual parameters
reachy_mini.set_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.1, 0.0),  # (x, y, z, roll, pitch, yaw)
    antennas=None,
    body_yaw=None
)
```

### Go To Target (Smooth Movement)

```python
# Smooth movement to target
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.1, 0.0),
    antennas=(0.5, 0.5),
    duration=2.0,
    method="MIN_JERK",
    body_yaw=0.0
)
```

## Interpolation Methods

Available methods for smooth movement:

| Method | Description |
|--------|-------------|
| `"LINEAR"` | Linear interpolation |
| `"MIN_JERK"` | Minimum jerk trajectory (smoothest) |
| `"EASE_IN_OUT"` | Ease in and out |
| `"CARTOON"` | Cartoon-style bounce effect |

```python
# Use different interpolation methods
methods = ["LINEAR", "MIN_JERK", "EASE_IN_OUT", "CARTOON"]

for method in methods:
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
        duration=1.0,
        method=method
    )
```

## Example: Nodding Motion

```python
import time

# Create nodding motion
for _ in range(3):
    # Look down
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.05, 0.0, 0.2, 0.0),
        duration=0.5,
        method="MIN_JERK"
    )
    time.sleep(0.5)

    # Look straight
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        duration=0.5,
        method="MIN_JERK"
    )
    time.sleep(0.5)
```

## Example: Scanning Motion

```python
# Scan left to right
positions = [
    (0.0, -0.1, 0.0, 0.0, 0.0, -0.3),  # Look left
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),     # Center
    (0.0, 0.1, 0.0, 0.0, 0.0, 0.3),    # Look right
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),     # Center
]

for pos in positions:
    reachy_mini.goto_target(
        head=pos,
        duration=1.0,
        method="MIN_JERK"
    )
    time.sleep(1.0)
```

## Pose Matrix Helper

```python
from reachy_mini.utils import create_head_pose

# Create pose matrix from components
pose = create_head_pose(
    x=0.1,    # x position (meters)
    y=0.05,   # y position (meters)
    z=0.2,    # z position (meters)
    roll=0.0,    # roll rotation (radians)
    pitch=0.1,   # pitch rotation (radians)
    yaw=0.0,     # yaw rotation (radians)
    mm=False,    # use meters (True for millimeters)
    degrees=False # use radians (True for degrees)
)
```
