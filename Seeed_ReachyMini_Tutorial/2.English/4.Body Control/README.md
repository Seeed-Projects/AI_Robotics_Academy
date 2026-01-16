# Body Control - Reachy Mini SDK

## Overview

The body control system manages the rotation of Reachy Mini's body around the vertical axis (yaw). This is the first joint in the 7-DOF head system and provides the base rotation for all head movements.

## Body Yaw Control

### Set Body Yaw Directly

```python
# Set body yaw angle in radians
reachy_mini.set_target_body_yaw(body_yaw=0.5)  # Rotate left 0.5 radians

# Negative values rotate right
reachy_mini.set_target_body_yaw(body_yaw=-0.3)  # Rotate right 0.3 radians
```

### Set Body Yaw with Smooth Movement

```python
# Smooth rotation to target
reachy_mini.goto_target(
    head=None,
    antennas=None,
    body_yaw=0.5,
    duration=1.0,
    method="MIN_JERK"
)
```

### Quick Set Method

```python
# Convenience method
reachy_mini.set_target(
    head=None,
    antennas=None,
    body_yaw=0.5
)
```

## Automatic Body Yaw

The automatic body yaw feature allows the robot to automatically adjust its body rotation to better reach target head positions.

### Enable Automatic Body Yaw

```python
# Enable (default)
reachy_mini.set_automatic_body_yaw(automatic_body_yaw=True)
```

### Disable Automatic Body Yaw

```python
# Disable - body yaw stays fixed
reachy_mini.set_automatic_body_yaw(automatic_body_yaw=False)
```

### Initialize with Auto Body Yaw Setting

```python
from reachy_mini import ReachyMini

# Create with automatic body yaw enabled (default)
reachy_mini = ReachyMini(automatic_body_yaw=True)

# Or disable it
reachy_mini = ReachyMini(automatic_body_yaw=False)
```

## How Automatic Body Yaw Works

When enabled, the kinematics system automatically calculates the optimal body yaw angle to reach a given head pose. This provides:

- **Extended Workspace**: Can reach positions that would be impossible with fixed body yaw
- **Natural Movements**: Mimics how humans naturally rotate their body
- **Optimal Posture**: Finds the most comfortable body position for each head pose

### Example: Automatic vs Manual

```python
# With automatic body yaw (enabled)
reachy_mini.set_automatic_body_yaw(True)
reachy_mini.goto_target(
    head=(0.2, 0.1, 0.1, 0.0, 0.0, 0.0),
    duration=1.0
)
# Body yaw automatically calculated

# With manual body yaw (disabled)
reachy_mini.set_automatic_body_yaw(False)
reachy_mini.goto_target(
    head=(0.2, 0.1, 0.1, 0.0, 0.0, 0.0),
    body_yaw=0.5,  # Must specify manually
    duration=1.0
)
```

## Example: Scanning with Body Rotation

```python
import time

# Scan left and right using body yaw
angles = [-0.5, -0.25, 0.0, 0.25, 0.5, 0.0]

for angle in angles:
    reachy_mini.goto_target(
        body_yaw=angle,
        duration=0.5
    )
    time.sleep(0.5)
```

## Example: Tracking Object

```python
# Simulate tracking an object moving left to right
import time
import math

for i in range(20):
    t = i / 20.0

    # Object moves in arc
    x = 0.3 * math.cos(math.pi * t)
    y = 0.3 * math.sin(math.pi * t)

    # Look at object with automatic body yaw
    reachy_mini.look_at_world(
        x=x, y=y, z=0.2,
        duration=0.1,
        perform_movement=True
    )

    time.sleep(0.1)
```

## Example: Greeting Motion

```python
import time

# Bow and turn to greet
def greet(direction):
    """
    direction: 1 for right, -1 for left
    """
    # Turn body
    reachy_mini.goto_target(
        body_yaw=direction * 0.4,
        duration=0.5
    )

    # Bow head
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.05, 0.0, 0.2, 0.0),
        duration=0.3
    )

    time.sleep(0.5)

    # Return to neutral
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        body_yaw=0.0,
        duration=0.5
    )

# Greet to the right
greet(direction=1)

time.sleep(1.0)

# Greet to the left
greet(direction=-1)
```

## Body Yaw Limits

| Parameter | Value | Description |
|-----------|-------|-------------|
| Min Angle | ~-1.0 rad | Maximum right rotation |
| Max Angle | ~1.0 rad | Maximum left rotation |
| Center | 0.0 rad | Forward facing |

Note: Actual limits may vary based on calibration and mechanical constraints.

## Joint Position Reference

When getting joint positions, body yaw is the first element:

```python
head_joints, antenna_joints = reachy_mini.get_current_joint_positions()

body_yaw = head_joints[0]  # First joint
stewart_joints = head_joints[1:]  # Remaining 6 joints

print(f"Body yaw: {body_yaw} radians")
print(f"Stewart platform: {stewart_joints}")
```
