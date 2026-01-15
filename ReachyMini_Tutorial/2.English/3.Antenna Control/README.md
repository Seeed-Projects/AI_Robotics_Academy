# Antenna Control - Reachy Mini SDK

## Overview

Reachy Mini has two antennas (left and right) that can be controlled independently. Each antenna has 1 degree of freedom for angle adjustment.

## Antenna Joint Structure

```
Antennas: 2 DOF
- [0] right_angle: Right antenna angle
- [1] left_angle: Left antenna angle
```

## Get Antenna Positions

### Get All Antenna Positions

```python
# Get both head and antenna positions
head_joints, antenna_joints = reachy_mini.get_current_joint_positions()

print(f"Right antenna: {antenna_joints[0]}")
print(f"Left antenna: {antenna_joints[1]}")
```

### Get Only Antenna Positions

```python
# Get just the antenna positions
antenna_positions = reachy_mini.get_present_antenna_joint_positions()

print(f"Antenna positions: {antenna_positions}")
```

## Set Antenna Positions

### Set Target Antenna Positions

```python
# Set antenna angles directly
reachy_mini.set_target_antenna_joint_positions(
    antennas=[0.5, 0.5]  # [right, left] in radians
)

# Or via goto_target for smooth movement
reachy_mini.goto_target(
    head=None,
    antennas=[0.5, 0.5],  # [right, left]
    duration=0.5
)
```

### Combined Head and Antenna Control

```python
# Control head and antennas together
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),  # Head pose
    antennas=[0.5, 0.5],                     # Antenna angles
    duration=1.0,
    method="MIN_JERK"
)
```

## Quick Set Method

```python
# Convenience method for quick positioning
reachy_mini.set_target(
    head=None,           # Don't change head
    antennas=[0.5, 0.5], # Set antenna angles
    body_yaw=None        # Don't change body yaw
)
```

## Example: Antenna Wave

```python
import time

# Wave antennas alternately
for _ in range(3):
    # Right antenna up
    reachy_mini.goto_target(
        antennas=[0.8, 0.0],
        duration=0.3
    )
    time.sleep(0.3)

    # Left antenna up
    reachy_mini.goto_target(
        antennas=[0.0, 0.8],
        duration=0.3
    )
    time.sleep(0.3)

# Return to center
reachy_mini.goto_target(
    antennas=[0.5, 0.5],
    duration=0.5
)
```

## Example: Antenna Dance

```python
import time
import math

# Animate antennas in a pattern
duration = 2.0
steps = 20

for i in range(steps):
    t = i / steps

    # Create oscillating motion
    right_angle = 0.5 + 0.3 * math.sin(2 * math.pi * t)
    left_angle = 0.5 + 0.3 * math.sin(2 * math.pi * t + math.pi)

    reachy_mini.set_target_antenna_joint_positions(
        antennas=[right_angle, left_angle]
    )

    time.sleep(duration / steps)

# Reset to center
reachy_mini.set_target_antenna_joint_positions(
    antennas=[0.5, 0.5]
)
```

## Example: Expressive Movements

```python
# Different antenna expressions

# Happy / Excited (both antennas up)
reachy_mini.goto_target(
    antennas=[0.8, 0.8],
    duration=0.5
)

# Curious (one antenna up)
reachy_mini.goto_target(
    antennas=[0.7, 0.3],
    duration=0.5
)

# Alert (both antennas forward)
reachy_mini.goto_target(
    antennas=[0.2, 0.2],
    duration=0.3
)

# Relaxed (center position)
reachy_mini.goto_target(
    antennas=[0.5, 0.5],
    duration=0.5
)
```

## Combined Movement Example

```python
# Coordinated head and antenna movement
import time

# Look at something with interest
reachy_mini.goto_target(
    head=(0.0, 0.1, 0.15, 0.0, -0.1, 0.3),  # Look right and up
    antennas=[0.7, 0.3],                       # Right antenna up
    duration=1.0,
    method="MIN_JERK"
)

time.sleep(1.0)

# Switch attention
reachy_mini.goto_target(
    head=(0.0, -0.1, 0.15, 0.0, -0.1, -0.3), # Look left and up
    antennas=[0.3, 0.7],                       # Left antenna up
    duration=1.0,
    method="MIN_JERK"
)
```

## Antenna Angle Range

| Antenna | Min Angle | Max Angle | Center |
|---------|-----------|-----------|--------|
| Right | ~0.0 rad | ~1.0 rad | 0.5 rad |
| Left | ~0.0 rad | ~1.0 rad | 0.5 rad |

Note: Actual range may vary slightly depending on calibration and mechanical limits.
