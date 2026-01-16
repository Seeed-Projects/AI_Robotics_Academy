# Interpolation & Motion - Reachy Mini SDK

## Overview

Reachy Mini provides multiple interpolation techniques for creating smooth, natural movements. The interpolation system controls how the robot transitions between poses.

## Interpolation Methods

| Method | Description | Best For |
|--------|-------------|----------|
| `LINEAR` | Linear interpolation | Simple, predictable movements |
| `MIN_JERK` | Minimum jerk trajectory | Smoothest, most natural motion |
| `EASE_IN_OUT` | Ease in and out | Gentle start and stop |
| `CARTOON` | Cartoon bounce effect | Fun, expressive movements |

## Using Interpolation with goto_target

### Basic Usage

```python
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    duration=1.0,
    method="MIN_JERK"  # Interpolation method
)
```

### Linear Interpolation

```python
# Constant velocity throughout
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    duration=1.0,
    method="LINEAR"
)
```

### Minimum Jerk (Recommended)

```python
# Smoothest motion, minimizes acceleration changes
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    duration=1.0,
    method="MIN_JERK"
)
```

### Ease In Out

```python
# Gentle start, accelerate, then decelerate
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    duration=1.0,
    method="EASE_IN_OUT"
)
```

### Cartoon Effect

```python
# Bouncy, cartoon-like motion
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    duration=1.0,
    method="CARTOON"
)
```

## Motion Duration

The duration parameter controls how long the movement takes.

### Fast Movement

```python
# Quick movement (0.3 seconds)
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    duration=0.3,
    method="MIN_JERK"
)
```

### Slow Movement

```python
# Slow, deliberate movement (2 seconds)
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    duration=2.0,
    method="MIN_JERK"
)
```

## Example: Compare Interpolation Methods

```python
import time

# Target position
target = (0.0, 0.0, 0.1, 0.0, 0.0, 0.0)

methods = ["LINEAR", "MIN_JERK", "EASE_IN_OUT", "CARTOON"]

for method in methods:
    print(f"Using {method}...")

    # Move to target
    reachy_mini.goto_target(
        head=target,
        duration=1.0,
        method=method
    )

    time.sleep(1.5)  # Wait for movement to complete

    # Return to center
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        duration=1.0,
        method=method
    )

    time.sleep(1.5)
```

## Example: Smooth Sequence

```python
import time

# Define a sequence of poses
poses = [
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),   # Center
    (0.0, -0.1, 0.1, 0.0, 0.0, -0.3), # Left
    (0.0, 0.0, 0.15, 0.0, -0.1, 0.0), # Up
    (0.0, 0.1, 0.1, 0.0, 0.0, 0.3),   # Right
    (0.0, 0.0, 0.05, 0.0, 0.2, 0.0),  # Down
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),   # Center
]

# Execute sequence with smooth interpolation
for pose in poses:
    reachy_mini.goto_target(
        head=pose,
        duration=1.0,
        method="MIN_JERK"  # Smoothest
    )
    time.sleep(1.1)  # Slightly longer than duration
```

## Example: Dynamic Duration

```python
import math

def calculate_duration(distance, max_speed=0.2):
    """Calculate duration based on distance"""
    return distance / max_speed

# Current position
current_head = reachy_mini.get_current_head_pose()
current_pos = current_head[:3, 3]

# Target position
target = create_head_pose(0.1, 0.0, 0.15, 0.0, 0.0, 0.0)
target_pos = target[:3, 3]

# Calculate distance
distance = np.linalg.norm(target_pos - current_pos)

# Calculate duration
duration = calculate_duration(distance)

# Move with calculated duration
reachy_mini.goto_target(
    head=(0.1, 0.0, 0.15, 0.0, 0.0, 0.0),
    duration=duration,
    method="MIN_JERK"
)
```

## Helper Functions

### Minimum Jerk Trajectory

```python
from reachy_mini.utils import minimum_jerk
import numpy as np

# Define start and goal positions
start_pos = np.array([0.0, 0.0, 0.0])
goal_pos = np.array([0.1, 0.0, 0.1])

# Create minimum jerk trajectory
trajectory_func = minimum_jerk(
    starting_position=start_pos,
    goal_position=goal_pos,
    duration=1.0,
    sample_time=0.01
)

# Evaluate at different times
for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    position = trajectory_func(t)
    print(f"t={t:.2f}: {position}")
```

### Time Trajectory

```python
from reachy_mini.utils import time_trajectory

# Get time trajectory value for different methods
methods = ["LINEAR", "MIN_JERK", "EASE_IN_OUT", "CARTOON"]

for method in methods:
    print(f"\n{method}:")
    for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
        value = time_trajectory(t, method)
        print(f"  t={t:.2f}: {value:.3f}")
```

## Example: Custom Animation

```python
import numpy as np
import time

def create_animation_keyframes(duration=2.0, fps=30):
    """Create animation keyframes"""
    keyframes = []
    num_frames = int(duration * fps)

    for i in range(num_frames):
        t = i / num_frames

        # Create circular motion
        x = 0.1 * np.cos(2 * np.pi * t)
        y = 0.1 * np.sin(2 * np.pi * t)
        z = 0.15 + 0.05 * np.sin(4 * np.pi * t)

        keyframes.append((x, y, z, 0.0, 0.0, 0.0))

    return keyframes

# Generate keyframes
keyframes = create_animation_keyframes(duration=3.0)

# Play animation
frame_duration = 1.0 / 30  # 30 FPS

for pose in keyframes:
    reachy_mini.goto_target(
        head=pose,
        duration=frame_duration,
        method="MIN_JERK"
    )
    time.sleep(frame_duration)
```

## Example: Bezier Curve Motion

```python
import numpy as np

def bezier_point(t, control_points):
    """Calculate point on Bezier curve at time t"""
    n = len(control_points) - 1
    point = np.zeros(3)

    for i, cp in enumerate(control_points):
        # Bernstein polynomial
        coeff = np.math.comb(n, i) * (1-t)**(n-i) * t**i
        point += coeff * np.array(cp[:3])

    return point

# Define control points
control_points = [
    (0.0, 0.0, 0.0),
    (0.05, -0.05, 0.1),
    (0.1, 0.05, 0.15),
    (0.1, 0.0, 0.1)
]

# Follow Bezier curve
steps = 50

for i in range(steps):
    t = i / (steps - 1)
    point = bezier_point(t, control_points)

    reachy_mini.goto_target(
        head=(point[0], point[1], point[2], 0.0, 0.0, 0.0),
        duration=0.05,
        method="MIN_JERK"
    )
```

## Best Practices

### 1. Use MIN_JERK for Natural Movement

```python
# Default recommendation
reachy_mini.goto_target(
    head=target,
    duration=1.0,
    method="MIN_JERK"  # Most natural
)
```

### 2. Adjust Duration Based on Distance

```python
# Longer distance = longer duration
duration = max(0.5, distance * 5)  # Minimum 0.5 seconds
```

### 3. Combine with Body Movements

```python
# Coordinate head, antennas, and body
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    antennas=(0.5, 0.5),
    body_yaw=0.0,
    duration=1.0,
    method="MIN_JERK"
)
```
