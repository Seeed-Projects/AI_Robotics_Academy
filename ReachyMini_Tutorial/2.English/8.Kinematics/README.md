# Kinematics - Reachy Mini SDK

## Overview

The kinematics system handles the mathematical calculations for converting between joint angles and head poses. Reachy Mini uses a Stewart platform mechanism for head control, requiring specialized kinematics.

## Kinematics Engines

Reachy Mini supports multiple kinematics engines:

| Engine | Description | Use Case |
|--------|-------------|----------|
| `AnalyticalKinematics` | Analytical solution | Default, fast, accurate |
| `NNKinematics` | Neural network | Optional, experimental |
| `PlacoKinematics` | Placo solver | Optional, advanced |

The analytical kinematics engine is used by default and provides accurate solutions for most use cases.

## Forward Kinematics (FK)

Converts joint angles to head pose.

### Basic Forward Kinematics

```python
# Joint angles: [body_yaw, stewart_1, ..., stewart_6]
joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Calculate forward kinematics
pose = reachy_mini.kinematics.fk(joint_angles)

print(f"Head pose:\n{pose}")
# Output: 4x4 homogeneous transformation matrix
```

### Extract Position from Pose

```python
import numpy as np

pose = reachy_mini.kinematics.fk(joint_angles)

# Extract position (x, y, z)
position = pose[:3, 3]
print(f"Position: x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")
```

### Extract Rotation from Pose

```python
import numpy as np

pose = reachy_mini.kinematics.fk(joint_angles)

# Extract rotation matrix
rotation = pose[:3, :3]
print(f"Rotation matrix:\n{rotation}")
```

## Inverse Kinematics (IK)

Converts desired head pose to joint angles.

### Basic Inverse Kinematics

```python
import numpy as np

# Target pose (4x4 matrix)
target_pose = np.array([
    [1, 0, 0, 0.1],
    [0, 1, 0, 0.0],
    [0, 0, 1, 0.2],
    [0, 0, 0, 1]
])

# Calculate inverse kinematics
joint_angles = reachy_mini.kinematics.ik(pose=target_pose)

print(f"Joint angles: {joint_angles}")
# Output: [body_yaw, stewart_1, ..., stewart_6]
```

### Inverse Kinematics with Body Yaw

```python
# Specify body yaw angle
joint_angles = reachy_mini.kinematics.ik(
    pose=target_pose,
    body_yaw=0.5  # radians
)
```

### Disable Collision Checking

```python
# Disable collision checking for faster calculation
joint_angles = reachy_mini.kinematics.ik(
    pose=target_pose,
    check_collision=False
)
```

### Set Iteration Count

```python
# Set number of iterations for solver
joint_angles = reachy_mini.kinematics.ik(
    pose=target_pose,
    no_iterations=100
)
```

## Using Kinematics with look_at

The `look_at` functions internally use inverse kinematics to calculate the required joint angles.

### Look at Image Point

```python
# This uses IK internally
joint_positions = reachy_mini.look_at_image(
    u=320, v=240,
    duration=0.5,
    perform_movement=True
)
```

### Look at World Point

```python
# This also uses IK internally
joint_positions = reachy_mini.look_at_world(
    x=0.5, y=0.0, z=0.3,
    duration=1.0,
    perform_movement=True
)
```

## Automatic Body Yaw

The kinematics system can automatically calculate the optimal body yaw angle for reaching a target pose.

### Enable Automatic Body Yaw

```python
# Enable in kinematics
reachy_mini.kinematics.set_automatic_body_yaw(True)

# Now IK will automatically calculate body yaw
joint_angles = reachy_mini.kinematics.ik(pose=target_pose)
```

### Disable Automatic Body Yaw

```python
# Disable in kinematics
reachy_mini.kinematics.set_automatic_body_yaw(False)

# Must specify body yaw manually
joint_angles = reachy_mini.kinematics.ik(
    pose=target_pose,
    body_yaw=0.0
)
```

## Example: Calculate Reachable Positions

```python
import numpy as np
import math

def is_reachable(reachy_mini, x, y, z):
    """Check if a position is reachable"""
    from reachy_mini.utils import create_head_pose

    # Create target pose
    target_pose = create_head_pose(
        x=x, y=y, z=z,
        roll=0, pitch=0, yaw=0
    )

    try:
        # Try to calculate IK
        joint_angles = reachy_mini.kinematics.ik(target_pose)
        return joint_angles is not None
    except:
        return False

# Test various positions
print("Testing reachable positions...")

for x in np.linspace(-0.2, 0.2, 5):
    for y in np.linspace(-0.2, 0.2, 5):
        for z in np.linspace(0.1, 0.3, 3):
            reachable = is_reachable(reachy_mini, x, y, z)
            status = "✓" if reachable else "✗"
            print(f"{status} ({x:.2f}, {y:.2f}, {z:.2f})")
```

## Example: Workspace Visualization

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def sample_workspace(reachy_mini, resolution=10):
    """Sample the workspace and return reachable points"""
    reachable_points = []

    # Define workspace bounds
    x_range = np.linspace(-0.2, 0.2, resolution)
    y_range = np.linspace(-0.2, 0.2, resolution)
    z_range = np.linspace(0.1, 0.3, resolution)

    for x in x_range:
        for y in y_range:
            for z in z_range:
                if is_reachable(reachy_mini, x, y, z):
                    reachable_points.append([x, y, z])

    return np.array(reachable_points)

# Sample workspace
points = sample_workspace(reachy_mini, resolution=8)

# Visualize
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

if len(points) > 0:
    ax.scatter(points[:, 0], points[:, 1], points[:, 2],
               c=points[:, 2], cmap='viridis', s=10)

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Reachy Mini Workspace')

plt.show()
```

## Example: Trajectory Planning

```python
import numpy as np
from reachy_mini.utils import create_head_pose

def plan_trajectory(reachy_mini, start_pose, end_pose, steps=10):
    """Plan a trajectory between two poses"""
    trajectory = []

    for i in range(steps):
        t = i / (steps - 1)

        # Linear interpolation
        interpolated_pose = start_pose + t * (end_pose - start_pose)

        # Calculate IK for each point
        joint_angles = reachy_mini.kinematics.ik(interpolated_pose)

        if joint_angles is not None:
            trajectory.append(joint_angles)

    return trajectory

# Define start and end poses
start = create_head_pose(0, 0, 0.15, 0, 0, 0)
end = create_head_pose(0.1, 0.1, 0.2, 0.1, 0, 0)

# Plan trajectory
trajectory = plan_trajectory(reachy_mini, start, end, steps=20)

# Execute trajectory
for joint_angles in trajectory:
    reachy_mini.set_target_antenna_joint_positions(
        antennas=joint_angles[-2:]  # Last two are antennas
    )
    # Additional control here...
```

## Kinematics Helper Functions

### Create Head Pose

```python
from reachy_mini.utils import create_head_pose

pose = create_head_pose(
    x=0.1, y=0.0, z=0.2,    # Position
    roll=0.0, pitch=0.1, yaw=0.0,  # Orientation
    mm=False,     # Use meters
    degrees=False # Use radians
)
```

### Linear Pose Interpolation

```python
from reachy_mini.utils import linear_pose_interpolation

import numpy as np

start_pose = np.eye(4)
end_pose = create_head_pose(0.1, 0, 0.1, 0, 0, 0)

# Interpolate at t=0.5 (halfway)
interpolated = linear_pose_interpolation(start_pose, end_pose, t=0.5)
```

### Calculate Angle Between Rotations

```python
from reachy_mini.utils import delta_angle_between_mat_rot

import numpy as np

R1 = np.eye(3)
R2 = create_head_pose(0, 0, 0, 0.1, 0, 0)[:3, :3]

# Calculate angle between rotations
angle = delta_angle_between_mat_rot(R1, R2)
print(f"Angle: {angle:.4f} radians")
```
