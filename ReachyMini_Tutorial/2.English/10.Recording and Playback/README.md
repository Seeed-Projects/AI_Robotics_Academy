# Recording - Reachy Mini SDK

## Overview

Reachy Mini can record various data during operation, including joint positions, timestamps, and other telemetry. This is useful for:
- Recording movements for playback
- Analyzing robot behavior
- Creating motion datasets
- Debugging

## Start Recording

```python
# Start recording
reachy_mini.start_recording()

print("Recording started...")
```

## Stop Recording

```python
# Stop recording and get data
data = reachy_mini.stop_recording()

if data:
    print(f"Recorded {len(data)} frames")
    print(f"First frame: {data[0]}")
```

## Recorded Data Structure

Each recorded frame contains:

```python
{
    'timestamp': float,        # Time since start
    'head_joints': [...],      # Head joint positions (7 DOF)
    'antenna_joints': [...],   # Antenna joint positions (2 DOF)
    'body_yaw': float,         # Body yaw angle
    # Additional fields may be present
}
```

## Example: Record and Playback Movement

```python
import time

# Start recording
reachy_mini.start_recording()

# Perform movements
poses = [
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, -0.1, 0.1, 0.0, 0.0, -0.3),
    (0.0, 0.0, 0.15, 0.0, -0.1, 0.0),
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
]

for pose in poses:
    reachy_mini.goto_target(
        head=pose,
        duration=1.0,
        method="MIN_JERK"
    )
    time.sleep(1.1)

# Stop recording
data = reachy_mini.stop_recording()

print(f"Recorded {len(data)} frames")
```

## Example: Playback Recorded Movement

```python
import time

# Assume we have recorded data
if data:
    print(f"Playing back {len(data)} frames...")

    for frame in data:
        # Extract joint positions
        head_joints = frame.get('head_joints')
        antenna_joints = frame.get('antenna_joints')

        if head_joints:
            # Set joint positions
            reachy_mini.set_target_antenna_joint_positions(
                antennas=antenna_joints
            )

        # Wait for next frame (adjust based on recording speed)
        time.sleep(0.033)  # ~30 FPS
```

## Example: Save Recording to File

```python
import json
from datetime import datetime

# Record some data
reachy_mini.start_recording()

# ... perform movements ...

data = reachy_mini.stop_recording()

# Save to file
if data:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"recording_{timestamp}.json"

    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"Saved recording to {filename}")
```

## Example: Load Recording from File

```python
import json

# Load recording
with open('recording_20250110_140000.json', 'r') as f:
    data = json.load(f)

print(f"Loaded {len(data)} frames")

# Playback
import time

for frame in data:
    head_joints = frame.get('head_joints')
    antenna_joints = frame.get('antenna_joints')

    if antenna_joints:
        reachy_mini.set_target_antenna_joint_positions(
            antennas=antenna_joints
        )

    time.sleep(0.033)
```

## Example: Analyze Recording

```python
import json
import numpy as np

# Load recording
with open('recording.json', 'r') as f:
    data = json.load(f)

# Extract joint trajectories
head_trajectories = []
antenna_trajectories = []

for frame in data:
    if 'head_joints' in frame:
        head_trajectories.append(frame['head_joints'])
    if 'antenna_joints' in frame:
        antenna_trajectories.append(frame['antenna_joints'])

# Convert to numpy arrays
head_trajectories = np.array(head_trajectories)
antenna_trajectories = np.array(antenna_trajectories)

# Analyze
print(f"Recording duration: {data[-1]['timestamp']:.2f} seconds")
print(f"Average head position: {np.mean(head_trajectories, axis=0)}")
print(f"Head position range: {np.ptp(head_trajectories, axis=0)}")

# Find maximum velocity
if len(head_trajectories) > 1:
    velocities = np.diff(head_trajectories, axis=0)
    max_velocity = np.max(np.abs(velocities))
    print(f"Maximum velocity: {max_velocity:.4f} rad/frame")
```

## Example: Visualize Recording

```python
import json
import matplotlib.pyplot as plt

# Load recording
with open('recording.json', 'r') as f:
    data = json.load(f)

# Extract data
timestamps = [frame['timestamp'] for frame in data]
antenna_right = [frame.get('antenna_joints', [0, 0])[0] for frame in data]
antenna_left = [frame.get('antenna_joints', [0, 0])[1] for frame in data]

# Plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Right antenna
ax1.plot(timestamps, antenna_right, 'b-', linewidth=2)
ax1.set_ylabel('Right Antenna (rad)')
ax1.set_title('Recorded Antenna Movements')
ax1.grid(True)

# Left antenna
ax2.plot(timestamps, antenna_left, 'r-', linewidth=2)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Left Antenna (rad)')
ax2.grid(True)

plt.tight_layout()
plt.show()
```

## Example: Record with Metadata

```python
import json
from datetime import datetime

def create_recording_metadata(description, tags=None):
    """Create metadata for recording"""
    return {
        'timestamp': datetime.now().isoformat(),
        'description': description,
        'tags': tags or [],
        'version': '1.0'
    }

# Start recording
reachy_mini.start_recording()

# ... perform movements ...

data = reachy_mini.stop_recording()

# Create recording with metadata
recording = {
    'metadata': create_recording_metadata(
        description='Nodding motion',
        tags=['demo', 'nodding']
    ),
    'frames': data
}

# Save
with open('recording_with_metadata.json', 'w') as f:
    json.dump(recording, f, indent=2)
```

## Example: Loop Recording

```python
import time

# Record in a loop
recordings = []

for i in range(3):
    print(f"Recording take {i+1}...")

    # Start recording
    reachy_mini.start_recording()

    # Perform movement
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
        duration=1.0,
        method="MIN_JERK"
    )
    time.sleep(1.1)

    # Stop recording
    data = reachy_mini.stop_recording()
    recordings.append(data)

    # Return to start
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        duration=0.5
    )
    time.sleep(0.6)

print(f"Recorded {len(recordings)} takes")
```

## Recording Tips

### 1. Keep Recording Sessions Focused

```python
# Record one specific movement at a time
reachy_mini.start_recording()
# ... perform single movement ...
data = reachy_mini.stop_recording()
```

### 2. Add Pauses Between Movements

```python
# Add slight pause at start and end
time.sleep(0.5)  # Pause before
reachy_mini.start_recording()
# ... movement ...
data = reachy_mini.stop_recording()
time.sleep(0.5)  # Pause after
```

### 3. Use Consistent Timing

```python
# Use fixed duration movements
duration = 1.0
reachy_mini.goto_target(head=pose, duration=duration)
time.sleep(duration + 0.1)  # Consistent wait
```

### 4. Check Recording Success

```python
# Verify recording was successful
data = reachy_mini.stop_recording()

if data is None or len(data) == 0:
    print("Warning: No data recorded!")
else:
    print(f"Successfully recorded {len(data)} frames")
```
