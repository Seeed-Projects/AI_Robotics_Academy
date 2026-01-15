# Async Operations - Reachy Mini SDK

## Overview

Reachy Mini supports asynchronous operations, allowing you to start movements that run in the background while your code continues executing. This is useful for:
- Non-blocking movements
- Parallel operations
- Complex sequencing
- Real-time control

## Async Move Playback

### Basic Async Playback

```python
import asyncio

# Play a move asynchronously
await reachy_mini.async_play_move(
    move=your_move_object,
    play_frequency=100,  # Hz
    initial_goto_duration=1.0,
    sound=None  # Optional sound file
)
```

### Async with Sound

```python
# Play move with sound
await reachy_mini.async_play_move(
    move=your_move_object,
    play_frequency=100,
    initial_goto_duration=1.0,
    sound="path/to/sound.wav"  # Play sound during movement
)
```

## Creating Custom Moves

Moves are defined by creating a class that implements the `Move` interface:

```python
from reachy_mini.motion import Move
import numpy as np

class CustomMove(Move):
    def __init__(self):
        super().__init__()
        self.duration = 2.0  # Movement duration in seconds

    def evaluate(self, t):
        """
        Evaluate move at time t (0 to duration)
        Returns: (head_pose, antennas, body_yaw)
        """
        # Calculate head pose at time t
        if t < 1.0:
            # First half: look left
            head = (0.0, -0.1 * t, 0.1 * t, 0.0, 0.0, -0.3 * t)
        else:
            # Second half: look right
            t2 = t - 1.0
            head = (0.0, 0.1 * t2, 0.1 - 0.05 * t2, 0.0, 0.0, 0.3 * t2)

        antennas = (0.5, 0.5)
        body_yaw = 0.0

        return head, antennas, body_yaw

# Use the move
move = CustomMove()
await reachy_mini.async_play_move(move, play_frequency=100)
```

## Example: Smooth Animation Move

```python
import numpy as np
from reachy_mini.motion import Move

class SmoothCircleMove(Move):
    """Move head in a smooth circle"""

    def __init__(self, duration=3.0):
        super().__init__()
        self.duration = duration
        self.radius = 0.05

    def evaluate(self, t):
        # Calculate angle (0 to 2π)
        theta = 2 * np.pi * t / self.duration

        # Circle in x-y plane
        x = self.radius * np.cos(theta)
        y = self.radius * np.sin(theta)
        z = 0.15

        # Slight tilt for natural look
        roll = 0.05 * np.sin(2 * theta)
        pitch = 0.05 * np.cos(2 * theta)
        yaw = 0.0

        head = (x, y, z, roll, pitch, yaw)
        antennas = (0.5 + 0.1 * np.sin(theta), 0.5 + 0.1 * np.cos(theta))
        body_yaw = 0.0

        return head, antennas, body_yaw

# Play the move
move = SmoothCircleMove(duration=3.0)
await reachy_mini.async_play_move(move, play_frequency=100)
```

## Example: Sequential Moves

```python
import asyncio

async def play_sequence():
    """Play multiple moves in sequence"""

    # Define moves
    moves = [
        NodMove(),
        ShakeMove(),
        LookAroundMove()
    ]

    # Play sequentially
    for move in moves:
        await reachy_mini.async_play_move(
            move=move,
            play_frequency=100,
            initial_goto_duration=0.5
        )

# Run sequence
await play_sequence()
```

## Example: Parallel Tasks

```python
import asyncio

async def head_movement():
    """Control head movement"""
    move = HeadScanMove()
    await reachy_mini.async_play_move(move, play_frequency=100)

async def antenna_dance():
    """Control antennas independently"""
    while True:
        reachy_mini.set_target_antenna_joint_positions([0.8, 0.2])
        await asyncio.sleep(0.5)
        reachy_mini.set_target_antenna_joint_positions([0.2, 0.8])
        await asyncio.sleep(0.5)

async def parallel_demo():
    """Run head and antennas in parallel"""
    # Create tasks
    head_task = asyncio.create_task(head_movement())
    antenna_task = asyncio.create_task(antenna_dance())

    # Wait for head to finish
    await head_task

    # Cancel antenna task
    antenna_task.cancel()

# Run
await parallel_demo()
```

## Example: Cancellation

```python
import asyncio

async def play_with_timeout():
    """Play move with timeout"""

    # Create task
    task = asyncio.create_task(
        reachy_mini.async_play_move(move, play_frequency=100)
    )

    try:
        # Wait for max 5 seconds
        await asyncio.wait_for(task, timeout=5.0)
    except asyncio.TimeoutError:
        print("Movement timed out, cancelling...")
        task.cancel()
```

## Example: Real-time Adjustment

```python
import asyncio

async def interactive_movement():
    """Movement that can be adjusted in real-time"""

    class InteractiveMove(Move):
        def __init__(self):
            super().__init__()
            self.duration = 10.0  # Long duration
            self.target_z = 0.15  # Can be modified

        def evaluate(self, t):
            # Look at adjustable target
            return (0.0, 0.0, self.target_z, 0.0, 0.0, 0.0), (0.5, 0.5), 0.0

    move = InteractiveMove()

    # Start movement
    task = asyncio.create_task(
        reachy_mini.async_play_move(move, play_frequency=50)
    )

    # Adjust target during movement
    for i in range(5):
        await asyncio.sleep(1.0)
        move.target_z = 0.15 + i * 0.02  # Raise head
        print(f"Adjusted target to {move.target_z}")

    await task
```

## Synchronous Wrapper

For non-async code, use the synchronous wrapper:

```python
# Synchronous playback (blocks until complete)
reachy_mini.play_move(
    move=your_move_object,
    play_frequency=100,
    initial_goto_duration=1.0,
    sound=None
)
```

## Best Practices

### 1. Handle Exceptions

```python
try:
    await reachy_mini.async_play_move(move, play_frequency=100)
except Exception as e:
    print(f"Movement failed: {e}")
```

### 2. Clean Up Resources

```python
async def safe_movement():
    try:
        await reachy_mini.async_play_move(move, play_frequency=100)
    finally:
        # Ensure motors are in safe state
        reachy_mini.disable_motors()
```

### 3. Use Appropriate Frequency

```python
# Higher frequency = smoother but more CPU
# Lower frequency = more efficient but less smooth

# For smooth movements: 100 Hz
await reachy_mini.async_play_move(move, play_frequency=100)

# For simple movements: 50 Hz
await reachy_mini.async_play_move(move, play_frequency=50)
```

### 4. Check Movement Status

```python
# For long movements, add checkpoints
async def monitored_movement(move):
    task = asyncio.create_task(
        reachy_mini.async_play_move(move, play_frequency=100)
    )

    while not task.done():
        # Check status
        print("Movement in progress...")
        await asyncio.sleep(1.0)

    await task
    print("Movement complete!")
```
