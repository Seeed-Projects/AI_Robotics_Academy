# Getting Started - Reachy Mini SDK

## Introduction

Reachy Mini is an educational robotic arm designed for teaching robotics and programming. This SDK provides a comprehensive Python interface for controlling all aspects of the robot.

## Installation

```bash
pip install reachy-mini
```

## Quick Start

### Basic Connection

```python
from reachy_mini import ReachyMini

# Connect to the robot
reachy_mini = ReachyMini()

# The robot is now ready to use!
print(f"Connected: {reachy_mini.io_client.is_connected()}")
```

### Connection Modes

```python
from reachy_mini import ReachyMini

# Auto mode (try localhost, then network)
reachy_mini = ReachyMini(connection_mode="auto")

# Localhost only
reachy_mini = ReachyMini(connection_mode="localhost_only")

# Network discovery
reachy_mini = ReachyMini(connection_mode="network")

# With custom timeout
reachy_mini = ReachyMini(timeout=10.0)
```

### Simulation Mode

```python
# Use simulation without physical robot
reachy_mini = ReachyMini(use_sim=True)
```

## Constructor Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_name` | str | `"reachy_mini"` | Name of the robot |
| `connection_mode` | str | `"auto"` | Connection strategy |
| `spawn_daemon` | bool | `False` | Spawn daemon process |
| `use_sim` | bool | `False` | Use simulation mode |
| `timeout` | float | `5.0` | Connection timeout (seconds) |
| `automatic_body_yaw` | bool | `True` | Auto body yaw adjustment |
| `log_level` | str | `"INFO"` | Logging level |
| `media_backend` | str | `"default"` | Media backend selection |

## Basic Operations

### Wake Up and Sleep

```python
# Perform wake up animation
reachy_mini.wake_up()

# Go to sleep position
reachy_mini.goto_sleep()
```

### Check Connection Status

```python
# Check if connected
is_connected = reachy_mini.io_client.is_connected()

# Get robot status
status = reachy_mini.io_client.get_status()
print(status)
```

### Disconnect

```python
# Properly disconnect
reachy_mini.io_client.disconnect()
```

## Next Steps

- [Head Control](./02-head-control.md) - Control the robot head
- [Antenna Control](./03-antenna-control.md) - Control the antennas
- [Body Control](./04-body-control.md) - Control body movements
- [Media](./05-media.md) - Camera and audio
- [Motor Control](./06-motor-control.md) - Direct motor access
- [IMU](./07-imu.md) - Inertial measurement unit
