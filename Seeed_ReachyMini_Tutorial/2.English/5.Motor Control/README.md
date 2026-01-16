# Motor Control - Reachy Mini SDK

## Overview

Reachy Mini uses multiple motors to control its joints. The motor control system allows you to enable, disable, and configure individual motors or groups of motors.

## Enable/Disable Motors

### Enable All Motors

```python
# Enable all motors
reachy_mini.enable_motors()
```

### Enable Specific Motors

```python
# Enable specific motor by ID
reachy_mini.enable_motors(ids=[1, 2, 3])

# Enable single motor
reachy_mini.enable_motors(ids=1)
```

### Disable All Motors

```python
# Disable all motors (robot goes limp)
reachy_mini.disable_motors()
```

### Disable Specific Motors

```python
# Disable specific motors
reachy_mini.disable_motors(ids=[1, 2, 3])

# Disable single motor
reachy_mini.disable_motors(ids=1)
```

## Gravity Compensation

Gravity compensation helps the robot hold its position without active control, making it safer and more energy-efficient.

### Enable Gravity Compensation

```python
# Enable gravity compensation
reachy_mini.enable_gravity_compensation()
```

### Disable Gravity Compensation

```python
# Disable gravity compensation
reachy_mini.disable_gravity_compensation()
```

## Motor IDs

Reachy Mini has multiple motors for different joints:

| Motor ID | Joint | Description |
|----------|-------|-------------|
| 1 | Body Yaw | Main body rotation |
| 2-7 | Stewart Platform | Head position control (6 DOF) |
| 8 | Right Antenna | Right antenna angle |
| 9 | Left Antenna | Left antenna angle |

Note: Motor IDs may vary based on hardware configuration. Check your robot's documentation for exact IDs.

## Example: Selective Motor Control

```python
# Enable only head motors, disable antennas
reachy_mini.enable_motors(ids=[1, 2, 3, 4, 5, 6, 7])
reachy_mini.disable_motors(ids=[8, 9])

# Now head can move but antennas are disabled
```

## Example: Safe Motor Management

```python
from contextlib import contextmanager

@contextmanager
def motor_control(reachy_mini, motor_ids):
    """Context manager for safe motor control"""
    try:
        # Enable motors
        reachy_mini.enable_motors(ids=motor_ids)
        yield reachy_mini
    finally:
        # Disable motors when done
        reachy_mini.disable_motors(ids=motor_ids)

# Usage
with motor_control(reachy_mini, motor_ids=[1, 2, 3]):
    # Do movements
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
        duration=1.0
    )
    import time
    time.sleep(1.0)

# Motors automatically disabled
```

## Example: Emergency Stop

```python
import time

def emergency_stop(reachy_mini):
    """Immediately stop all motor control"""
    # Disable all motors
    reachy_mini.disable_motors()
    # Disable gravity compensation
    reachy_mini.disable_gravity_compensation()
    print("Emergency stop activated!")

# In case of emergency
emergency_stop(reachy_mini)
```

## Example: Motor Testing

```python
def test_motor(reachy_mini, motor_id):
    """Test if a motor is working"""
    import time

    print(f"Testing motor {motor_id}...")

    # Enable the motor
    reachy_mini.enable_motors(ids=motor_id)
    time.sleep(0.5)

    # Disable the motor
    reachy_mini.disable_motors(ids=motor_id)

    print(f"Motor {motor_id} test complete")

# Test all motors
for motor_id in range(1, 10):
    test_motor(reachy_mini, motor_id)
    time.sleep(1.0)
```

## Best Practices

### 1. Always Disable Motors When Done

```python
try:
    # Your code here
    reachy_mini.goto_target(head=target, duration=1.0)
finally:
    # Always disable motors
    reachy_mini.disable_motors()
```

### 2. Use Gravity Compensation for Holding Positions

```python
# Enable gravity compensation for stationary positions
reachy_mini.enable_gravity_compensation()
reachy_mini.goto_target(head=target, duration=1.0)

# Robot will hold position more naturally
```

### 3. Check Motor Status Before Movement

```python
# Ensure motors are enabled before movement
reachy_mini.enable_motors()

# Proceed with movement
reachy_mini.goto_target(head=target, duration=1.0)
```

## Motor Configuration (Advanced)

For advanced users, motors can be configured using the tools module:

```python
from reachy_mini.tools.setup_motor import setup_motor, change_id

# Configure motor baudrate
setup_motor(
    motor_config="path/to/config.yaml",
    serial_port="/dev/ttyUSB0",
    from_baudrate=1000000,
    target_baudrate=1000000
)

# Change motor ID
change_id(
    serial_port="/dev/ttyUSB0",
    current_id=1,
    new_id=10,
    baudrate=1000000
)
```

Warning: Motor configuration should only be done by experienced users. Incorrect settings can damage the robot.
