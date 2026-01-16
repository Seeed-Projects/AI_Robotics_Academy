# IMU (Inertial Measurement Unit) - Reachy Mini SDK

## Overview

Reachy Mini includes an IMU sensor that provides accelerometer, gyroscope, quaternion (orientation), and temperature data. This allows the robot to sense its own movement and orientation.

## Access IMU Data

### Get All IMU Data

```python
# Get IMU data
imu_data = reachy_mini.imu

if imu_data is not None:
    print(f"Accelerometer: {imu_data['accelerometer']}")
    print(f"Gyroscope: {imu_data['gyroscope']}")
    print(f"Quaternion: {imu_data['quaternion']}")
    print(f"Temperature: {imu_data['temperature']} °C")
else:
    print("IMU data not available")
```

## IMU Data Structure

```python
{
    'accelerometer': [x, y, z],  # m/s²
    'gyroscope': [x, y, z],      # rad/s
    'quaternion': [w, x, y, z],  # orientation quaternion
    'temperature': float          # °C
}
```

## Accelerometer

Measures proper acceleration (gravity + movement acceleration).

### Get Accelerometer Data

```python
imu_data = reachy_mini.imu

if imu_data:
    ax, ay, az = imu_data['accelerometer']
    print(f"X acceleration: {ax:.2f} m/s²")
    print(f"Y acceleration: {ay:.2f} m/s²")
    print(f"Z acceleration: {az:.2f} m/s²")
```

### Detect Movement

```python
def is_moving(imu_data, threshold=0.1):
    """Check if robot is moving based on accelerometer"""
    if imu_data is None:
        return False

    ax, ay, az = imu_data['accelerometer']

    # Calculate magnitude excluding gravity
    magnitude = (ax**2 + ay**2 + (az - 9.81)**2)**0.5

    return magnitude > threshold

# Usage
imu_data = reachy_mini.imu
if is_moving(imu_data):
    print("Robot is moving!")
else:
    print("Robot is stationary")
```

## Gyroscope

Measures angular velocity (rate of rotation).

### Get Gyroscope Data

```python
imu_data = reachy_mini.imu

if imu_data:
    gx, gy, gz = imu_data['gyroscope']
    print(f"X angular velocity: {gx:.4f} rad/s")
    print(f"Y angular velocity: {gy:.4f} rad/s")
    print(f"Z angular velocity: {gz:.4f} rad/s")
```

### Calculate Rotation

```python
import time

def track_rotation(reachy_mini, duration=5.0):
    """Track total rotation over time"""
    start_time = time.time()

    total_rotation = [0.0, 0.0, 0.0]  # [roll, pitch, yaw]

    while time.time() - start_time < duration:
        imu_data = reachy_mini.imu

        if imu_data:
            gx, gy, gz = imu_data['gyroscope']

            # Integrate angular velocity
            dt = 0.01  # 100 Hz
            total_rotation[0] += gx * dt
            total_rotation[1] += gy * dt
            total_rotation[2] += gz * dt

        time.sleep(dt)

    return total_rotation

# Usage
rotation = track_rotation(reachy_mini, duration=2.0)
print(f"Total rotation: {rotation}")
```

## Quaternion (Orientation)

Represents the robot's orientation in 3D space.

### Get Orientation

```python
imu_data = reachy_mini.imu

if imu_data:
    w, x, y, z = imu_data['quaternion']
    print(f"Quaternion: [{w:.4f}, {x:.4f}, {y:.4f}, {z:.4f}]")
```

### Convert Quaternion to Euler Angles

```python
import math

def quaternion_to_euler(w, x, y, z):
    """Convert quaternion to Euler angles (roll, pitch, yaw)"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# Usage
imu_data = reachy_mini.imu

if imu_data:
    w, x, y, z = imu_data['quaternion']
    roll, pitch, yaw = quaternion_to_euler(w, x, y, z)

    print(f"Roll: {math.degrees(roll):.2f}°")
    print(f"Pitch: {math.degrees(pitch):.2f}°")
    print(f"Yaw: {math.degrees(yaw):.2f}°")
```

## Temperature

Measures the IMU sensor temperature.

### Get Temperature

```python
imu_data = reachy_mini.imu

if imu_data:
    temp = imu_data['temperature']
    print(f"IMU temperature: {temp:.2f} °C")
```

## Example: Detect Tap/Shake

```python
import time

def detect_shake(imu_data, threshold=2.0):
    """Detect if robot is being shaken"""
    if imu_data is None:
        return False

    ax, ay, az = imu_data['accelerometer']

    # Calculate magnitude
    magnitude = (ax**2 + ay**2 + az**2)**0.5

    return magnitude > threshold

# Monitor for shake events
print("Monitoring for shake events... Press Ctrl+C to stop")

try:
    while True:
        imu_data = reachy_mini.imu

        if detect_shake(imu_data):
            print("Shake detected!")
            # React to shake
            reachy_mini.goto_target(
                antennas=[0.8, 0.8],  # Antennas up
                duration=0.2
            )

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped monitoring")
```

## Example: Orientation Monitoring

```python
import time
import math

def quaternion_to_euler(w, x, y, z):
    """Convert quaternion to Euler angles"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# Monitor orientation
print("Monitoring orientation... Press Ctrl+C to stop")

try:
    while True:
        imu_data = reachy_mini.imu

        if imu_data:
            w, x, y, z = imu_data['quaternion']
            roll, pitch, yaw = quaternion_to_euler(w, x, y, z)

            print(f"\rRoll: {math.degrees(roll):6.2f}° | "
                  f"Pitch: {math.degrees(pitch):6.2f}° | "
                  f"Yaw: {math.degrees(yaw):6.2f}°", end='')

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopped monitoring")
```

## Example: Freefall Detection

```python
def detect_freefall(imu_data, threshold=0.5):
    """Detect if robot is in freefall"""
    if imu_data is None:
        return False

    ax, ay, az = imu_data['accelerometer']

    # In freefall, all axes should read near zero
    # (gravity not felt)
    magnitude = (ax**2 + ay**2 + az**2)**0.5

    return magnitude < threshold

# Monitor for freefall
print("Monitoring for freefall...")

try:
    while True:
        imu_data = reachy_mini.imu

        if detect_freefall(imu_data):
            print("Freefall detected! Prepare for landing!")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped monitoring")
```

## IMU Availability

Note: IMU data may not be available on all Reachy Mini configurations. Always check if `imu_data` is not None before accessing.

```python
imu_data = reachy_mini.imu

if imu_data is not None:
    # IMU is available
    print("IMU data available")
else:
    # IMU is not available
    print("IMU data not available on this configuration")
```
