# Troubleshooting - Reachy Mini SDK

## Connection Issues

### Robot Not Found

**Problem:** Cannot connect to Reachy Mini

```python
reachy_mini = ReachyMini()
# Timeout or connection error
```

**Solutions:**

1. Check robot is powered on
2. Verify network connection
3. Try specific connection mode:

```python
# Try localhost only
reachy_mini = ReachyMini(connection_mode="localhost_only")

# Or try network only
reachy_mini = ReachyMini(connection_mode="network")

# Increase timeout
reachy_mini = ReachyMini(timeout=10.0)
```

4. Check if daemon is running:

```python
# Check status
status = reachy_mini.io_client.get_status()
print(status)
```

### Connection Drops

**Problem:** Robot disconnects randomly

**Solutions:**

1. Use wired connection instead of WiFi
2. Check network stability
3. Implement reconnection logic:

```python
def safe_connect(max_retries=3):
    for i in range(max_retries):
        try:
            reachy_mini = ReachyMini(timeout=10.0)
            if reachy_mini.io_client.is_connected():
                return reachy_mini
        except Exception as e:
            print(f"Attempt {i+1} failed: {e}")
            time.sleep(2.0)

    raise ConnectionError("Failed to connect after retries")
```

## Movement Issues

### Motors Not Moving

**Problem:** Commands sent but no movement

**Solutions:**

1. Check if motors are enabled:

```python
# Enable motors
reachy_mini.enable_motors()
```

2. Check for errors:

```python
# Get status
status = reachy_mini.io_client.get_status()
```

3. Try gravity compensation:

```python
reachy_mini.enable_gravity_compensation()
```

### Jerky Movements

**Problem:** Movements are not smooth

**Solutions:**

1. Use minimum jerk interpolation:

```python
reachy_mini.goto_target(
    head=target,
    duration=1.0,
    method="MIN_JERK"
)
```

2. Increase duration:

```python
# Slower = smoother
duration = 2.0
```

3. Reduce frequency (for async):

```python
await reachy_mini.async_play_move(
    move,
    play_frequency=50  # Lower frequency
)
```

### Out of Range

**Problem:** Target position is unreachable

**Solutions:**

1. Check workspace limits:

```python
# Use more conservative targets
target = (0.0, 0.0, 0.1, 0.0, 0.0, 0.0)  # Stay within range
```

2. Enable automatic body yaw:

```python
reachy_mini.set_automatic_body_yaw(True)
```

3. Test reachable positions:

```python
# Try smaller movements
for offset in [0.05, 0.1, 0.15]:
    try:
        reachy_mini.goto_target(
            head=(0.0, 0.0, offset, 0.0, 0.0, 0.0),
            duration=1.0
        )
        print(f"Offset {offset} OK")
    except:
        print(f"Offset {offset} failed")
```

## Media Issues

### Camera Not Working

**Problem:** Camera returns None or errors

**Solutions:**

1. Check media backend:

```python
# Try different backend
reachy_mini = ReachyMini(media_backend="DEFAULT")
```

2. Check camera is connected:

```python
frame = reachy_mini.media.get_frame()
if frame is None:
    print("Camera not available")
```

3. Check camera specs:

```python
specs = reachy_mini.media.camera.get_camera_specs()
print(specs)
```

### Audio Not Working

**Problem:** Cannot play or record sound

**Solutions:**

1. Check audio backend:

```python
# Audio only (no camera)
reachy_mini = ReachyMini(media_backend="DEFAULT_NO_VIDEO")
```

2. Test audio device:

```python
# Try playing
reachy_mini.media.play_sound("test.wav")
```

3. Check sample rate:

```python
sample_rate = reachy_mini.media.audio.get_input_audio_samplerate()
print(f"Sample rate: {sample_rate}")
```

## IMU Issues

### IMU Data Not Available

**Problem:** `reachy_mini.imu` returns None

**Solutions:**

1. Check if IMU is configured:

```python
imu_data = reachy_mini.imu

if imu_data is None:
    print("IMU not available on this configuration")
else:
    print("IMU working!")
```

2. Some configurations may not have IMU

## Performance Issues

### Slow Response

**Problem:** Commands take too long to execute

**Solutions:**

1. Reduce interpolation complexity:

```python
# Use LINEAR instead of MIN_JERK for faster response
reachy_mini.goto_target(
    head=target,
    duration=0.3,
    method="LINEAR"
)
```

2. Use async operations:

```python
# Non-blocking movement
await reachy_mini.async_play_move(move, play_frequency=100)
```

3. Optimize frequency:

```python
# Lower frequency for simple movements
play_frequency = 50  # Instead of 100
```

### High CPU Usage

**Problem:** SDK uses too much CPU

**Solutions:**

1. Reduce polling frequency:

```python
# Don't poll too fast
while True:
    frame = reachy_mini.media.get_frame()
    time.sleep(0.1)  # 10 FPS instead of 30+
```

2. Use async properly:

```python
# Don't create too many concurrent tasks
# Limit to 1-2 background movements
```

## Debugging Tips

### Enable Debug Logging

```python
# Enable debug logging
reachy_mini = ReachyMini(log_level="DEBUG")
```

### Check Status Regularly

```python
# Monitor status
def check_status(reachy_mini):
    status = reachy_mini.io_client.get_status()
    connected = reachy_mini.io_client.is_connected()

    print(f"Connected: {connected}")
    print(f"Status: {status}")

    # Check IMU
    imu = reachy_mini.imu
    if imu:
        print(f"IMU temp: {imu['temperature']}°C")
```

### Test Individual Components

```python
# Test motors
reachy_mini.enable_motors()
time.sleep(0.5)
reachy_mini.disable_motors()

# Test movement
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.05, 0.0, 0.0, 0.0),
    duration=0.5
)

# Test camera
frame = reachy_mini.media.get_frame()
print(f"Frame: {frame.shape if frame is not None else 'None'}")

# Test audio
reachy_mini.media.play_sound("test.wav")
```

## Getting Help

If issues persist:

1. Check SDK version:

```python
import reachy_mini
print(reachy_mini.__version__)
```

2. Check system requirements:

```bash
python --version  # Should be 3.8+
pip list | grep reachy
```

3. Enable verbose logging and capture output

4. Report issues with:
   - SDK version
   - Python version
   - Operating system
   - Error messages
   - Minimal reproduction code
