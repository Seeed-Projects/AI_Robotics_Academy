# Media - Reachy Mini SDK

## Overview

Reachy Mini's media system provides access to the camera and audio devices. The MediaManager offers a unified interface for capturing images, playing sounds, and recording audio.

## Media Backends

The SDK supports multiple backends for different use cases:

| Backend | Description | Camera | Audio |
|---------|-------------|--------|-------|
| `NO_MEDIA` | No media devices | ❌ | ❌ |
| `DEFAULT` | OpenCV + SoundDevice | ✅ | ✅ |
| `DEFAULT_NO_VIDEO` | SoundDevice only | ❌ | ✅ |
| `GSTREAMER` | GStreamer backend | ✅ | ✅ |
| `GSTREAMER_NO_VIDEO` | GStreamer audio only | ❌ | ✅ |
| `WEBRTC` | WebRTC for streaming | ✅ | ✅ |

### Initialize with Media Backend

```python
from reachy_mini import ReachyMini

# Default backend (OpenCV + SoundDevice)
reachy_mini = ReachyMini(media_backend="default")

# No media
reachy_mini = ReachyMini(media_backend="NO_MEDIA")

# Audio only
reachy_mini = ReachyMini(media_backend="DEFAULT_NO_VIDEO")
```

## Camera

### Capture a Frame

```python
# Get a single frame from the camera
frame = reachy_mini.media.get_frame()

if frame is not None:
    print(f"Frame shape: {frame.shape}")  # (height, width, channels)
    print(f"Frame dtype: {frame.dtype}")  # uint8
```

### Continuous Capture

```python
import cv2
import time

# Capture and display frames
for _ in range(100):  # Capture 100 frames
    frame = reachy_mini.media.get_frame()

    if frame is not None:
        cv2.imshow('Reachy Mini Camera', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    time.sleep(0.033)  # ~30 FPS

cv2.destroyAllWindows()
```

### Set Camera Resolution

```python
# Set camera resolution
reachy_mini.media.camera.set_resolution((640, 480))

# Get current resolution
width, height = reachy_mini.media.camera.get_resolution()
print(f"Resolution: {width}x{height}")
```

### Get Camera Specifications

```python
# Get camera specs
specs = reachy_mini.media.camera.get_camera_specs()

if specs:
    print(f"Resolution: {specs.resolution}")
    print(f"FPS: {specs.fps}")
    print(f"Field of view: {specs.fov}")
```

## Audio

### Play Sound

```python
# Play a sound file
reachy_mini.media.play_sound("path/to/sound.wav")

# Play is non-blocking
import time
time.sleep(2.0)  # Wait for sound to finish
```

### Stop Playback

```python
# Stop currently playing sound
reachy_mini.media.stop_playing()
```

### Record Audio

```python
# Start recording
reachy_mini.media.start_recording()

# Record for 3 seconds
import time
time.sleep(3.0)

# Stop recording and get audio data
audio_data = reachy_mini.media.stop_recording()

if audio_data is not None:
    print(f"Audio shape: {audio_data.shape}")
    print(f"Audio dtype: {audio_data.dtype}")  # float32
```

### Get Audio Sample

```python
# Start recording
reachy_mini.media.start_recording()

# Get audio samples in real-time
for _ in range(100):  # Get 100 samples
    sample = reachy_mini.media.get_audio_sample()

    if sample is not None:
        print(f"Sample shape: {sample.shape}")
        # Process audio sample here

    time.sleep(0.01)  # 100 Hz

# Stop recording
reachy_mini.media.stop_recording()
```

### Audio Properties

```python
# Get input sample rate
sample_rate = reachy_mini.media.audio.get_input_audio_samplerate()
print(f"Sample rate: {sample_rate} Hz")

# Get input channels
channels = reachy_mini.media.audio.get_input_channels()
print(f"Channels: {channels}")

# Get output channels
output_channels = reachy_mini.media.audio.get_output_channels()
print(f"Output channels: {output_channels}")
```

## Direction of Arrival (DOA)

If using a ReSpeaker microphone array, you can get the direction of sound sources:

```python
# Get direction of arrival
doa = reachy_mini.media.get_doa()

if doa is not None:
    print(f"Sound direction: {doa} radians")
    # Use this to make the robot look at the sound source
    reachy_mini.look_at_world(
        x=0.5, y=0.0, z=0.2,
        duration=0.5
    )
```

## Example: Look at Face in Camera

```python
import cv2

# Capture frame
frame = reachy_mini.media.get_frame()

if frame is not None:
    # Detect faces (using Haar cascade or other detector)
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    if len(faces) > 0:
        # Get first face
        x, y, w, h = faces[0]

        # Calculate face center in image coordinates
        u = x + w // 2
        v = y + h // 2

        # Look at the face
        reachy_mini.look_at_image(u=u, v=v, duration=0.5)
```

## Example: Sound Reactivity

```python
import numpy as np

def get_audio_level(audio_sample):
    """Calculate audio level from sample"""
    return np.sqrt(np.mean(audio_sample**2))

# Start recording
reachy_mini.media.start_recording()

threshold = 0.1  # Adjust based on your environment

try:
    while True:
        # Get audio sample
        sample = reachy_mini.media.get_audio_sample()

        if sample is not None:
            # Calculate audio level
            level = get_audio_level(sample)

            # React to sound
            if level > threshold:
                # Look in direction of sound
                doa = reachy_mini.media.get_doa()
                if doa is not None:
                    # Convert DOA to look direction
                    reachy_mini.goto_target(
                        body_yaw=doa,
                        duration=0.2
                    )

except KeyboardInterrupt:
    # Stop recording
    reachy_mini.media.stop_recording()
```

## Example: Save Camera Frame

```python
import cv2
from datetime import datetime

# Capture frame
frame = reachy_mini.media.get_frame()

if frame is not None:
    # Generate filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"reachy_mini_{timestamp}.jpg"

    # Save frame
    cv2.imwrite(filename, frame)
    print(f"Saved frame to {filename}")
```

## Cleanup

```python
# Always close media when done
reachy_mini.media.close()

# This is also called automatically when disconnecting
reachy_mini.io_client.disconnect()
```

## Media Manager Properties

```python
# Access camera directly
camera = reachy_mini.media.camera
frame = camera.read()

# Access audio directly
audio = reachy_mini.media.audio
audio.play_sound("file.wav")
```
