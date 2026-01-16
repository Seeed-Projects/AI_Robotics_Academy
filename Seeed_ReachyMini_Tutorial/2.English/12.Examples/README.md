# Complete Examples - Reachy Mini SDK

## Overview

This section provides complete, ready-to-run examples that demonstrate various capabilities of Reachy Mini.

## Example 1: Hello Reachy

```python
#!/usr/bin/env python3
"""
Basic "Hello World" example for Reachy Mini
"""

from reachy_mini import ReachyMini
import time

def main():
    # Connect to robot
    print("Connecting to Reachy Mini...")
    reachy_mini = ReachyMini()

    if not reachy_mini.io_client.is_connected():
        print("Failed to connect!")
        return

    print("Connected successfully!")

    # Perform wake up
    print("Waking up...")
    reachy_mini.wake_up()
    time.sleep(2.0)

    # Simple movements
    print("Moving head...")
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
        duration=1.0
    )
    time.sleep(1.5)

    # Move antennas
    print("Moving antennas...")
    reachy_mini.goto_target(
        antennas=(0.7, 0.3),
        duration=0.5
    )
    time.sleep(1.0)

    # Go to sleep
    print("Going to sleep...")
    reachy_mini.goto_sleep()
    time.sleep(2.0)

    # Disconnect
    print("Disconnecting...")
    reachy_mini.io_client.disconnect()
    print("Done!")

if __name__ == "__main__":
    main()
```

## Example 2: Face Tracking

```python
#!/usr/bin/env python3
"""
Track faces using the camera and move head to follow
"""

from reachy_mini import ReachyMini
import cv2
import time

class FaceTracker:
    def __init__(self):
        # Connect to robot
        self.reachy_mini = ReachyMini()

        # Load face detector
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

        # Tracking state
        self.tracking = True
        self.last_look_time = 0
        self.min_look_interval = 0.1  # 100ms between looks

    def get_face_center(self, frame):
        """Get center of detected face"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
        )

        if len(faces) > 0:
            # Get largest face
            face = max(faces, key=lambda f: f[2] * f[3])
            x, y, w, h = face
            return (x + w // 2, y + h // 2), face

        return None, None

    def track_face(self):
        """Main tracking loop"""
        print("Starting face tracking... Press 'q' to quit")

        try:
            while self.tracking:
                # Get camera frame
                frame = self.reachy_mini.media.get_frame()

                if frame is None:
                    continue

                # Find face
                center, face_rect = self.get_face_center(frame)

                if center:
                    # Check if enough time has passed
                    current_time = time.time()
                    if current_time - self.last_look_time >= self.min_look_interval:
                        # Look at face
                        u, v = center
                        self.reachy_mini.look_at_image(u=u, v=v, duration=0.2)
                        self.last_look_time = current_time

                    # Draw rectangle
                    if face_rect is not None:
                        x, y, w, h = face_rect
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # Display frame
                cv2.imshow('Face Tracking', frame)

                # Check for quit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        self.tracking = False
        cv2.destroyAllWindows()
        self.reachy_mini.io_client.disconnect()
        print("Tracking stopped")

if __name__ == "__main__":
    tracker = FaceTracker()
    tracker.track_face()
```

## Example 3: Sound Reactive

```python
#!/usr/bin/env python3
"""
React to sound by turning to look at sound source
"""

from reachy_mini import ReachyMini
import numpy as np
import time

class SoundReactive:
    def __init__(self, threshold=0.1):
        self.reachy_mini = ReachyMini()
        self.threshold = threshold
        self.running = True

    def get_audio_level(self, audio_sample):
        """Calculate audio level from sample"""
        if audio_sample is None:
            return 0.0
        return np.sqrt(np.mean(audio_sample**2))

    def react_to_sound(self):
        """Main reactivity loop"""
        print("Starting sound reactivity... Press Ctrl+C to stop")

        # Start recording
        self.reachy_mini.media.start_recording()

        try:
            while self.running:
                # Get audio sample
                sample = self.reachy_mini.media.get_audio_sample()

                if sample is not None:
                    level = self.get_audio_level(sample)

                    # React if above threshold
                    if level > self.threshold:
                        # Get direction of arrival
                        doa = self.reachy_mini.media.get_doa()

                        if doa is not None:
                            print(f"Sound detected! Level: {level:.3f}, DOA: {doa:.2f}")

                            # Turn to sound
                            self.reachy_mini.goto_target(
                                body_yaw=doa,
                                duration=0.3
                            )

                            # Perk antennas
                            self.reachy_mini.set_target_antenna_joint_positions(
                                antennas=[0.8, 0.8]
                            )

                        time.sleep(0.5)

                        # Return antennas
                        self.reachy_mini.set_target_antenna_joint_positions(
                            antennas=[0.5, 0.5]
                        )

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nStopping...")

        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        self.running = False
        self.reachy_mini.media.stop_recording()
        self.reachy_mini.io_client.disconnect()
        print("Sound reactivity stopped")

if __name__ == "__main__":
    reactor = SoundReactive(threshold=0.15)
    reactor.react_to_sound()
```

## Example 4: Gesture Sequence

```python
#!/usr/bin/env python3
"""
Perform a sequence of expressive gestures
"""

from reachy_mini import ReachyMini
import time

class GestureController:
    def __init__(self):
        self.reachy_mini = ReachyMini()

    def nod(self, count=3):
        """Nod head"""
        for _ in range(count):
            self.reachy_mini.goto_target(
                head=(0.0, 0.0, 0.05, 0.0, 0.2, 0.0),
                duration=0.3,
                method="EASE_IN_OUT"
            )
            time.sleep(0.35)

            self.reachy_mini.goto_target(
                head=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                duration=0.3,
                method="EASE_IN_OUT"
            )
            time.sleep(0.35)

    def shake(self, count=3):
        """Shake head"""
        for i in range(count):
            direction = 1 if i % 2 == 0 else -1

            self.reachy_mini.goto_target(
                head=(0.0, direction * 0.05, 0.05, 0.0, 0.0, direction * 0.3),
                duration=0.3,
                method="EASE_IN_OUT"
            )
            time.sleep(0.35)

        # Return to center
        self.reachy_mini.goto_target(
            head=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            duration=0.3
        )
        time.sleep(0.35)

    def look_around(self):
        """Look around"""
        positions = [
            (0.0, -0.1, 0.1, 0.0, 0.0, -0.4),  # Left
            (0.0, 0.0, 0.15, 0.0, -0.2, 0.0),  # Up
            (0.0, 0.1, 0.1, 0.0, 0.0, 0.4),    # Right
            (0.0, 0.0, 0.05, 0.0, 0.1, 0.0),   # Down
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),    # Center
        ]

        for pos in positions:
            self.reachy_mini.goto_target(
                head=pos,
                duration=0.8,
                method="MIN_JERK"
            )
            time.sleep(0.9)

    def antenna_wave(self, count=3):
        """Wave antennas"""
        for _ in range(count):
            self.reachy_mini.goto_target(
                antennas=[0.8, 0.3],
                duration=0.2
            )
            time.sleep(0.25)

            self.reachy_mini.goto_target(
                antennas=[0.3, 0.8],
                duration=0.2
            )
            time.sleep(0.25)

        # Return to center
        self.reachy_mini.goto_target(
            antennas=[0.5, 0.5],
            duration=0.3
        )
        time.sleep(0.35)

    def agree(self):
        """Agree gesture (nod + antennas happy)"""
        self.reachy_mini.goto_target(
            antennas=[0.7, 0.7],
            duration=0.3
        )
        time.sleep(0.35)
        self.nod(count=2)

    def disagree(self):
        """Disagree gesture (shake + antennas down)"""
        self.reachy_mini.goto_target(
            antennas=[0.3, 0.3],
            duration=0.3
        )
        time.sleep(0.35)
        self.shake(count=2)

    def curious(self):
        """Curious gesture"""
        # Look left with antenna
        self.reachy_mini.goto_target(
            head=(0.0, -0.1, 0.15, 0.0, -0.1, -0.4),
            antennas=[0.8, 0.4],
            duration=1.0,
            method="MIN_JERK"
        )
        time.sleep(1.5)

        # Look right
        self.reachy_mini.goto_target(
            head=(0.0, 0.1, 0.15, 0.0, -0.1, 0.4),
            antennas=[0.4, 0.8],
            duration=1.0,
            method="MIN_JERK"
        )
        time.sleep(1.5)

        # Return to center
        self.reachy_mini.goto_target(
            head=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            antennas=[0.5, 0.5],
            duration=0.8
        )
        time.sleep(1.0)

    def cleanup(self):
        """Clean up"""
        self.reachy_mini.goto_sleep()
        time.sleep(2.0)
        self.reachy_mini.io_client.disconnect()

def main():
    controller = GestureController()

    try:
        # Wake up
        print("Waking up...")
        controller.reachy_mini.wake_up()
        time.sleep(2.0)

        # Perform gestures
        print("Nodding...")
        controller.nod(count=3)

        print("Shaking...")
        controller.shake(count=3)

        print("Looking around...")
        controller.look_around()

        print("Waving antennas...")
        controller.antenna_wave(count=3)

        print("Showing agreement...")
        controller.agree()

        print("Showing disagreement...")
        controller.disagree()

        print("Being curious...")
        controller.curious()

        print("Going to sleep...")
        controller.cleanup()

    except Exception as e:
        print(f"Error: {e}")
        controller.cleanup()

if __name__ == "__main__":
    main()
```

## Example 5: Record and Playback

```python
#!/usr/bin/env python3
"""
Record a movement and play it back
"""

from reachy_mini import ReachyMini
import json
import time

def record_movement():
    """Record a movement sequence"""
    reachy_mini = ReachyMini()

    print("=== Recording Mode ===")
    print("Get ready to record...")

    # Wake up
    reachy_mini.wake_up()
    time.sleep(2.0)

    input("Press Enter to start recording...")
    print("Recording started! Move the robot manually or use goto_target.")
    input("Press Enter to stop recording...")

    # Stop recording
    data = reachy_mini.stop_recording()

    if data:
        print(f"Recorded {len(data)} frames")

        # Save to file
        filename = "recorded_movement.json"
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"Saved to {filename}")

    # Go to sleep
    reachy_mini.goto_sleep()
    time.sleep(2.0)
    reachy_mini.io_client.disconnect()

def play_back_movement(filename="recorded_movement.json"):
    """Play back a recorded movement"""
    reachy_mini = ReachyMini()

    print("=== Playback Mode ===")

    # Load recording
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"Recording file '{filename}' not found!")
        return

    print(f"Loaded {len(data)} frames")
    print("Playing back in 3 seconds...")
    time.sleep(3.0)

    # Playback
    start_time = time.time()

    for i, frame in enumerate(data):
        # Calculate timing
        frame_time = frame['timestamp']
        elapsed = time.time() - start_time

        if elapsed < frame_time:
            time.sleep(frame_time - elapsed)

        # Extract and set positions
        antenna_joints = frame.get('antenna_joints')
        if antenna_joints:
            reachy_mini.set_target_antenna_joint_positions(
                antennas=antenna_joints
            )

        # Progress
        print(f"\rFrame {i+1}/{len(data)}", end='')

    print("\nPlayback complete!")

    # Go to sleep
    time.sleep(1.0)
    reachy_mini.goto_sleep()
    time.sleep(2.0)
    reachy_mini.io_client.disconnect()

def main():
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "play":
        # Playback mode
        filename = sys.argv[2] if len(sys.argv) > 2 else "recorded_movement.json"
        play_back_movement(filename)
    else:
        # Record mode
        record_movement()

if __name__ == "__main__":
    main()
```

## Running the Examples

```bash
# Hello Reachy
python 01_hello_reachy.py

# Face tracking (requires camera)
python 02_face_tracking.py

# Sound reactivity (requires microphone)
python 03_sound_reactive.py

# Gesture sequence
python 04_gesture_sequence.py

# Record movement
python 05_record_playback.py

# Play back recording
python 05_record_playback.py play recorded_movement.json
```
