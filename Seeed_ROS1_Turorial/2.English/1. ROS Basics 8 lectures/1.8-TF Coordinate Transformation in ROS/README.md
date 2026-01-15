# 1.1.8-TF Coordinate Transformation in ROS
## 1. Introduction: Why Do We Need TF? (Analogy with the Human Body)
Script:
"Imagine your eyes see an apple 1 meter in front of you. How does your brain know how far to stretch your hand to reach it?

Your eyes tell your brain: The apple is 1 meter in front of the eyes.
Your brain knows: The eyes are on the head.
Your brain knows: The head is connected to the body.
Your brain knows: The hand is connected to the body.

Through this series of 'connection relationships', the brain can calculate the position of the 'hand' relative to the 'apple'. ROS TF is this 'brain' — it maintains the relative positions and angles between various components of the robot."

## 2. Core Concept: The TF Tree
Key Knowledge Points:
- **Parent/Child Relationship**: TF is a tree structure that does not allow loops. Each coordinate frame can have only one parent node but multiple child nodes.
- **Transform**: Contains **Translation (x/y/z)** and **Rotation (Quaternion/RPY)**. In other words: how much the child frame has moved and rotated relative to the parent frame.
- **Timestamp**: Robots are in motion, and coordinate relationships change over time. Therefore, each TF data is tagged with a timestamp, and the TF library automatically handles time synchronization (e.g., interpolation) for you.

## 3. The "Standard Coordinate Frame Suite" in ROS

<p align="center">
  <a>
    <img src="./images/Mermaid.png" width="600" height="auto">
  </a>
</p>


### map (Map Coordinate Frame)
- **Role**: The absolute center of the world, a god's-eye view.
- **Characteristics**: Coordinates here do not drift over time (or should not drift). If you are doing navigation, target points are usually defined in the `map` frame.
- **Publisher**: Typically published by localization algorithms (e.g., AMCL) or SLAM algorithms (e.g., Gmapping).

### odom (Odometry Coordinate Frame)
- **Role**: The path the robot *thinks* it has traveled.
- **Characteristics**: It is continuous (smooth) but has errors (will drift). For example, if the robot's wheels slip, `odom` thinks the vehicle has moved, but it actually hasn't.
- **Relationship**: The transform between `map` and `odom` represents the **"robot's localization error correction"**.

### base_link (Base Link Coordinate Frame)
- **Role**: The physical center of the robot (usually at the center of the two-wheel axle or the chassis center).
- **Relationship**: The transform between `odom` and `base_link` represents the robot's movement calculated from encoders/IMU.

### laser_link / camera_link (Sensor Coordinate Frames)
- **Role**: The mounting position of the sensor.
- **Characteristics**: Usually published via `static_transform_publisher`, because sensors are fixed to the chassis — their position relative to the chassis never changes.

## 4. Example: Projecting LiDAR Data to the Map
Scenario Description:
The LiDAR detects an obstacle ahead, and the data reads: "Obstacle at x=2.0 meters ahead".

TF Workflow:
1. LiDAR says: I (`laser_link`) see something at (2, 0, 0).
2. TF query: How far is `laser_link` from `base_link`? (Assume the LiDAR is mounted 0.5 meters in front of the vehicle head.)
3. Calculation: The obstacle is 2.0 + 0.5 = 2.5 meters from the vehicle center.
4. TF query: Where is `base_link` in the `map` frame now? (Assume the vehicle is at position (10, 10) on the map, facing east.)
5. Calculation: The obstacle is at position (12.5, 10) on the map.

Summary: As long as you have the TF tree, you can convert data between any two coordinate frames — this is what makes TF powerful.

## 5. Common Tools (Show me the code/tools)
Teach others how to debug TF issues:

### View Tree Structure (Generate PDF Diagram)
```bash
rosrun tf view_frames
evince frames.pdf
```
This is the most intuitive method — you can see the connections between frames and check if the frequency is normal.

### Real-Time Monitoring of Transform Relationships
```bash
rosrun tf tf_echo [source_frame] [target_frame]
# Example: rosrun tf tf_echo map base_link
```
This will show you the robot's current absolute coordinates.

### Rviz Visualization
1. Add the **TF display** in Rviz.
2. Check the **Show Names** option.

You will see a set of RGB coordinate axes (RGB corresponds to XYZ) on the screen, which intuitively displays the TF tree.


# 6. Let's Get Hands-On
Design a simple **"LiDAR Scanning" scenario**:
1.  **`base_link`**: The robot's base, stationary.
2.  **`radar_link`**: A LiDAR mounted on the base, which rotates.

We will write two Python scripts: one to **publish** this rotational relationship (Broadcaster), and another to **listen** and calculate coordinates (Listener).

---

## Preparation
First, ensure that a ROS workspace and package have been created (assume the package is named `learning_tf`).
```bash
# If the package does not exist yet
cd ~/catkin_ws/src
catkin_create_pkg learning_tf rospy tf geometry_msgs
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Step 1: Publish Coordinate Frames (The Broadcaster)
**Task**: Tell ROS the position of `radar_link`.
**Logic**: Update the angle of `radar_link` relative to `base_link` every 0.1 seconds.

Create a new file `tf_broadcaster.py` under `learning_tf/src/`:

```
cd ~/catkin_ws/src/learning_tf/src/
touch tf_broadcaster.py
```


```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math

if __name__ == '__main__':
    rospy.init_node('simple_tf_broadcaster')
    
    # Create a broadcaster, equivalent to a "megaphone"
    br = tf.TransformBroadcaster()
    
    rate = rospy.Rate(10) # 10Hz
    angle = 0
    
    print("Start publishing coordinate transform from radar_link to base_link...")

    while not rospy.is_shutdown():
        # 1. Calculate the quaternion for rotation (ROS uses quaternions to represent rotation. 
        # Don't worry, tf provides conversion functions)
        # Parameters: roll, pitch, yaw. Here we rotate it around the Z-axis (yaw)
        angle += 0.05
        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
        
        # 2. Publish TF
        # Parameter order:
        # (x, y, z): Translation distance. Assume the LiDAR is mounted 1.0m in front of the base and 0.5m in height
        # quaternion: Rotation angle
        # time: Timestamp, usually using current time
        # child_frame: Name of the child coordinate frame (radar)
        # parent_frame: Name of the parent coordinate frame (base)
        br.sendTransform(
            (1.0, 0.0, 0.5),          
            quaternion,               
            rospy.Time.now(),         
            "radar_link",             
            "base_link"               
        )
        rate.sleep()
```

---

## Step 2: Listen and Convert Coordinates (The Listener)
**Task**: Assume the LiDAR detects an obstacle **2 meters ahead of the LiDAR**. What is the position of this obstacle relative to the **robot's base**?
**Logic**: Listen to the TF tree, obtain the current relationship, and perform mathematical transformation.

Create a new file `tf_listener.py` under `learning_tf/src/`:

```
cd ~/catkin_ws/src/learning_tf/src/
touch tf_listener.py
```


```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import PointStamped

if __name__ == '__main__':
    rospy.init_node('simple_tf_listener')

    # Create a listener, equivalent to an "ear"
    listener = tf.TransformListener()
    
    rate = rospy.Rate(1) # 1Hz, check once per second
    
    print("Waiting for TF network to be established...")
    
    while not rospy.is_shutdown():
        try:
            # 1. This is a key step: Wait for the connection between the two coordinate frames to be established
            # Without this line, the program may report an error due to no received data when it first starts
            listener.waitForTransform("base_link", "radar_link", rospy.Time(0), rospy.Duration(3.0))
            
            # 2. Define a point: Coordinate (2.0, 0.0, 0.0) under radar_link
            # Meaning: The LiDAR detects an object 2 meters directly in front of it
            radar_point = PointStamped()
            radar_point.header.frame_id = "radar_link"
            radar_point.header.stamp = rospy.Time(0) # Get the latest available transform
            radar_point.point.x = 2.0
            radar_point.point.y = 0.0
            radar_point.point.z = 0.0
            
            # 3. The moment of truth: Perform the transform!
            # Question: What are the coordinates of this point in "base_link"?
            base_point = listener.transformPoint("base_link", radar_point)
            
            print("-------------------------------------------")
            print("Point in LiDAR frame: (%.2f, %.2f, %.2f)" % (radar_point.point.x, radar_point.point.y, radar_point.point.z))
            print("Point in base frame: (%.2f, %.2f, %.2f)" % (base_point.point.x, base_point.point.y, base_point.point.z))
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("TF transform failed")
            continue

        rate.sleep()
```

---

## Step 3: Hands-On Execution and Visualization (The Most Important Step)

Grant execution permissions to the scripts:
```bash
chmod +x src/tf_broadcaster.py src/tf_listener.py
```

Add the following code to the `CMakeLists.txt` file in the parent directory:
```cmake
catkin_install_python(PROGRAMS
  src/tf_broadcaster.py
  src/tf_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Compile the workspace:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 1. Launch Terminal A (Core)
```bash
roscore
```

### 2. Launch Terminal B (Broadcaster)
```bash
rosrun learning_tf tf_broadcaster.py
```
*At this point, you have a virtual rotating LiDAR running on your computer.*

### 3. Launch Terminal C (Rviz Visualization)
```bash
rosrun rviz rviz
```
**Rviz Configuration (Step-by-Step Guide)**:
1.  In the left **Displays** panel, go to `Global Options` -> set `Fixed Frame` to **`base_link`** (manually type it if it's not listed).
2.  Click the **Add** button at the bottom left -> select **`TF`**.
3.  You will see two coordinate axes in the center of the screen, where one (`radar_link`) is **rotating** around the other (`base_link`)!
4.  Check the `Show Names` option in the TF properties to clearly view the frame names.

<p align="center">
  <a>
    <img src="./images/rviz_result.png" width="600" height="auto">
  </a>
</p>



### 4. Launch Terminal D (Listener)
```bash
rosrun learning_tf tf_listener.py
```
**Observe the Output**:
You will notice that although the input is always `(2.0, 0, 0)` (relative to the LiDAR), the output coordinates relative to the base keep changing!
-   When the LiDAR rotates to the front of the base, the base coordinate may be `(3.0, 0, 0.5)` (1m mounting offset + 2m LiDAR detection distance).
-   When the LiDAR rotates to the left of the base, the Y-value of the base coordinate will become significantly larger.

### 5. Check with Command-Line Tools (Debugging Essentials)
Teach them these two commands—they are indispensable for troubleshooting later:

*   **View TF Tree Structure**:
    ```bash
    rosrun tf view_frames
    evince frames.pdf
    ```
    *Explanation: This proves that `base_link` is the parent frame and `radar_link` is the child frame.*

*   **Print the Relationship Between Two Frames in Real Time**:
    ```bash
    rosrun tf tf_echo base_link radar_link
    ```
    *Explanation: You will see that the Translation (1, 0, 0.5) is basically unchanged, but the Rotation is fluctuating rapidly.*

<p align="center">
  <a>
    <img src="./images/tf_result.png" width="600" height="auto">
  </a>
</p>

---

# Summary
> **"The TF system in ROS is the robot's 'spatial perception nerve'. It unifies scattered, independent sensor data (eyes, ears) into the same world coordinate frame (brain) through mathematical transformation, allowing the robot to know 'where I am' and 'where obstacles are'."**

