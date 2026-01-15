# Getting Started with URDF

First, prepare the workspace:
```bash
cd ~/catkin_ws/src
catkin_create_pkg robot_modeling urdf xacro geometry_msgs sensor_msgs
mkdir -p robot_modeling/urdf
mkdir -p robot_modeling/launch
mkdir -p robot_modeling/rviz
```

## 1.1 What is URDF?
URDF (Unified Robot Description Format) is the standard format for ROS. Like **HTML**, it is static.

**The Core Analogy:**
If a robot were a human, URDF would be the robot's "anatomical chart" or "DNA."
*   **Link:** Corresponds to human bones and flesh (e.g., upper arm, forearm, palm). It is a rigid body and does not deform.
*   **Joint:** Corresponds to human joints like the elbow or wrist. It connects two Links and defines how they move relative to each other.

**The Tree Structure of URDF:**
A robot model must be a **Tree Structure**.
*   There is one root node (Root Link).
*   There can be no closed loops.
*   Clear parent-child relationships: A Parent can have multiple Children, but a Child can only have one Parent.

### A. The "Trinity" of a Link
A complete Link definition contains three parts, all of which are indispensable (especially for Gazebo simulation):

1.  **Visual:** For **humans** to see. It determines whether you see a square or a circle in RViz, and whether it's red or blue.
2.  **Collision:** For the **physics engine** to see. It determines if the part will pass through walls or fall through the floor. To improve calculation speed, collision models are usually simpler than visual models (e.g., a complex robotic arm might be visually detailed but represented by a few simple cylinders for collision).
3.  **Inertial:** Used by the **physics engine** for mechanical calculations. It includes mass and the Inertia Matrix.
    *   *Teaching Tip: If you don't include Inertial, the model will behave like a ghost in Gazebo or be ignored by the system entirely.*

### B. Joint Types
*   **Fixed:** Like welding; completely immobile (e.g., a LiDAR fixed to a car roof).
*   **Continuous:** Can rotate infinitely (e.g., wheels of a robot).
*   **Revolute:** Rotation with angular limits (e.g., robotic arm joints with a range from -90° to +90°).
*   **Prismatic:** Linear sliding (e.g., drawers, elevators).

## 1.2 Basic URDF Syntax

-  Root Tag: `<robot>`

Every URDF file must follow standard XML specifications. Just as an HTML file starts with `<html>`, a URDF file must contain a root tag.

*   **Function**: Defines the scope of the robot. All descriptions regarding the robot (links, joints, transmissions, etc.) must be written inside this tag.
*   **Core Attribute**:
    *   `name`: Give your robot a name (Required attribute).

**Code Example:**
```xml
<robot name="my_first_robot">
    <!-- All robot component definitions go here -->
</robot>
```

### 1.2.1 Rigid Body Description: `<link>`

- The `<link>` tag represents a **rigid body component** of the robot.
*   **Analogy**: If you were building with LEGOs, a `<link>` is a LEGO brick (e.g., chassis, wheel, a section of a robotic arm, LiDAR).
*   **Feature**: It is rigid and cannot be deformed.

- `<visual>`: Putting "skin" on the robot.
Inside the `<link>` tag, the most important sub-tag is `<visual>`. It determines **what the robot looks like** in RViz.
*   **Note**: `<visual>` data is only for display and is not used in physics engine collision calculations.

- `<geometry>` (Shape)
This is the core of `<visual>`, used to define the geometric shape. URDF supports the following basic shapes:

| Shape Tag | Attribute Description | Example |
| :--- | :--- | :--- |
| **box** | `size="Length Width Height"` (x y z) | `<box size="0.5 0.3 0.1" />` |
| **cylinder** | `radius="Radius" length="Height"` | `<cylinder radius="0.1" length="0.2" />` |
| **sphere** | `radius="Radius"` | `<sphere radius="0.1" />` |
| **mesh** | `filename="Path"` (loads skin) | `filename="package://pkg_name/path/xxx.dae"` |

- `<origin>` (Pose Offset)
Determines the position of the **center of the shape** relative to the **center of the link coordinate system**.
*   `xyz`: Displacement on the x, y, and z axes (meters).
*   `rpy`: Rotation in radians around the x, y, and z axes (Roll, Pitch, Yaw).
    *   *Note: 3.14 radians ≈ 180 degrees.*

-  `<material>` (Color/Texture)
Colors the shape.
*   `rgba`: Red (R), Green (G), Blue (B), Alpha (A). Values range from **0 to 1**.
    *   Example: `1 0 0 1` is pure red and opaque.

#### Example
```xml
<link name="base_link">
    <visual>
        <!-- 1. Shape: A box with Length 0.5, Width 0.2, Height 0.1 -->
        <geometry>
            <box size="0.5 0.2 0.1"/>
        </geometry>
        <!-- 2. Offset: Stays in place, no rotation -->
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <!-- 3. Material: Blue -->
        <material name="blue_color">
            <color rgba="0 0 1 1"/>
        </material>
    </visual>
</link>
```

### 1.2.2 Joint Description: `<joint>`

- The `<joint>` tag describes the **connection relationship** and **motion rules** between two `<link>` tags.
*   **Analogy**: If `<link>` tags are bones, then `<joint>` tags are elbows or knees, or the glue connecting the bones.
*   **Structure Tree**: URDF uses a tree structure; joints connect a "Parent Link" and a "Child Link."
*   `name`: Joint name (must be unique).
*   `type`: **This is the most important attribute**, determining how the joint moves.

| Joint Type (type) | Motion Description | Typical Use Case Scenario |
| :--- | :--- | :--- |
| **fixed** | **Fixed**, cannot move at all | LiDAR fixed to a chassis |
| **continuous** | **Continuous rotation**, no angle limits | Robot wheels |
| **revolute** | **Rotation**, with angle limits | Robotic arm joints, servos |
| **prismatic** | **Translation**, sliding along an axis | Drawers, elevators |
| **floating** | Floating, 6 degrees of freedom (Rarely used) | Zero-gravity simulation |
| **planar** | Planar motion (Rarely used) | Omnidirectional mobile base |

#### Core Sub-tags

-  `<parent>` and `<child>` (Required)
Defines who is connected to whom.
```xml
<parent link="base_link"/> <!-- Parent is the chassis -->
<child link="camera_link"/> <!-- Child is the camera -->
```

- `<origin>` (Relative Position)
Very crucial! It defines the position of the **Child Link coordinate system origin** relative to the **Parent Link coordinate system origin**.
*   For example: If the camera is installed 0.1m above the chassis, then `xyz="0 0 0.1"`.

-  `<axis>` (Motion Axis)
For rotation or sliding joints, you must specify which axis it rotates around or moves along.
*   `xyz`: Normalized vector.
    *   `0 0 1`: Rotation around the Z-axis (horizontal rotation).
    *   `0 1 0`: Rotation around the Y-axis (wheel rolling is usually Y-axis).

#### Example
```xml
<joint name="base_to_camera" type="fixed">
    <!-- Parent-Child Relationship -->
    <parent link="base_link"/>
    <child link="camera_link"/>
    
    <!-- Installation position: 0.1m above the parent center, 0.05m in front -->
    <origin xyz="0.05 0 0.1" rpy="0 0 0"/>
    
    <!-- Even for "fixed" joints, some parsers may require an axis for syntax completeness; usually set to the Z-axis -->
    <axis xyz="0 0 1"/>
</joint>
```

### Summary: The URDF Modeling "Workflow"

Building a robot model in ROS is essentially repeating the following process:
1.  **Build the Rigid Body**: Use `<link>` to draw what the part looks like.
2.  **Define Relationships**: Use `<joint>` to "hang" this part onto an existing part.
3.  **Set Parameters**: Adjust `<origin>` to ensure accurate positioning, and set `type` to ensure the correct motion.

> **Tip**: `<collision>` (collision parameters) and `<inertial>` (inertia parameters) are only needed later for Gazebo physical simulations. When simply viewing a model in RViz, only the `<visual>` configuration is necessary.

**Experience Case 1: A Red Box**
Please create a new file `01_simple_box.urdf` in the `~/catkin_ws/src/robot_modeling/urdf` folder:
```xml
<?xml version="1.0"?>
<robot name="simple_box">
    <link name="base_link">
        <visual>
            <geometry>
                <!-- Length, width, and height are all 0.2 meters -->
                <box size="0.2 0.2 0.2"/>
            </geometry>
            <material name="red">
                <color rgba="1 0 0 1"/>
            </material>
        </visual>
    </link>
</robot>
```
**View the Result:**
Run in the terminal:
```bash
roslaunch urdf_tutorial display.launch model:=src/robot_modeling/urdf/01_simple_box.urdf
```

<p align="center">
  <a>
    <img src="./images/01_link_sample.png" width="600" height="auto">
  </a>
</p>

*Note: `urdf_tutorial` is a built-in ROS package. If you get an error, please install it: `sudo apt install ros-noetic-urdf-tutorial`. Alternatively, use the generic launch file I provide later.*

**Experience Case 2: Different Joint Effects**
Please create a new file `02_simple_joint.urdf` in the `~/catkin_ws/src/robot_modeling/urdf` folder:

```xml
<?xml version="1.0"?>
<robot name="joint_lab_pure">

    <!-- ================= Material Definitions (Materials) ================= -->
    <!-- In pure URDF, it's best to define materials at the beginning for easy reference later -->
    <material name="blue">
        <color rgba="0 0 0.8 1"/>
    </material>
    
    <material name="red">
        <color rgba="1 0 0 1"/>
    </material>
    
    <material name="green">
        <color rgba="0 1 0 1"/>
    </material>
    
    <material name="white">
        <color rgba="1 1 1 1"/>
    </material>

    <!-- ================= 1. Base (Immobile) ================= -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.2 0.2 0.1"/>
            </geometry>
            <material name="white"/>
        </visual>
    </link>

    <!-- ================= 2. Lift Column (Prismatic Joint) ================= -->
    <link name="slide_pole">
        <visual>
            <geometry>
                <cylinder radius="0.02" length="0.3"/>
            </geometry>
            <!-- Origin offset: Move the cylinder up by 0.15 to align the bottom with the origin -->
            <origin xyz="0 0 0.15" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
    </link>

    <joint name="elevator_joint" type="prismatic">
        <parent link="base_link"/>
        <child link="slide_pole"/>
        <!-- Installation position: 0.05m above the base surface -->
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        
        <!-- Axis: Move along the Z-axis -->
        <axis xyz="0 0 1"/> 
        
        <!-- Limits: Movement range 0 to 0.5 meters -->
        <limit lower="0.0" upper="0.5" effort="10" velocity="1"/>
    </joint>

    <!-- ================= 3. Turntable (Continuous Joint) ================= -->
    <link name="turntable">
        <visual>
            <geometry>
                <cylinder radius="0.08" length="0.02"/>
            </geometry>
            <material name="red"/>
        </visual>
    </link>

    <joint name="spinner_joint" type="continuous">
        <parent link="slide_pole"/>
        <child link="turntable"/>
        <!-- Installation position: Top of the lift column (at 0.3m) -->
        <origin xyz="0 0 0.3" rpy="0 0 0"/>
        
        <!-- Axis: Rotate around the Z-axis -->
        <axis xyz="0 0 1"/>
    </joint>

    <!-- ================= 4. Swing Arm (Revolute Joint) ================= -->
    <link name="swing_arm">
        <visual>
            <geometry>
                <box size="0.3 0.04 0.04"/>
            </geometry>
            <!-- Geometric offset: x=0.15 to place the rotation axis at one end of the box -->
            <origin xyz="0.15 0 0" rpy="0 0 0"/>
            <material name="green"/>
        </visual>
    </link>

    <joint name="elbow_joint" type="revolute">
        <parent link="turntable"/>
        <child link="swing_arm"/>
        <!-- Installation position: 0.02m above the turntable -->
        <origin xyz="0 0 0.02" rpy="0 0 0"/>
        
        <!-- Axis: Rotate around the Y-axis (nodding motion) -->
        <axis xyz="0 1 0"/>
        
        <!-- Limits: Rotation range -1.57 rad (-90°) to +1.57 rad (+90°) -->
        <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>

</robot>
```

#### 2. Running and Experience

We will continue using the generic launch file created earlier, specifying this new file through the `model` parameter.

Run in the terminal:
```bash
roslaunch urdf_tutorial display.launch model:=src/robot_modeling/urdf/02_sample_joint.urdf
```

<p align="center">
  <a>
    <img src="./images/02_joint_sample.png" width="600" height="auto">
  </a>
</p>

1.  **Do you see the GUI window that popped up?** (The title is `Joint State Publisher`)
    *   This is not just a display; it is a **controller**.
2.  **Operate the sliders:**
    *   **`elevator_joint` (Prismatic):** Drag it, and you will see the blue column move **straight up and down** like an elevator.
    *   **`spinner_joint` (Continuous):** Drag it, and you will see the red disc **spin infinitely** (you'll notice the slider can be dragged indefinitely, or the value keeps increasing).
    *   **`elbow_joint` (Revolute):** Drag it, and you will see the green arm **swing up and down** like a human elbow, but it gets stuck and won't turn past +/- 90 degrees.

**Thinking:** If I want to change the box to 0.5 meters, I need to change 3 numbers. If it's a complex robot, do I have to change hundreds of places? I'll give you the answer in the next lesson.