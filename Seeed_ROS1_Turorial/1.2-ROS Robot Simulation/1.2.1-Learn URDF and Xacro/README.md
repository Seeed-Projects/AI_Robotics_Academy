# Part 1: First Experience with URDF and Xacro

First, set up the workspace:
```bash
cd ~/catkin_ws/src
catkin_create_pkg robot_modeling urdf xacro geometry_msgs sensor_msgs
mkdir -p robot_modeling/urdf
mkdir -p robot_modeling/launch
mkdir -p robot_modeling/rviz
```

## 1.1 What is URDF?
URDF (Unified Robot Description Format) is a standard format in ROS. Like **HTML**, it is static.

**Core Analogy:**
If a robot were compared to a human being, URDF would be the robot's "anatomical drawing" or "DNA".
*   **Link:** Corresponding to human bones and body tissues (e.g., upper arm, forearm, palm). It is a rigid body and does not deform.
*   **Joint:** Corresponding to human elbow joints and wrist joints. It connects two Links and defines how they move relative to each other.

**Tree Structure of URDF:**
A robot model must have a **Tree Structure**.
*   There is one root node (Root Link).
*   No closed loops are allowed.
*   Parent-child relationships are clear: one Parent can have multiple Children, but one Child can only have one Parent.

### A. The "Trinity" of a Link
A complete Link definition consists of three indispensable parts (especially for Gazebo simulation):
1.  **Visual:** For **humans** to see. It determines whether the model appears square or round, red or blue in RViz.
2.  **Collision:** For **collision detection**. It is usually simpler than the visual model (e.g., a complex robotic arm may have a detailed visual appearance, but its collision model could be just a few cylinders).
3.  **Inertial:** For **physics engines** to calculate mechanics. It includes Mass and Inertia Matrix.
    *   *Teaching Tip: If Inertial is not defined, the model will behave like a ghost in Gazebo or be ignored by the system entirely.*

### B. Types of Joints
*   **Fixed:** Like being welded, it does not move at all (e.g., a lidar fixed on the roof of a car).
*   **Continuous:** Allows infinite rotation (e.g., the wheels of a car).
*   **Revolute:** Allows rotation within a limited angle range (e.g., a robotic arm joint with a range of -90° to +90°).
*   **Prismatic:** Allows linear sliding (e.g., a drawer, an elevator).

**Hands-On Example 1: A Red Box**
Create a new file named `01_simple_box.urdf` in the `~/catkin_ws/src/robot_modeling/urdf` folder:
```xml
<?xml version="1.0"?>
<robot name="simple_box">
    <link name="base_link">
        <visual>
            <geometry>
                <!-- Length, width and height are all 0.2 meters -->
                <box size="0.2 0.2 0.2"/>
            </geometry>
            <material name="red">
                <color rgba="1 0 0 1"/>
            </material>
        </visual>
    </link>
</robot>
```
**Check the Result:**
Run the following command in the terminal:
```bash
roslaunch urdf_tutorial display.launch model:=src/robot_modeling/urdf/01_simple_box.urdf
```

<p align="center">
  <a>
    <img src="./images/01_link_sample.png" width="600" height="auto">
  </a>
</p>

*Note: `urdf_tutorial` is a built-in ROS package. If an error occurs, use the general launch file provided later.*

**Hands-On Example 2: Effects of Different Joints**
Create a new file named `02_simple_joint.urdf` in the `~/catkin_ws/src/robot_modeling/urdf` folder:

```xml
<?xml version="1.0"?>
<robot name="joint_lab_pure">

    <!-- ================= Material Definitions ================= -->
    <!-- In pure URDF, it is best to define materials at the beginning for easy reference later -->
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

    <!-- ================= 1. Base (Non-movable) ================= -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.2 0.2 0.1"/>
            </geometry>
            <material name="white"/>
        </visual>
    </link>

    <!-- ================= 2. Lifting Column (Prismatic Joint) ================= -->
    <link name="slide_pole">
        <visual>
            <geometry>
                <cylinder radius="0.02" length="0.3"/>
            </geometry>
            <!-- Origin Offset: Lift the cylinder up by 0.15 meters to align its bottom end with the origin -->
            <origin xyz="0 0 0.15" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
    </link>

    <joint name="elevator_joint" type="prismatic">
        <parent link="base_link"/>
        <child link="slide_pole"/>
        <!-- Mounting Position: 0.05 meters above the surface of the base -->
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        
        <!-- Axis Direction: Move along the Z-axis -->
        <axis xyz="0 0 1"/> 
        
        <!-- Limits: Movement range from 0 to 0.5 meters -->
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
        <!-- Mounting Position: At the top of the lifting column (0.3 meters) -->
        <origin xyz="0 0 0.3" rpy="0 0 0"/>
        
        <!-- Axis Direction: Rotate around the Z-axis -->
        <axis xyz="0 0 1"/>
    </joint>

    <!-- ================= 4. Swing Arm (Revolute Joint) ================= -->
    <link name="swing_arm">
        <visual>
            <geometry>
                <box size="0.3 0.04 0.04"/>
            </geometry>
            <!-- Geometry Offset: x=0.15 to position the rotation axis at one end of the cuboid -->
            <origin xyz="0.15 0 0" rpy="0 0 0"/>
            <material name="green"/>
        </visual>
    </link>

    <joint name="elbow_joint" type="revolute">
        <parent link="turntable"/>
        <child link="swing_arm"/>
        <!-- Mounting Position: 0.02 meters above the turntable -->
        <origin xyz="0 0 0.02" rpy="0 0 0"/>
        
        <!-- Axis Direction: Rotate around the Y-axis (nodding motion) -->
        <axis xyz="0 1 0"/>
        
        <!-- Limits: Rotation range from -1.57 radians (-90 degrees) to +1.57 radians (+90 degrees) -->
        <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>

</robot>
```

### 2. Running and Experiencing
We will continue to use the general launch file created earlier, specifying this new file via the `model` parameter.

Run the following command in the terminal:
```bash
roslaunch urdf_tutorial display.launch model:=src/robot_modeling/urdf/02_sample_joint.urdf
```

<p align="center">
  <a>
    <img src="./images/02_joint_sample.png" width="600" height="auto">
  </a>
</p>

1.  **Did you see the pop-up GUI window?** (Title: `Joint State Publisher`)
    *   This is not just a display tool, but also a **controller**.
2.  **Operate the Sliders:**
    *   **`elevator_joint` (Prismatic):** Drag it and you will see the blue column move **straight up and down** like an elevator.
    *   **`spinner_joint` (Continuous):** Drag it and you will see the red disc rotate **infinitely** (you will find that the slider can be dragged endlessly, or the value can keep increasing).
    *   **`elbow_joint` (Revolute):** Drag it and you will see the green arm swing **up and down** like a human elbow, but it will get stuck at +/- 90 degrees and cannot rotate further.

**Food for Thought:** If I want to change the size of the box to 0.5 meters, I need to modify 3 values. For a complex robot, I would have to modify hundreds of places.

## 1.2 What is Xacro? (The Magic of Variables)
Native URDF is a pure XML file, which is very cumbersome to write.
*   **Pain Point 1 (No Variables):** If you want to change the body length from 0.5 to 0.6 meters, you need to manually modify dozens of places (including position calculations, collision volumes, etc.).
*   **Pain Point 2 (High Redundancy):** The code for the left wheel and the right wheel is 99% the same, with only the Y-coordinate being positive and negative respectively, but you have to copy and paste it twice.

**Solution: Xacro (XML Macros)**
Xacro is an "enhanced version" or "preprocessor" of URDF. It supports:
1.  **Property (Variables):** Define constants that take effect globally with a single modification.
2.  **Macro (Functions):** Encapsulate repetitive code into functions and call them with parameters.
3.  **Math (Mathematical Calculations):** Support addition, subtraction, multiplication and division for automatic coordinate offset calculation.

**Hands-On Example 1: A Box with Adjustable Size**
Create a new file named `01_simple_box.xacro` in the `urdf` folder:
```xml
<?xml version="1.0"?>
<robot name="simple_box_xacro" xmlns:xacro="http://www.ros.org/wiki/xacro">
    
    <!-- 1. Define Variable: To change the size, only modify this line -->
    <xacro:property name="box_size" value="0.5" />
    
    <link name="base_link">
        <visual>
            <geometry>
                <!-- 2. Call Variable -->
                <box size="${box_size} ${box_size} ${box_size}"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
    </link>
</robot>
```

**Check the Result:**
ROS cannot read `.xacro` files directly; they need to be converted first. However, conversion can be done automatically in Launch files.
```bash
roslaunch urdf_tutorial display.launch model:=src/robot_modeling/urdf/01_simple_box.xacro
```

<p align="center">
  <a>
    <img src="./images/01_link_xacro.png" width="600" height="auto">
  </a>
</p>

**Observation:** You will see an enlarged blue box. Learners can try modifying `value="0.5"`—the model will change without modifying the code below.

**Hands-On Example 1: Xacro for Different Joints**
### 1. Writing the Code
Create a new file named `02_joints_demo.xacro` in the `urdf` folder:

```xml
<?xml version="1.0"?>
<robot name="joint_lab" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Color Definitions -->
    <material name="blue"><color rgba="0 0 0.8 1"/></material>
    <material name="red"><color rgba="1 0 0 1"/></material>
    <material name="green"><color rgba="0 1 0 1"/></material>
    <material name="white"><color rgba="1 1 1 1"/></material>

    <!-- 1. Base (Non-movable) -->
    <link name="base_link">
        <visual>
            <geometry><box size="0.2 0.2 0.1"/></geometry>
            <material name="white"/>
        </visual>
    </link>

    <!-- ========================================== -->
    <!-- Demo 1: Prismatic Joint (Linear Sliding - Lifting Column) -->
    <!-- ========================================== -->
    <link name="slide_pole">
        <visual>
            <geometry><cylinder radius="0.02" length="0.3"/></geometry>
            <origin xyz="0 0 0.15" rpy="0 0 0"/> <!-- Lift the cylinder up to align its bottom with the origin -->
            <material name="blue"/>
        </visual>
    </link>

    <joint name="elevator_joint" type="prismatic">
        <parent link="base_link"/>
        <child link="slide_pole"/>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        
        <!-- Key Point: The axis determines the sliding direction (Z-axis here) -->
        <axis xyz="0 0 1"/> 
        
        <!-- Key Point: Limits must be defined (Unit: meters) -->
        <!-- lower: Minimum point, upper: Maximum point -->
        <limit lower="0.0" upper="0.5" effort="10" velocity="1"/>
    </joint>

    <!-- ========================================== -->
    <!-- Demo 2: Continuous Joint (Infinite Rotation - Turntable) -->
    <!-- ========================================== -->
    <link name="turntable">
        <visual>
            <geometry><cylinder radius="0.08" length="0.02"/></geometry>
            <material name="red"/>
        </visual>
    </link>

    <joint name="spinner_joint" type="continuous">
        <parent link="slide_pole"/>
        <child link="turntable"/>
        <!-- Mounted at the top of the lifting column (0.3 meters) -->
        <origin xyz="0 0 0.3" rpy="0 0 0"/>
        
        <!-- Rotate around the Z-axis -->
        <axis xyz="0 0 1"/>
    </joint>

    <!-- ========================================== -->
    <!-- Demo 3: Revolute Joint (Limited Rotation - Swing Arm) -->
    <!-- ========================================== -->
    <link name="swing_arm">
        <visual>
            <geometry><box size="0.3 0.04 0.04"/></geometry>
            <!-- Offset x=0.15 to position the rotation axis at one end of the cuboid instead of the center -->
            <origin xyz="0.15 0 0" rpy="0 0 0"/>
            <material name="green"/>
        </visual>
    </link>

    <joint name="elbow_joint" type="revolute">
        <parent link="turntable"/>
        <child link="swing_arm"/>
        <origin xyz="0 0 0.02" rpy="0 0 0"/>
        
        <!-- Rotate around the Y-axis (nodding motion) -->
        <axis xyz="0 1 0"/>
        
        <!-- Key Point: Limits must be defined (Unit: radians) -->
        <!-- -1.57 rad ≈ -90 degrees, 1.57 rad ≈ +90 degrees -->
        <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>

</robot>
```

### 2. Running and Experiencing
Run the following command in the terminal:
```bash
roslaunch urdf_tutorial display.launch model:=src/robot_modeling/urdf/02_joints_demo.xacro
```

<p align="center">
  <a>
    <img src="./images/02_joint_sample.png" width="600" height="auto">
  </a>
</p>
