# First Experience with Xacro

## 1.2 What is Xacro? (The Magic of Variables)

Native URDF consists of pure XML files, which can be quite painful to write.
*   **Pain Point 1 (No Variables):** If you want to change the chassis length from 0.5 to 0.6, you need to manually modify dozens of places (including position calculations, collision volumes, etc.).
*   **Pain Point 2 (High Redundancy):** The code for the left wheel and right wheel is 99% identical—only the Y-coordinate value is positive in one and negative in the other—yet you must copy and paste to write it twice.

**The Solution: Xacro (XML Macros)**
Xacro is an "upgraded version" or "preprocessor" for URDF. It supports:
1.  **Properties (Variables):** Define constants once; changes apply globally.
2.  **Macros (Functions):** Encapsulate repetitive code into functions and call them with parameters.
3.  **Math (Mathematical Calculations):** Supports addition, subtraction, multiplication, and division to automatically calculate coordinate offsets.

**Basic Syntax**
-  **Defining a Parameter**  
    ```xml
    <xacro:property name="xxxx" value="yyyy" />
    <xacro:property name="box_size" value="0.5" />
    <material name="xxx"><color rgba="x x x x"/></material>
    <material name="red"><color rgba="1 0 0 1"/></material>
    ```
-  **Calling a Parameter**
    ```xml
    ${name_value_of_property}
    ${box_size}
    ${red}
    ```

**Experience Case 1: Variable Sized Box**
Create a new file `01_simple_box.xacro` in the `urdf` folder:
```xml
<?xml version="1.0"?>
<robot name="simple_box_xacro" xmlns:xacro="http://www.ros.org/wiki/xacro">
    
    <!-- 1. Define variable: To change the size, only modify this value -->
    <xacro:property name="box_size" value="0.5" />
    
    <link name="base_link">
        <visual>
            <geometry>
                <!-- 2. Call the variable -->
                <box size="${box_size} ${box_size} ${box_size}"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
    </link>
</robot>
```

**Viewing the Result:**
ROS cannot read `.xacro` files directly; they need to be converted first. However, conversion is handled automatically within Launch files.
```bash
roslaunch urdf_tutorial display.launch model:=src/robot_modeling/urdf/01_simple_box.xacro
```

<p align="center">
  <a>
    <img src="./images/01_link_xacro.png" width="600" height="auto">
  </a>
</p>

**Observation:** You will see a blue box that has become larger. Students can try modifying `value="0.5"`; the model will change without needing to touch the code below.

**Experience Case 2: Experiencing Different Joints with Xacro**
#### 1. Writing the Code
Please create a new file `02_joints_demo.xacro` in the `urdf` folder:

```xml
<?xml version="1.0"?>
<robot name="joint_lab" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Color Definitions -->
    <material name="blue"><color rgba="0 0 0.8 1"/></material>
    <material name="red"><color rgba="1 0 0 1"/></material>
    <material name="green"><color rgba="0 1 0 1"/></material>
    <material name="white"><color rgba="1 1 1 1"/></material>

    <!-- 1. Base (Immobile) -->
    <link name="base_link">
        <visual>
            <geometry><box size="0.2 0.2 0.1"/></geometry>
            <material name="white"/>
        </visual>
    </link>

    <!-- ========================================== -->
    <!-- Demo 1: Prismatic Joint (Linear Sliding - Lift Column) -->
    <!-- ========================================== -->
    <link name="slide_pole">
        <visual>
            <geometry><cylinder radius="0.02" length="0.3"/></geometry>
            <origin xyz="0 0 0.15" rpy="0 0 0"/> <!-- Lift the cylinder so the bottom aligns with the origin -->
            <material name="blue"/>
        </visual>
    </link>

    <joint name="elevator_joint" type="prismatic">
        <parent link="base_link"/>
        <child link="slide_pole"/>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        
        <!-- Key point: axis determines which axis to slide along (Z-axis here) -->
        <axis xyz="0 0 1"/> 
        
        <!-- Key point: limit must be specified (Unit: meters) -->
        <!-- lower: lowest point, upper: highest point -->
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
        <!-- Installed at the top of the lift column (at 0.3m) -->
        <origin xyz="0 0 0.3" rpy="0 0 0"/>
        
        <!-- Rotate around the Z-axis -->
        <axis xyz="0 0 1"/>
    </joint>

    <!-- ========================================== -->
    <!-- Demo 3: Revolute Joint (Finite Rotation - Swing Arm) -->
    <!-- ========================================== -->
    <link name="swing_arm">
        <visual>
            <geometry><box size="0.3 0.04 0.04"/></geometry>
            <!-- Offset x=0.15 to place the rotation axis at one end of the box rather than the middle -->
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
        
        <!-- Key point: limit must be specified (Unit: radians) -->
        <!-- -1.57 rad is approx -90 degrees, 1.57 rad is approx +90 degrees -->
        <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>

</robot>
```

#### 2. Running and Experience

Run in the terminal:
```bash
roslaunch urdf_tutorial display.launch model:=src/robot_modeling/urdf/02_joints_demo.xacro
```

<p align="center">
  <a>
    <img src="./images/02_joint_sample.png" width="600" height="auto">
  </a>
</p>