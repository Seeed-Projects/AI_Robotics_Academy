# Step-by-Step Construction of a Differential Car (Xacro in Practice)

Now, we will formally begin building the car. We will create a file named `my_car.xacro` and add code to it step-by-step.

**Preparation: General Launch File**
To facilitate viewing the model, create a new file: `launch/view_car.launch`:
```xml
<launch>
    <!-- Automatically parse xacro files -->
    <param name="robot_description" command="$(find xacro)/xacro $(find robot_modeling)/urdf/my_car.xacro" />
    
    <!-- GUI tool for publishing joint states -->
    <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />
    <!-- Publish the robot's TF tree -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
    <!-- Launch RViz -->
    <node name="rviz" pkg="rviz" type="rviz" />
</launch>
```

---

## Step 1: Building the Chassis
**Teaching Points:** Define physical constants and understand the role of `base_footprint`.

Create `urdf/my_car.xacro` and write the following:

```xml
<?xml version="1.0"?>
<robot name="my_car" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- =========== 1. Property Definitions =========== -->
    <xacro:property name="base_len" value="0.4"/>
    <xacro:property name="base_width" value="0.2"/>
    <xacro:property name="base_height" value="0.05"/>
    
    <!-- Color Definitions -->
    <material name="yellow"><color rgba="1 0.8 0 1"/></material>

    <!-- =========== 2. Base Footprint =========== -->
    <!-- This is a virtual point placed on the ground, serving as the robot's origin -->
    <link name="base_footprint">
        <visual>
            <geometry>
                <sphere radius="0.001"/>
            </geometry>
        </visual>
        <inertial>
            <mass value="0.0001"/>
            <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
        </inertial>
    </link>

    <joint name="base_joint" type="fixed">
        <parent link="base_footprint"/>
        <child link="base_link"/>
        <!-- The 0.05 here is the initial ground clearance; we will modify it later based on the wheel radius -->
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
    </joint>

    <!-- =========== 3. Base Link (Chassis Body) =========== -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="${base_len} ${base_width} ${base_height}"/>
            </geometry>
            <material name="yellow"/>
        </visual>
        <collision>
            <geometry>
                <box size="${base_len} ${base_width} ${base_height}"/>
            </geometry>
        </collision>
        <!-- Inertia matrices are omitted for now for teaching simplicity; they will be added in the simulation section -->
    </link>

</robot>
```

**Run Verification:**
```bash
roslaunch robot_modeling view_car.launch
```

In RViz, change the `Fixed Frame` to `base_footprint`, and click `Add` in the bottom left corner to add the `RobotModel`.

*Effect: You should see a yellow block suspended in the air.*

---

## Step 2: Adding Wheels (Wheels & Macro)
**Teaching Points:** Use Macros to avoid redundant code and understand the coordinate transformation of Joints.

**Modify `my_car.xacro`** by inserting the following before `</robot>`:

```xml
    <!-- =========== 4. Wheel Properties =========== -->
    <xacro:property name="wheel_radius" value="0.05"/>
    <xacro:property name="wheel_width" value="0.02"/>

    <!-- Correct Chassis Height: Place the chassis exactly above the wheel radius -->
    <!-- Please manually change the z-offset of the base_joint above to ${wheel_radius} -->
    
    <!-- =========== 5. Wheel Macro Definition (Crucial) =========== -->
    <!-- name: Wheel name prefix (left/right) -->
    <!-- reflect: 1 for left, -1 for right (for Y-axis mirroring) -->
    <xacro:macro name="wheel_func" params="name reflect">
        <link name="${name}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="${pi/2} 0 0"/> <!-- Flip the cylinder to make it horizontal -->
                <material name="black"><color rgba="0 0 0 1"/></material>
            </visual>
        </link>

        <joint name="${name}_wheel_joint" type="continuous"> <!-- Continuous rotation joint -->
            <parent link="base_link"/>
            <child link="${name}_wheel"/>
            <!-- Core Algorithm: Y-axis position = (Chassis Width/2 + Wheel Width/2) * Direction -->
            <origin xyz="0 ${reflect * (base_width/2 + wheel_width/2)} 0" rpy="0 0 0"/>
            <axis xyz="0 1 0"/> <!-- Rotate around the Y-axis -->
        </joint>
    </xacro:macro>

    <!-- =========== 6. Call the Macro to generate Drive Wheels =========== -->
    <xacro:wheel_func name="left" reflect="1"/>
    <xacro:wheel_func name="right" reflect="-1"/>

    <!-- =========== 6.5 Caster =========== -->
    <xacro:property name="caster_radius" value="0.015"/> <!-- Smaller radius -->

    <!-- Caster Macro -->
    <xacro:macro name="caster_func" params="name location_x">
        <link name="${name}_caster">
            <visual>
                <geometry>
                    <sphere radius="${caster_radius}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <material name="black"/>
            </visual>
            <collision>
                <geometry>
                    <sphere radius="${caster_radius}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="0 0 0"/>
            </collision>
            <!-- Inertia Matrix Macro: Sphere -->
            <xacro:macro name="sphere_inertia" params="m r">
                <inertial>
                    <mass value="${m}"/>
                    <inertia ixx="${2*m*r*r/5}" ixy="0" ixz="0"
                             iyy="${2*m*r*r/5}" iyz="0"
                             izz="${2*m*r*r/5}"/>
                </inertial>
            </xacro:macro>
            <xacro:sphere_inertia m="0.05" r="${caster_radius}"/>
        </link>

        <joint name="${name}_caster_joint" type="fixed"> <!-- Using 'fixed' temporarily for simulation stability -->
            <parent link="base_link"/>
            <child link="${name}_caster"/>
            <!-- Z offset = Caster Radius - Drive Wheel Radius (ensures it touches the ground) -->
            <origin xyz="${location_x} 0 ${caster_radius - wheel_radius}" rpy="0 0 0"/>
        </joint>
    </xacro:macro>

    <!-- Generate front and rear casters: positioned slightly inside the front and rear (0.18m) -->
    <xacro:caster_func name="front" location_x="0.18"/>
    <xacro:caster_func name="rear" location_x="-0.18"/>
```

**Run Verification:**
Run the launch file again. In RViz, change the `Fixed Frame` to `base_footprint`, and add the `RobotModel`.

*Effect: You will see two black wheels on the sides of the chassis and small casters at the front and rear. Drag the sliders in the GUI tool, and the drive wheels will rotate! This is how a URDF comes to life.*

---

## Step 3: Adding Lidar
**Teaching Points:** Fixed Joints and the passing of parent-child relationships.

**Modify `my_car.xacro`** by continuing to insert:

```xml
    <!-- =========== 7. Lidar =========== -->
    <link name="laser_link">
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.1"/>
            </geometry>
            <material name="red"><color rgba="1 0 0 1"/></material>
        </visual>
    </link>

    <joint name="laser_joint" type="fixed">
        <parent link="base_link"/>
        <child link="laser_link"/>
        <!-- Installation position: 0.15m forward from the chassis center, height above the roof (Chassis Height/2 + Lidar Height/2) -->
        <origin xyz="0.15 0 ${base_height/2 + 0.05}" rpy="0 0 0"/>
    </joint>
```

---

## Step 4: Adding a Camera
**Teaching Points:** Simple geometric combinations and understanding the forward-facing direction.

**Modify `my_car.xacro`** by continuing to insert:

```xml
    <!-- =========== 8. Camera =========== -->
    <link name="camera_link">
        <visual>
            <geometry>
                <box size="0.02 0.04 0.02"/>
            </geometry>
            <material name="green"><color rgba="0 1 0 1"/></material>
        </visual>
    </link>

    <joint name="camera_joint" type="fixed">
        <parent link="base_link"/>
        <child link="camera_link"/>
        <!-- Installation position: Very front of the chassis (Chassis Length/2), level with the roof height -->
        <origin xyz="${base_len/2} 0 ${base_height/2}" rpy="0 0 0"/>
    </joint>
```

---

## Final Result Verification
Run `roslaunch robot_modeling view_car.launch` again.

**RViz Setup Guide:**
1.  **Global Options -> Fixed Frame:** Select `base_footprint`.
2.  **Add -> RobotModel:** You should see a complete car:
    *   A yellow chassis.
    *   Two black wheels (rotatable).
    *   A red lidar column on top.
    *   A small green camera in the front.
3.  **Add -> TF:** You will see a beautiful coordinate tree with all arrows correctly connected.