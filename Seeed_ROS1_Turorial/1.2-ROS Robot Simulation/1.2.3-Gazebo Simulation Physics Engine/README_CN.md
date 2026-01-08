
### 第一阶段：物理觉醒 (配置 Gazebo)

之前的 URDF 只有“皮囊”（Visual），没有“灵魂”（Inertial 和 Plugins）。Gazebo 是物理引擎，如果物体没有质量（Inertia），它就会被忽略或者乱飞。

#### 1. 修改 `my_car.xacro` 添加惯性矩阵
请打开 `src/robot_modeling/urdf/my_car.xacro`，我们需要做两件事：
1.  **定义惯性宏**（放在文件最前面，属性定义之后）。
2.  **给每个 Link 加上 `<inertial>`**。

**将以下代码插入到 `my_car.xacro` 的属性定义下方：**

```xml
    <!-- 惯性矩阵宏 (直接复制即可) -->
    <xacro:macro name="box_inertia" params="m w h d">
        <inertial>
            <mass value="${m}"/>
            <inertia ixx="${m/12*(h*h+d*d)}" ixy="0" ixz="0" iyy="${m/12*(w*w+d*d)}" iyz="0" izz="${m/12*(w*w+h*h)}"/>
        </inertial>
    </xacro:macro>

    <xacro:macro name="cylinder_inertia" params="m r h">
        <inertial>
            <mass value="${m}"/>
            <inertia ixx="${m*(3*r*r+h*h)/12}" ixy="0" ixz="0" iyy="${m*(3*r*r+h*h)/12}" iyz="0" izz="${m*r*r/2}"/>
        </inertial>
    </xacro:macro>
```

**然后修改各个 Link，添加惯性调用（在 collision 标签后面添加）：**

*   **对于 `base_link`:**
    ```xml
    <xacro:box_inertia m="2.0" w="${base_width}" h="${base_height}" d="${base_len}"/>
    ```
*   **对于 `wheel_func` 宏里的 link:**
    ```xml
    <xacro:cylinder_inertia m="0.2" r="${wheel_radius}" h="${wheel_width}"/>
    ```
*   **对于 `laser_link`:**
    ```xml
    <xacro:cylinder_inertia m="0.1" r="0.03" h="0.04"/>
    ```
*   **对于 `camera_link`:**
    ```xml
    <xacro:box_inertia m="0.05" w="0.02" h="0.02" d="0.04"/>
    ```

完整的代码如下 

```
<?xml version="1.0"?>
<robot name="my_car" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- ================================================================== -->
    <!-- 1. 参数配置 (Properties) -->
    <!-- ================================================================== -->
    <xacro:property name="base_len" value="0.4"/>
    <xacro:property name="base_width" value="0.2"/>
    <xacro:property name="base_height" value="0.05"/>
    <xacro:property name="wheel_radius" value="0.05"/>
    <xacro:property name="wheel_width" value="0.02"/>
    <xacro:property name="caster_radius" value="0.015"/> <!-- 万向轮半径 -->

    <!-- 颜色定义 -->
    <material name="yellow"><color rgba="1 0.8 0 1"/></material>
    <material name="black"><color rgba="0 0 0 1"/></material>
    <material name="red"><color rgba="1 0 0 1"/></material>
    <material name="green"><color rgba="0 1 0 1"/></material>

    <!-- ================================================================== -->
    <!-- 2. 惯性矩阵宏 (Inertia Macros) - 物理引擎的核心 -->
    <!-- ================================================================== -->
    
    <!-- 盒子惯性公式 -->
    <xacro:macro name="box_inertia" params="m w h d">
        <inertial>
            <mass value="${m}"/>
            <inertia ixx="${m/12*(h*h+d*d)}" ixy="0" ixz="0" 
                     iyy="${m/12*(w*w+d*d)}" iyz="0" 
                     izz="${m/12*(w*w+h*h)}"/>
        </inertial>
    </xacro:macro>

    <!-- 圆柱体惯性公式 -->
    <xacro:macro name="cylinder_inertia" params="m r h">
        <inertial>
            <mass value="${m}"/>
            <inertia ixx="${m*(3*r*r+h*h)/12}" ixy="0" ixz="0" 
                     iyy="${m*(3*r*r+h*h)/12}" iyz="0" 
                     izz="${m*r*r/2}"/>
        </inertial>
    </xacro:macro>
    
    <!-- 球体惯性公式 (用于万向轮) -->
    <xacro:macro name="sphere_inertia" params="m r">
        <inertial>
            <mass value="${m}"/>
            <inertia ixx="${2*m*r*r/5}" ixy="0" ixz="0" 
                     iyy="${2*m*r*r/5}" iyz="0" 
                     izz="${2*m*r*r/5}"/>
        </inertial>
    </xacro:macro>

    <!-- ================================================================== -->
    <!-- 3. 机器人本体 (Robot Body) -->
    <!-- ================================================================== -->

    <!-- Base Footprint: 虚拟根节点 -->
    <link name="base_footprint">
        <visual>
            <geometry>
                <sphere radius="0.001"/>
            </geometry>
        </visual>
        <!-- [重要修复] 给根节点加微小惯性，防止Gazebo报错把整个车删掉 -->
        <inertial>
            <mass value="0.0001"/>
            <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
        </inertial>
    </link>

    <joint name="base_joint" type="fixed">
        <parent link="base_footprint"/>
        <child link="base_link"/>
        <!-- 修正高度：让车轮底端刚好接触地面 -->
        <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
    </joint>

    <!-- Base Link: 车身 -->
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
        <!-- 调用宏：车身重 2kg -->
        <xacro:box_inertia m="2.0" w="${base_width}" h="${base_height}" d="${base_len}"/>
    </link>

    <!-- ================================================================== -->
    <!-- 4. 轮子宏 (Wheels) -->
    <!-- ================================================================== -->
    <xacro:macro name="wheel_func" params="name reflect">
        <link name="${name}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="${pi/2} 0 0"/> 
                <material name="black"/>
            </visual>
            <collision>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="${pi/2} 0 0"/>
            </collision>
            <!-- [重要修复] 轮子必须有惯性，否则Gazebo会忽略它 -->
            <xacro:cylinder_inertia m="0.2" r="${wheel_radius}" h="${wheel_width}"/>
        </link>

        <joint name="${name}_wheel_joint" type="continuous">
            <parent link="base_link"/>
            <child link="${name}_wheel"/>
            <origin xyz="0 ${reflect * (base_width/2 + wheel_width/2)} 0" rpy="0 0 0"/>
            <axis xyz="0 1 0"/>
        </joint>
    </xacro:macro>

    <!-- 生成左右驱动轮 -->
    <xacro:wheel_func name="left" reflect="1"/>
    <xacro:wheel_func name="right" reflect="-1"/>

    <!-- ================================================================== -->
    <!-- 5. 万向轮宏 (Caster Wheels) - 防止翻车 -->
    <!-- ================================================================== -->
    <xacro:macro name="caster_func" params="name location_x">
        <link name="${name}_caster">
            <visual>
                <geometry>
                    <sphere radius="${caster_radius}"/>
                </geometry>
                <material name="black"/>
            </visual>
            <collision>
                <geometry>
                    <sphere radius="${caster_radius}"/>
                </geometry>
            </collision>
            <xacro:sphere_inertia m="0.05" r="${caster_radius}"/>
        </link>

        <joint name="${name}_caster_joint" type="fixed">
            <parent link="base_link"/>
            <child link="${name}_caster"/>
            <!-- 核心计算：Z = 0.015 - 0.05 = -0.035，让小球刚好接触地面 -->
            <origin xyz="${location_x} 0 ${caster_radius - wheel_radius}" rpy="0 0 0"/>
        </joint>
    </xacro:macro>

    <!-- 生成前后万向轮 -->
    <xacro:caster_func name="front" location_x="0.18"/>
    <xacro:caster_func name="rear" location_x="-0.18"/>

    <!-- ================================================================== -->
    <!-- 6. 传感器 (Sensors) -->
    <!-- ================================================================== -->

    <!-- 雷达 -->
    <link name="laser_link">
        <visual>
            <geometry>
                <cylinder radius="0.03" length="0.04"/>
            </geometry>
            <material name="red"/>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.03" length="0.04"/>
            </geometry>
        </collision>
        <xacro:cylinder_inertia m="0.1" r="0.03" h="0.04"/>
    </link>

    <joint name="laser_joint" type="fixed">
        <parent link="base_link"/>
        <child link="laser_link"/>
        <origin xyz="0.15 0 ${base_height/2 + 0.02}" rpy="0 0 0"/>
    </joint>

    <!-- 摄像头 -->
    <link name="camera_link">
        <visual>
            <geometry>
                <box size="0.02 0.04 0.02"/>
            </geometry>
            <material name="green"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.02 0.04 0.02"/>
            </geometry>
        </collision>
        <xacro:box_inertia m="0.05" w="0.02" h="0.02" d="0.04"/>
    </link>

    <joint name="camera_joint" type="fixed">
        <parent link="base_link"/>
        <child link="camera_link"/>
        <origin xyz="${base_len/2} 0 ${base_height/2}" rpy="0 0 0"/>
    </joint>

    <!-- ================================================================== -->
    <!-- 7. 引入外部插件文件 (Controller) -->
    <!-- ================================================================== -->
    <xacro:include filename="$(find robot_modeling)/urdf/gazebo_plugins.xacro"/>

</robot>
```


#### 2. 创建 Gazebo 插件文件
为了保持代码整洁，我们新建一个文件 `src/robot_modeling/urdf/gazebo_plugins.xacro`，专门放控制器。

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- ================== 1. 颜色修正 ================== -->
    <!-- Gazebo 无法识别 URDF 的 material 标签，需要专用标签 -->
    <gazebo reference="base_link"><material>Gazebo/Yellow</material></gazebo>
    <gazebo reference="left_wheel"><material>Gazebo/Black</material></gazebo>
    <gazebo reference="right_wheel"><material>Gazebo/Black</material></gazebo>
    <gazebo reference="front_caster"><material>Gazebo/Black</material></gazebo>
    <gazebo reference="rear_caster"><material>Gazebo/Black</material></gazebo>
    <gazebo reference="laser_link"><material>Gazebo/Red</material></gazebo>
    <gazebo reference="camera_link"><material>Gazebo/Green</material></gazebo>

    <!-- ================== 2. 差速驱动插件 (让车能动) ================== -->
    <gazebo>
        <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
            <alwaysOn>true</alwaysOn>
            <updateRate>50.0</updateRate>
            
            <!-- 关节名必须和 my_car.xacro 中定义的 Joint 名字一致 -->
            <leftJoint>left_wheel_joint</leftJoint>
            <rightJoint>right_wheel_joint</rightJoint>
            
            <!-- 轮间距和直径 -->
            <wheelSeparation>${base_width + wheel_width}</wheelSeparation>
            <wheelDiameter>${wheel_radius * 2}</wheelDiameter>
            
            <torque>10</torque>
            <commandTopic>cmd_vel</commandTopic>
            <odometryTopic>odom</odometryTopic>
            <odometryFrame>odom</odometryFrame>
            <robotBaseFrame>base_footprint</robotBaseFrame>
            <publishWheelTF>true</publishWheelTF>
            <publishOdomTF>true</publishOdomTF>
        </plugin>
    </gazebo>

    <!-- ================== 3. 雷达插件 (让雷达发数据) ================== -->
    <gazebo reference="laser_link">
        <sensor type="ray" name="head_hokuyo_sensor">
            <pose>0 0 0 0 0 0</pose>
            <visualize>true</visualize> <!-- 在 Gazebo 中可以看到蓝色的扫描线 -->
            <update_rate>20</update_rate>
            <ray>
                <scan>
                    <horizontal>
                        <samples>720</samples>
                        <resolution>1</resolution>
                        <min_angle>-1.57</min_angle> <!-- -90度 -->
                        <max_angle>1.57</max_angle>  <!-- +90度 -->
                    </horizontal>
                </scan>
                <range>
                    <min>0.10</min>
                    <max>1.0</max>
                    <resolution>0.01</resolution>
                </range>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.01</stddev>
                </noise>
            </ray>
            <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
                <topicName>/scan</topicName>
                <frameName>laser_link</frameName>
            </plugin>
        </sensor>
    </gazebo>

</robot>
```

#### 3. 启动 Gazebo 环境
新建 `launch/gazebo_world.launch`：

```xml
<launch>
    <!-- 1. 加载机器人模型 -->
    <param name="robot_description" command="$(find xacro)/xacro $(find robot_modeling)/urdf/my_car.xacro" />

    <!-- 2. 启动 Gazebo -->
    <!-- 我们改用 empty_world.launch，它是最底层的启动文件，支持所有参数 -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <!-- 指定加载 willowgarage 世界文件 -->
        <arg name="world_name" value="$(find gazebo_ros)/worlds/willowgarage.world"/>
        <arg name="paused" value="false"/>
        <arg name="use_sim_time" value="true"/>
        <arg name="gui" value="true"/> <!-- 这里就可以用 gui 参数了 -->
    </include>

    <!-- 3. 在 Gazebo 中生成机器人 -->
    <node name="spawn_model" pkg="gazebo_ros" type="spawn_model" args="-urdf -model my_car -param robot_description" output="screen" />
    
    <!-- 4. 发布 TF -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
</launch>
```

**体验 1：** 运行 `roslaunch robot_modeling gazebo_world.launch`。
*   你应该能看到小车出现在一个环境中。
*   你会看到雷达在发射蓝色的光线。