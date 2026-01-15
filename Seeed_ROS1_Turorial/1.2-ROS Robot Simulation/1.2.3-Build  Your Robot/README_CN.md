# 分步构建差速小车 (Xacro 实战)

现在我们开始正式造车。我们将创建一个名为 `my_car.xacro` 的文件，并分三次向其中添加代码。

**准备工作：通用 Launch 文件**
为了方便查看，新建 `launch/view_car.launch`：
```xml
<launch>
    <!-- 自动解析 xacro 文件 -->
    <param name="robot_description" command="$(find xacro)/xacro $(find robot_modeling)/urdf/my_car.xacro" />
    
    <!-- 发布关节状态的 GUI 工具 -->
    <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" />
    <!-- 发布机器人的 TF 树 -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
    <!-- 启动 Rviz -->
    <node name="rviz" pkg="rviz" type="rviz" />
</launch>
```

---

## 步骤 1：构建车体 (Chassis)
**教学点：** 定义物理常数，理解 `base_footprint` 的作用。

新建 `urdf/my_car.xacro`，写入以下内容：

```xml
<?xml version="1.0"?>
<robot name="my_car" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- =========== 1. 属性定义 =========== -->
    <xacro:property name="base_len" value="0.4"/>
    <xacro:property name="base_width" value="0.2"/>
    <xacro:property name="base_height" value="0.05"/>
    
    <!-- 颜色定义 -->
    <material name="yellow"><color rgba="1 0.8 0 1"/></material>

    <!-- =========== 2. Base Footprint =========== -->
    <!-- 这是一个虚拟点，贴在地面上，是机器人的原点 -->
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

    <joint name="base_joint" type="fixed">
        <parent link="base_footprint"/>
        <child link="base_link"/>
        <!-- 这里的 0.05 是离地间隙，稍后我们会根据轮子半径修改它 -->
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
    </joint>

    <!-- =========== 3. Base Link (车身) =========== -->
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
        <!-- 惯性矩阵暂时省略，为了教学简洁，仿真章节再加 -->
    </link>

</robot>
```

**运行验证：**
```bash
roslaunch robot_modeling view_car.launch
```

在rviz中将`Fixed Frame`修改为`base_footprint`,并且点击左下角`Add`，添加`RobotModel`。

*效果：应该能看到一个悬浮在空中的黄色方块。*

---

## 步骤 2：添加轮子 (Wheels & Macro)
**教学点：** 使用宏 (Macro) 避免重复代码，理解关节 (Joint) 的坐标变换。

**修改 `my_car.xacro`**，在 `</robot>` 之前插入：

```xml
    <!-- =========== 4. 轮子属性 =========== -->
    <xacro:property name="wheel_radius" value="0.05"/>
    <xacro:property name="wheel_width" value="0.02"/>

    <!-- 修正车身高度：让车身恰好在轮子半径之上 -->
    <!-- 请学员手动修改上面 base_joint 的 z 值为 ${wheel_radius} -->
    
    <!-- =========== 5. 轮子宏定义 (重点) =========== -->
    <!-- name: 轮子名字前缀 (left/right) -->
    <!-- reflect: 1为左，-1为右 (用于Y轴镜像) -->
    <xacro:macro name="wheel_func" params="name reflect">
        <link name="${name}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="${pi/2} 0 0"/> <!-- 翻转圆柱体 -->
                <material name="black"><color rgba="0 0 0 1"/></material>
            </visual>
        </link>

        <joint name="${name}_wheel_joint" type="continuous"> <!-- 连续旋转关节 -->
            <parent link="base_link"/>
            <child link="${name}_wheel"/>
            <!-- 核心算法：Y轴位置 = (车宽/2 + 轮宽/2) * 方向 -->
            <origin xyz="0 ${reflect * (base_width/2 + wheel_width/2)} 0" rpy="0 0 0"/>
            <axis xyz="0 1 0"/> <!-- 绕Y轴旋转 -->
        </joint>
        
    </xacro:macro>

    <!-- =========== 6. 调用宏生成轮子 =========== -->
    <xacro:wheel_func name="left" reflect="1"/>
    <xacro:wheel_func name="right" reflect="-1"/>

    <!-- =========== 4.5 万向轮 (Caster) =========== -->
    <xacro:property name="caster_radius" value="0.015"/> <!-- 半径较小 -->

    <!-- 万向轮宏 -->
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
            <!-- 惯性矩阵：球体 -->
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

        <joint name="${name}_caster_joint" type="fixed"> <!-- 为了仿真稳定，暂时用 fixed，后面加无摩擦属性 -->
            <parent link="base_link"/>
            <child link="${name}_caster"/>
            <!-- Z = 万向轮半径 - 驱动轮半径 (确保刚好贴地) -->
            <origin xyz="${location_x} 0 ${caster_radius - wheel_radius}" rpy="0 0 0"/>
        </joint>
    </xacro:macro>

    <!-- 生成前后万向轮：位置在车头车尾偏内一点点 (0.18m) -->
    <xacro:caster_func name="front" location_x="0.18"/>
    <xacro:caster_func name="rear" location_x="-0.18"/>
```

**运行验证：**
再次运行 launch 文件。
在rviz中将`Fixed Frame`修改为`base_footprint`,并且点击左下角`Add`，添加`RobotModel`。

*效果：你会看到车身两侧多了两个黑色的轮子。拖动 GUI 工具里的滑条，轮子会转动！这就是 URDF 动起来的样子。*

---

## 步骤 3：添加雷达 (Lidar)
**教学点：** 固定关节 (Fixed Joint)，父子关系的传递。

**修改 `my_car.xacro`**，继续插入：

```xml
    <!-- =========== 7. 雷达 =========== -->
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
        <!-- 安装位置：车身中心偏前 0.15米，高度在车顶上方(车高/2 + 雷达高/2) -->
        <origin xyz="0.15 0 ${base_height/2 + 0.05}" rpy="0 0 0"/>
    </joint>
```

---

## 步骤 4：添加摄像头 (Camera)
**教学点：** 简单的几何体组合，理解前视方向。

**修改 `my_car.xacro`**，继续插入：

```xml
    <!-- =========== 8. 摄像头 =========== -->
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
        <!-- 安装位置：车头最前端(车长/2)，高度与车顶齐平 -->
        <origin xyz="${base_len/2} 0 ${base_height/2}" rpy="0 0 0"/>
    </joint>
```

完整代码
```
<?xml version="1.0"?>
<robot name="my_car" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- =========== 1. 属性定义 =========== -->
    <xacro:property name="base_len" value="0.4"/>
    <xacro:property name="base_width" value="0.2"/>
    <xacro:property name="base_height" value="0.05"/>
    
    <!-- 颜色定义 -->
    <material name="yellow"><color rgba="1 0.8 0 1"/></material>

    <!-- =========== 2. Base Footprint =========== -->
    <!-- 这是一个虚拟点，贴在地面上，是机器人的原点 -->
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

    <joint name="base_joint" type="fixed">
        <parent link="base_footprint"/>
        <child link="base_link"/>
        <!-- 这里的 0.05 是离地间隙，稍后我们会根据轮子半径修改它 -->
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
    </joint>

    <!-- =========== 3. Base Link (车身) =========== -->
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
        <!-- 惯性矩阵暂时省略，为了教学简洁，仿真章节再加 -->
    </link>
   
       <!-- =========== 4. 轮子属性 =========== -->
    <xacro:property name="wheel_radius" value="0.05"/>
    <xacro:property name="wheel_width" value="0.02"/>

    <!-- 修正车身高度：让车身恰好在轮子半径之上 -->


    <!-- =========== 5. 轮子宏定义 (重点) =========== -->
    <!-- name: 轮子名字前缀 (left/right) -->
    <!-- reflect: 1为左，-1为右 (用于Y轴镜像) -->
    <xacro:macro name="wheel_func" params="name reflect">
        <link name="${name}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
                </geometry>
                <origin xyz="0 0 0" rpy="${pi/2} 0 0"/> <!-- 翻转圆柱体 -->
                <material name="black"><color rgba="0 0 0 1"/></material>
            </visual>
        </link>

        <joint name="${name}_wheel_joint" type="continuous"> <!-- 连续旋转关节 -->
            <parent link="base_link"/>
            <child link="${name}_wheel"/>
            <!-- 核心算法：Y轴位置 = (车宽/2 + 轮宽/2) * 方向 -->
            <origin xyz="0 ${reflect * (base_width/2 + wheel_width/2)} 0" rpy="0 0 0"/>
            <axis xyz="0 1 0"/> <!-- 绕Y轴旋转 -->
        </joint>
    </xacro:macro>

    <!-- =========== 6. 调用宏生成轮子 =========== -->
    <xacro:wheel_func name="left" reflect="1"/>
    <xacro:wheel_func name="right" reflect="-1"/>
    
    <!-- =========== 6.5 万向轮 (Caster) =========== -->
    <xacro:property name="caster_radius" value="0.015"/> <!-- 半径较小 -->

    <!-- 万向轮宏 -->
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
            <!-- 惯性矩阵：球体 -->
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

        <joint name="${name}_caster_joint" type="fixed"> <!-- 为了仿真稳定，暂时用 fixed，后面加无摩擦属性 -->
            <parent link="base_link"/>
            <child link="${name}_caster"/>
            <!-- Z = 万向轮半径 - 驱动轮半径 (确保刚好贴地) -->
            <origin xyz="${location_x} 0 ${caster_radius - wheel_radius}" rpy="0 0 0"/>
        </joint>
    </xacro:macro>

    <!-- 生成前后万向轮：位置在车头车尾偏内一点点 (0.18m) -->
    <xacro:caster_func name="front" location_x="0.18"/>
    <xacro:caster_func name="rear" location_x="-0.18"/>
    
    <!-- =========== 7. 雷达 =========== -->
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
        <!-- 安装位置：车身中心偏前 0.15米，高度在车顶上方(车高/2 + 雷达高/2) -->
        <origin xyz="0.15 0 ${base_height/2 + 0.05}" rpy="0 0 0"/>
    </joint>
    
        <!-- =========== 8. 摄像头 =========== -->
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
        <!-- 安装位置：车头最前端(车长/2)，高度与车顶齐平 -->
        <origin xyz="${base_len/2} 0 ${base_height/2}" rpy="0 0 0"/>
    </joint>
    
    
</robot>

```


---

## 最终效果验收
再次运行 `roslaunch robot_modeling view_car.launch`。

<p align="center">
  <a>
    <img src="./images/mycar.png" width="600" height="auto">
  </a>
</p>


**Rviz 设置指南：**
1.  **Global Options -> Fixed Frame:** 选择 `base_footprint`。
2.  **Add -> RobotModel:** 你应该能看到一辆完整的车：
    *   黄色的车身。
    *   两个黑色的轮子（可旋转）。
    *   头顶有个红色的雷达柱。
    *   正前方有个绿色的小摄像头。
3.  **Add -> TF:** 你会看到一棵漂亮的坐标树，所有箭头都正确连接。

