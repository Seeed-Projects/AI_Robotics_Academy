

# URDF 初体验

先准备好工作空间：
```bash
cd ~/catkin_ws/src
catkin_create_pkg robot_modeling urdf xacro geometry_msgs sensor_msgs
mkdir -p robot_modeling/urdf
mkdir -p robot_modeling/launch
mkdir -p robot_modeling/rviz
```


## 1.1 什么是 URDF？
URDF (Unified Robot Description Format) 是 ROS 的标准格式，它像 **HTML** 一样，是静态的。

**核心比喻：**
如果把机器人比作人类，URDF 就是机器人的“解剖图”或“DNA”。
*   **Link (连杆)：** 对应人的骨头和肉体（如大臂、小臂、手掌）。它是刚体，不会变形。
*   **Joint (关节)：** 对应人的肘关节、腕关节。它连接了两个 Link，并定义了它们如何相对运动。

**URDF 的树状结构：**
机器人模型必须是一个**树状结构 (Tree Structure)**。
*   有一个根节点（Root Link）。
*   不能有闭环（Loop）。
*   父子关系明确：一个 Parent 可以有多个 Child，但一个 Child 只能有一个 Parent。


### A. Link 的“三位一体”
一个完善的 Link 定义包含三个部分，缺一不可（特别是为了 Gazebo 仿真）：

1.  **Visual (视觉)：** 给**人**看的。决定了你在 Rviz 里看到它是方的还是圆的，红的还是蓝的。
2.  **Collision (碰撞)：** 给**物理引擎**看的。决定了它会不会穿墙，会不会掉到地板下面。为了计算速度，碰撞模型通常比视觉模型简单（例如复杂的机械臂视觉上很精细，但碰撞模型可能只是几个圆柱体）。
3.  **Inertial (惯性)：** 给**物理引擎**计算力学用的。包含质量 (Mass) 和惯性张量 (Inertia Matrix)。
    *   *教学提示：如果不写 Inertial，模型在 Gazebo 里会表现得像幽灵一样，或者直接被系统忽略。*

### B. Joint 的类型
*   **Fixed (固定)：** 像焊接一样，完全不动（例如：激光雷达固定在车顶）。
*   **Continuous (连续旋转)：** 可以无限旋转（例如：小车的轮子）。
*   **Revolute (有限旋转)：** 有角度限制的旋转（例如：机械臂的关节，范围 -90° 到 +90°）。
*   **Prismatic (平移)：** 直线滑动（例如：抽屉、升降梯）。

## 1.2 URDF基本语法

-  根标签：`<robot>`

每一个URDF文件都必须遵循XML的标准规范。就像HTML文件以`<html>`开头一样，URDF文件必须包含一个根标签。

*   **作用**：定义机器人的作用域，所有关于机器人的描述（连杆、关节、传动等）都必须写在这个标签内部。
*   **核心属性**：
    *   `name`：给你的机器人起个名字（必须属性）。

**代码示例：**
```xml
<robot name="my_first_robot">
    <!-- 所有的机器人部件定义都写在这里 -->
</robot>
```

### 1.2.1 刚体描述：`<link>`

- `<link>` 标签代表机器人的**刚体部件**。
*   **比喻**：如果你在搭乐高，一个`<link>`就是一个乐高积木块（比如底盘、轮子、机械臂的一节、激光雷达）。
*   **特性**：它是刚性的，不可形变。

- `<visual>`：给机器人“穿皮肤”
在`<link>`标签下，最重要的子标签是`<visual>`。它决定了机器人在Rviz中**看起来是什么样子的**。
*   **注意**：`<visual>`的数据只用于显示，不参与物理引擎的碰撞计算。

- `<geometry>` (形状)
这是`<visual>`的核心，用来定义几何形状。URDF支持以下几种基础形状：

| 形状标签 | 属性说明 | 示例 |
| :--- | :--- | :--- |
| **box** (长方体) | `size="长 宽 高"` (x y z) | `<box size="0.5 0.3 0.1" />` |
| **cylinder** (圆柱) | `radius="半径" length="高度"` | `<cylinder radius="0.1" length="0.2" />` |
| **sphere** (球体) | `radius="半径"` | `<sphere radius="0.1" />` |
| **mesh** (模型文件) | `filename="路径"` (加载皮肤) | `filename="package://包名/路径/xxx.dae"` |

- `<origin>` (姿态偏移)
决定了**形状中心**相对于**连杆坐标系中心**的位置。
*   `xyz`：在x, y, z轴上的位移（米）。
*   `rpy`：绕x, y, z轴的旋转弧度（Roll, Pitch, Yaw）。
    *   *注：3.14弧度 ≈ 180度。*

-  `<material>` (颜色/材质)
给形状上色。
*   `rgba`：红(R)、绿(G)、蓝(B)、透明度(A)。数值范围均为 **0到1**。
    *   例：`1 0 0 1` 是纯红色，不透明。

#### 示例
```xml
<link name="base_link">
    <visual>
        <!-- 1. 形状：长0.5 宽0.2 高0.1 的长方体 -->
        <geometry>
            <box size="0.5 0.2 0.1"/>
        </geometry>
        <!-- 2. 偏移：原地不动，不旋转 -->
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <!-- 3. 材质：蓝色 -->
        <material name="blue_color">
            <color rgba="0 0 1 1"/>
        </material>
    </visual>
</link>
```


### 1.2.2 关节描述：`<joint>`

- `<joint>` 标签用于描述两个`<link>`之间的**连接关系**和**运动规则**。
*   **比喻**：如果`<link>`是骨头，那么`<joint>`就是肘关节或膝关节，或者是连接骨头的胶水。
*   **结构树**：URDF采用树状结构，关节连接了“父连杆（Parent）”和“子连杆（Child）”。
*   `name`：关节名称（必须唯一）。
*   `type`：**这是最重要的属性**，决定了关节怎么动。

| 关节类型 (type) | 运动描述 | 典型应用场景 |
| :--- | :--- | :--- |
| **fixed** | **固定**，完全不能动 | 激光雷达固定在底盘上 |
| **continuous** | **连续旋转**，无角度限制 | 机器人的轮子 |
| **revolute** | **旋转**，有角度限制 | 机械臂的关节、舵机 |
| **prismatic** | **平移**，沿轴线滑动 | 抽屉、升降梯 |
| **floating** | 浮动，6自由度 (较少用) | 无重力模拟 |
| **planar** | 平面运动 (较少用) | 全向移动底座 |

#### 核心子标签

-  `<parent>` 和 `<child>` (必选)
定义谁连着谁。
```xml
<parent link="base_link"/> <!-- 爸爸是底盘 -->
<child link="camera_link"/> <!-- 儿子是摄像头 -->
```

- `<origin>` (相对位置)
非常关键！它定义了**子连杆的坐标系原点**相对于**父连杆坐标系原点**的位置。
*   比如：摄像头安装在底盘上方0.1米处，则 `xyz="0 0 0.1"`。

-  `<axis>` (运动轴线)
对于旋转或滑动关节，必须指定它是绕着哪个轴转动/移动的。
*   `xyz`：归一化向量。
    *   `0 0 1`：绕 Z 轴旋转（水平转动）。
    *   `0 1 0`：绕 Y 轴旋转（车轮滚动通常是Y轴）。

#### 示例
```xml
<joint name="base_to_camera" type="fixed">
    <!-- 父子关系 -->
    <parent link="base_link"/>
    <child link="camera_link"/>
    
    <!-- 安装位置：在父级中心上方 0.1米，前方 0.05米处 -->
    <origin xyz="0.05 0 0.1" rpy="0 0 0"/>
    
    <!-- 虽然是fixed，但为了语法完整性，某些解析器可能仍需要axis，通常设为z轴 -->
    <axis xyz="0 0 1"/>
</joint>
```

### 总结：URDF 建模的“套路”

在ROS中构建机器人模型，其实就是不断重复以下过程：
1.  **建刚体**：用 `<link>` 画出部件的样子。
2.  **定关系**：用 `<joint>` 将这个部件“挂”到已有的部件上。
3.  **设参数**：调整 `<origin>` 确保位置准确，设置 `type` 确保运动方式正确。

> **提示**：`<collision>`（碰撞参数）和 `<inertial>`（惯性参数）在后续结合 Gazebo 物理仿真时才需要用到，单纯在 Rviz 中看模型时，仅配置 `<visual>` 即可。



**体验案例 1：一个红色的盒子**
请在 `~/catkin_ws/src/robot_modeling/urdf` 文件夹下新建 `01_simple_box.urdf`：
```xml
<?xml version="1.0"?>
<robot name="simple_box">
    <link name="base_link">
        <visual>
            <geometry>
                <!-- 长宽高都是 0.2 米 -->
                <box size="0.2 0.2 0.2"/>
            </geometry>
            <material name="red">
                <color rgba="1 0 0 1"/>
            </material>
        </visual>
    </link>
</robot>
```
**查看效果：**
在终端运行：
```bash
roslaunch urdf_tutorial display.launch model:=src/robot_modeling/urdf/01_simple_box.urdf
```

<p align="center">
  <a>
    <img src="./images/01_link_sample.png" width="600" height="auto">
  </a>
</p>

*注：`urdf_tutorial` 是ROS自带包，如果报错请安装：`sudo apt install ros-noetic-urdf-tutorial`。或者使用我在后面提供的通用 launch 文件。*


**体验案例 2：不同的Joint效果**
请在 `~/catkin_ws/src/robot_modeling/urdf` 文件夹下新建 `02_simple_joint.urdf`：

```xml
<?xml version="1.0"?>
<robot name="joint_lab_pure">

    <!-- ================= 材质定义 (Materials) ================= -->
    <!-- 在纯URDF中，最好在开头定义好材质，方便后面引用 -->
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

    <!-- ================= 1. 底座 (不可动) ================= -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.2 0.2 0.1"/>
            </geometry>
            <material name="white"/>
        </visual>
    </link>

    <!-- ================= 2. 升降柱 (Prismatic 移动关节) ================= -->
    <link name="slide_pole">
        <visual>
            <geometry>
                <cylinder radius="0.02" length="0.3"/>
            </geometry>
            <!-- 原点偏移：把圆柱向上提0.15，让底端对齐原点 -->
            <origin xyz="0 0 0.15" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
    </link>

    <joint name="elevator_joint" type="prismatic">
        <parent link="base_link"/>
        <child link="slide_pole"/>
        <!-- 安装位置：底座表面上方 0.05米 -->
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        
        <!-- 轴向：沿 Z 轴移动 -->
        <axis xyz="0 0 1"/> 
        
        <!-- 限位：移动范围 0 到 0.5 米 -->
        <limit lower="0.0" upper="0.5" effort="10" velocity="1"/>
    </joint>

    <!-- ================= 3. 旋转台 (Continuous 连续关节) ================= -->
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
        <!-- 安装位置：升降柱顶部 (0.3米处) -->
        <origin xyz="0 0 0.3" rpy="0 0 0"/>
        
        <!-- 轴向：绕 Z 轴旋转 -->
        <axis xyz="0 0 1"/>
    </joint>

    <!-- ================= 4. 摆臂 (Revolute 有限旋转关节) ================= -->
    <link name="swing_arm">
        <visual>
            <geometry>
                <box size="0.3 0.04 0.04"/>
            </geometry>
            <!-- 几何偏移：x=0.15，让转轴位于长方体的一端 -->
            <origin xyz="0.15 0 0" rpy="0 0 0"/>
            <material name="green"/>
        </visual>
    </link>

    <joint name="elbow_joint" type="revolute">
        <parent link="turntable"/>
        <child link="swing_arm"/>
        <!-- 安装位置：旋转台上方 0.02米 -->
        <origin xyz="0 0 0.02" rpy="0 0 0"/>
        
        <!-- 轴向：绕 Y 轴旋转 (点头动作) -->
        <axis xyz="0 1 0"/>
        
        <!-- 限位：旋转范围 -1.57弧度(-90度) 到 +1.57弧度(+90度) -->
        <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>

</robot>
```

#### 2. 运行与体验

我们将继续使用之前创建的通用 launch 文件，通过 `model` 参数指定加载这个新文件。

在终端中运行：
```bash
roslaunch urdf_tutorial display.launch model:=src/robot_modeling/urdf/02_sample_joint.urdf
```

<p align="center">
  <a>
    <img src="./images/02_joint_sample.png" width="600" height="auto">
  </a>
</p>


1.  **看到弹出的 GUI 窗口了吗？** (标题是 `Joint State Publisher`)
    *   这不仅是一个显示器，它是一个**控制器**。
2.  **操作滑块：**
    *   **`elevator_joint` (Prismatic):** 拖动它，你会看到蓝色的柱子像电梯一样**直上直下**。
    *   **`spinner_joint` (Continuous):** 拖动它，你会看到红色的圆盘**无限转圈**（你会发现滑块是可以无限拖动的，或者数值可以一直增加）。
    *   **`elbow_joint` (Revolute):** 拖动它，你会看到绿色的手臂像人的手肘一样**上下摆动**，但是到了 +/- 90度就被卡住了，转不过去。


**思考：** 如果我想把盒子改成 0.5米，我需要改 3 个数字。如果是复杂的机器人，我要改几百个地方。

