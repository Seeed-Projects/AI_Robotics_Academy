
---




# 第一部分：URDF 与 Xacro 的初体验

在机器人开发领域，直接在实体硬件上开发代码往往面临着“高成本、高风险、低效率”的困境。为了解决“买不起昂贵机器人”、“测试环境难以搭建”以及“代码BUG导致炸机”等痛点，仿真（Simulation） 成为了ROS开发中不可或缺的一环。
本节将带你理解ROS仿真的核心逻辑，并介绍构建虚拟机器人世界的三大基石：URDF、Rviz 与 Gazebo。

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

---

## 1.2 什么是 Xacro？（变量的魔法）

原生的 URDF 是纯 XML 文件，写起来非常痛苦。
*   **痛点1（无变量）：** 如果你想把车身长度从 0.5 改成 0.6，你需要手动修改几十个地方（包括位置计算、碰撞体积等）。
*   **痛点2（高冗余）：** 左轮和右轮的代码 99% 是一样的，仅仅是坐标的 Y 值一正一负，但你必须复制粘贴写两遍。

**解决方案：Xacro (XML Macros)**
Xacro 是 URDF 的“升级版”或“预处理器”。它支持：
1.  **Property（变量）：** 定义常量，一处修改，全局生效。
2.  **Macro（宏/函数）：** 把重复的代码封装成函数，调用时传入参数即可。
3.  **Math（数学计算）：** 支持加减乘除，自动计算坐标偏移。

**体验案例 1：可变大小的盒子**
在 `urdf` 文件夹下新建 `01_simple_box.xacro`：
```xml
<?xml version="1.0"?>
<robot name="simple_box_xacro" xmlns:xacro="http://www.ros.org/wiki/xacro">
    
    <!-- 1. 定义变量：我想改大小，只改这里 -->
    <xacro:property name="box_size" value="0.5" />
    
    <link name="base_link">
        <visual>
            <geometry>
                <!-- 2. 调用变量 -->
                <box size="${box_size} ${box_size} ${box_size}"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
    </link>
</robot>
```

**查看效果：**
ROS 不能直接读取 `.xacro`，需要先转换。但在 Launch 文件中可以自动转换。
```bash
roslaunch urdf_tutorial display.launch model:=src/robot_modeling/urdf/01_simple_box.xacro
```

<p align="center">
  <a>
    <img src="./images/01_link_xacro.png" width="600" height="auto">
  </a>
</p>


**现象：** 你会看到一个变大了的蓝色盒子。学员可以尝试修改 `value="0.5"`，不用改下面的代码，模型就会变。


**体验案例 1：Xacro体验不同Joint**
#### 1. 编写代码
请在 `urdf` 文件夹下新建文件 `02_joints_demo.xacro`：

```xml
<?xml version="1.0"?>
<robot name="joint_lab" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- 颜色定义 -->
    <material name="blue"><color rgba="0 0 0.8 1"/></material>
    <material name="red"><color rgba="1 0 0 1"/></material>
    <material name="green"><color rgba="0 1 0 1"/></material>
    <material name="white"><color rgba="1 1 1 1"/></material>

    <!-- 1. 底座 (不可动) -->
    <link name="base_link">
        <visual>
            <geometry><box size="0.2 0.2 0.1"/></geometry>
            <material name="white"/>
        </visual>
    </link>

    <!-- ========================================== -->
    <!-- 演示 1: Prismatic Joint (直线滑动 - 升降柱) -->
    <!-- ========================================== -->
    <link name="slide_pole">
        <visual>
            <geometry><cylinder radius="0.02" length="0.3"/></geometry>
            <origin xyz="0 0 0.15" rpy="0 0 0"/> <!-- 把圆柱向上提，让底端对齐原点 -->
            <material name="blue"/>
        </visual>
    </link>

    <joint name="elevator_joint" type="prismatic">
        <parent link="base_link"/>
        <child link="slide_pole"/>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        
        <!-- 重点：axis 决定沿哪个轴滑动 (这里是 Z 轴) -->
        <axis xyz="0 0 1"/> 
        
        <!-- 重点：limit 必须写 (单位：米) -->
        <!-- lower: 最低点, upper: 最高点 -->
        <limit lower="0.0" upper="0.5" effort="10" velocity="1"/>
    </joint>

    <!-- ========================================== -->
    <!-- 演示 2: Continuous Joint (无限旋转 - 旋转台) -->
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
        <!-- 安装在升降柱的顶部 (0.3米处) -->
        <origin xyz="0 0 0.3" rpy="0 0 0"/>
        
        <!-- 绕 Z 轴旋转 -->
        <axis xyz="0 0 1"/>
    </joint>

    <!-- ========================================== -->
    <!-- 演示 3: Revolute Joint (有限旋转 - 摆臂) -->
    <!-- ========================================== -->
    <link name="swing_arm">
        <visual>
            <geometry><box size="0.3 0.04 0.04"/></geometry>
            <!-- 偏移 x=0.15，让转轴在长方体的一端，而不是中间 -->
            <origin xyz="0.15 0 0" rpy="0 0 0"/>
            <material name="green"/>
        </visual>
    </link>

    <joint name="elbow_joint" type="revolute">
        <parent link="turntable"/>
        <child link="swing_arm"/>
        <origin xyz="0 0 0.02" rpy="0 0 0"/>
        
        <!-- 绕 Y 轴旋转 (点头动作) -->
        <axis xyz="0 1 0"/>
        
        <!-- 重点：limit 必须写 (单位：弧度) -->
        <!-- -1.57 rad 约等于 -90度, 1.57 rad 约等于 +90度 -->
        <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
    </joint>

</robot>
```

#### 2. 运行与体验

在终端中运行：
```bash
roslaunch urdf_tutorial display.launch model:=src/robot_modeling/urdf/02_joints_demo.xacro
```

<p align="center">
  <a>
    <img src="./images/02_joint_sample.png" width="600" height="auto">
  </a>
</p>
