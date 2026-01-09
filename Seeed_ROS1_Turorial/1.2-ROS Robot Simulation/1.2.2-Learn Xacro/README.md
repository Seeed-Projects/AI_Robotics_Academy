# Xacro 初体验

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
