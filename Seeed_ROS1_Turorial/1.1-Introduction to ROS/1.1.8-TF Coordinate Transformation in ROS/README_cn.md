# 1.1.8：ROS1 TF 坐标变换教程内容

## 1. 引子：为什么需要TF？（用人体做比喻）
**话术：**
“想象一下，你的眼睛看到前方一米处有一个苹果。你的大脑怎么知道手要伸多远才能拿到它？
1.  眼睛告诉大脑：苹果在**眼睛**的前方1米。
2.  大脑知道：眼睛长在**头**上。
3.  大脑知道：头连接在**身体**上。
4.  大脑知道：手连接在**身体**上。
通过这一系列的‘连接关系’，大脑才能算出‘手’相对于‘苹果’的位置。ROS的TF就是这个‘大脑’，它维护了机器人各个部件之间的**相对位置**和**相对角度**。”

## 2. 核心概念：TF树 (The TF Tree)
**核心知识点：**
*   **父子关系 (Parent/Child)：** TF是一个树状结构，不允许有闭环。每个坐标系（Frame）只能有一个父节点，但可以有多个子节点。
*   **变换 (Transform)：** 包含了**平移 (Translation, x/y/z)** 和 **旋转 (Rotation, Quaternion/RPY)**。即：子坐标系相对于父坐标系移动了多少，转动了多少。
*   **时间戳 (Timestamp)：** 机器人在动，坐标关系在变，所以每个TF数据都带有时间，TF库会自动帮你处理时间同步（比如插值）。

## 3. ROS中的“标准坐标系全家桶”

  <div align="center">
      <img width={400} 
      src="./images/Mermaid_cn.png" />
  </div>


*   **`map` (地图坐标系)**
    *   **角色：** 世界的绝对中心，上帝视角。
    *   **特点：** 在这里，坐标是不会随时间漂移的（或者说不应该漂移）。如果你做导航，目标点通常是在`map`坐标系下的。
    *   **谁发布它：** 通常是定位算法（如AMCL）或SLAM算法（如Gmapping）。

*   **`odom` (里程计坐标系)**
    *   **角色：** 机器人以为自己走过的路。
    *   **特点：** 它是连续的（平滑的），但有误差（会漂移）。比如机器人轮子打滑了，`odom`以为车走了，其实车没动。
    *   **关系：** `map` 到 `odom` 之间的变换，就是**“机器人的定位误差修正”**。

*   **`base_link` (基座坐标系)**
    *   **角色：** 机器人的物理中心（通常在两轮轴中心或底盘中心）。
    *   **关系：** `odom` 到 `base_link` 的变换，代表机器人根据编码器/IMU计算出的移动量。

*   **`laser_link` / `camera_link` (传感器坐标系)**
    *   **角色：** 传感器的安装位置。
    *   **特点：** 通常通过`static_transform_publisher`发布，因为传感器焊死在底盘上，它们相对底盘的位置是永远不变的。

## 4. 举个例子：雷达数据上墙
**场景描述：**
雷达扫到了前方障碍物，数据里写着：“前方 x=2.0米 有障碍”。

**TF的工作流：**
1.  **雷达说：** 我（`laser_link`）看到 (2, 0, 0) 处有东西。
2.  **TF查询：** `laser_link` 离 `base_link` 多远？（假设雷达装在车头前0.5米）。
    *   推算：障碍物离车中心 `2.0 + 0.5 = 2.5` 米。
3.  **TF查询：** `base_link` 现在在 `map` 的哪里？（假设车在地图的 (10, 10) 位置，朝向东）。
    *   推算：障碍物在地图的 `(12.5, 10)` 位置。

**总结：** 只要有TF树，你可以在任意两个坐标系之间转换数据，这就是TF强大的地方。

## 5. 常用工具（Show me the code/tools）
教给别人如何Debug TF问题：

1.  **查看树结构（生成PDF图）：**
    ```bash
    rosrun tf view_frames
    evince frames.pdf
    ```
    *这是最直观的方法，能看到谁连着谁，以及频率是否正常。*

2.  **实时监测变换关系：**
    ```bash
    rosrun tf tf_echo [source_frame] [target_frame]
    # 例如： rosrun tf tf_echo map base_link
    ```
    *这会告诉你机器人现在的绝对坐标。*

3.  **Rviz可视化：**
    *   在Rviz中添加 `TF` display。
    *   勾选 `Show Names`。
    *   你会在屏幕上看到一堆红绿蓝的坐标轴（RGB对应XYZ），这就直观展示了TF。

## 6. 现在来动手实操一下

设计一个最简单的**“雷达扫描”场景**：
1.  **`base_link`**：机器人的基座，不动。
2.  **`radar_link`**：安装在基座上的雷达，它会旋转。

我们将编写两个Python脚本：一个**发布**这个旋转关系（Broadcaster），一个**监听**并计算坐标（Listener）。


### 准备工作

首先，确保他们已经创建了一个ROS工作空间和包（假设包名叫 `learning_tf`）。
```bash
# 如果还没有包
cd ~/catkin_ws/src
catkin_create_pkg learning_tf rospy tf geometry_msgs
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

### 第一步：发布坐标系 (The Broadcaster)
**任务：** 告诉ROS，`radar_link` 在哪。
**逻辑：** 每隔0.1秒，更新一次 `radar_link` 相对于 `base_link` 的角度。

在 `src/` 下新建 `tf_broadcaster.py`：

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math

if __name__ == '__main__':
    rospy.init_node('simple_tf_broadcaster')
    
    # 创建一个广播器，相当于“大喇叭”
    br = tf.TransformBroadcaster()
    
    rate = rospy.Rate(10) # 10Hz
    angle = 0
    
    print("开始发布 radar_link 到 base_link 的坐标变换...")

    while not rospy.is_shutdown():
        # 1. 计算旋转的四元数 (ROS使用四元数表示旋转，不用担心，tf提供了转换函数)
        # 参数：roll, pitch, yaw。这里我们让它绕Z轴旋转(yaw)
        angle += 0.05
        quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
        
        # 2. 发布TF
        # 参数顺序：
        # (x, y, z): 平移距离。假设雷达安装在基座前方 1.0米，高度 0.5米处
        # quaternion: 旋转角度
        # time: 时间戳，通常用当前时间
        # child_frame: 子坐标系名字 (radar)
        # parent_frame: 父坐标系名字 (base)
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

### 第二步：监听并换算坐标 (The Listener)
**任务：** 假设雷达检测到了一个障碍物，坐标是 **雷达前方 2米**。请问这个障碍物在 **机器人基座** 看来，在哪里？
**逻辑：** 监听TF树，获取当前关系，进行数学变换。

在 `src/` 下新建 `tf_listener.py`：

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import PointStamped

if __name__ == '__main__':
    rospy.init_node('simple_tf_listener')

    # 创建一个监听器，相当于“耳朵”
    listener = tf.TransformListener()
    
    rate = rospy.Rate(1) # 1Hz，每秒查一次
    
    print("等待TF网络建立...")
    
    while not rospy.is_shutdown():
        try:
            # 1. 这是一个关键步骤：等待两个坐标系建立连接
            # 如果不加这句，程序刚启动时可能会因为没收到数据而报错
            listener.waitForTransform("base_link", "radar_link", rospy.Time(0), rospy.Duration(3.0))
            
            # 2. 定义一个点：在 radar_link 下的坐标 (2.0, 0.0, 0.0)
            # 意思就是：雷达看到正前方2米处有个东西
            radar_point = PointStamped()
            radar_point.header.frame_id = "radar_link"
            radar_point.header.stamp = rospy.Time(0) # 获取最新的可用变换
            radar_point.point.x = 2.0
            radar_point.point.y = 0.0
            radar_point.point.z = 0.0
            
            # 3. 见证奇迹的时刻：做变换！
            # 问：这个点在 "base_link" 里坐标是多少？
            base_point = listener.transformPoint("base_link", radar_point)
            
            print("-------------------------------------------")
            print("雷达坐标系下的点: (%.2f, %.2f, %.2f)" % (radar_point.point.x, radar_point.point.y, radar_point.point.z))
            print("基座坐标系下的点: (%.2f, %.2f, %.2f)" % (base_point.point.x, base_point.point.y, base_point.point.z))
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("TF 转换失败")
            continue

        rate.sleep()
```

---

### 第三步：实操运行与可视化（最重要的一步）

给脚本加权限：
```bash
chmod +x src/tf_broadcaster.py src/tf_listener.py
```

#### 1. 启动终端 A (Core)
```bash
roscore
```

#### 2. 启动终端 B (Broadcaster)
```bash
rosrun learning_tf tf_broadcaster.py
```
*此时，你的电脑里已经有了一个虚拟的旋转雷达。*

#### 3. 启动终端 C (Rviz 可视化)
```bash
rosrun rviz rviz
```
**配置 Rviz (手把手教):**
1.  左边 **Displays** 栏，`Global Options` -> `Fixed Frame` 改为 **`base_link`** (如果没有手动输入)。
2.  左下角 **Add** 按钮 -> 选择 **`TF`**。
3.  你会看到屏幕中间有两个坐标轴，其中一个(`radar_link`) 正在围着另一个(`base_link`) **旋转**！
4.  在 TF 属性里勾选 `Show Names`，能清楚看到名字。

#### 4. 启动终端 D (Listener)
```bash
rosrun learning_tf tf_listener.py
```
**观察输出结果：**
你会发现，虽然输入一直是 `(2.0, 0, 0)`（相对于雷达），但输出的基座坐标一直在变！
*   当雷达转到基座正前方时，基座坐标可能是 `(3.0, 0, 0.5)` (1米安装偏移 + 2米雷达探测)。
*   当雷达转到基座左边时，基座坐标的 Y 值会变得很大。

#### 5. 命令行工具查看 (Debug神器)
教他们两个命令，以后查错全靠它：

*   **查看坐标树结构：**
    ```bash
    rosrun tf view_frames
    evince frames.pdf
    ```
    *解释：这能证明 `base_link` 是父，`radar_link` 是子。*

*   **实时打印两个坐标系的关系：**
    ```bash
    rosrun tf tf_echo base_link radar_link
    ```
    *解释：你会看到 Translation (1, 0, 0.5) 基本不变，但 Rotation 在疯狂跳动。*

---


## 总结
> **“ROS的TF系统就是机器人的‘空间感知神经’。它把分散的、各自为政的传感器数据（眼睛、耳朵），通过数学变换，统一到了同一个世界坐标系（大脑）中，让机器人知道‘我在哪’以及‘障碍物在哪’。”**

