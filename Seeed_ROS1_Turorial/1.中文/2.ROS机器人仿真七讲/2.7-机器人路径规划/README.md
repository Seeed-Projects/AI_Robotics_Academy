

# 路径规划：机器人如何规划出一条“生路”？

如果说 SLAM 是地图绘制员，AMCL 是导游，那么 **move_base** 就是坐在驾驶位上的“老司机”。它不仅要看地图（全局规划），还要时刻盯着路面避开突然出现的行人（本地规划）。

## 1. move_base 逻辑架构：老司机的双重思考

`move_base` 并不是简单地画一条线。它的内部其实运行着两套逻辑：

1.  **全局规划 (Global Planner)：**
    *   **类比：** 手机地图导航。它根据已有的静态地图，计算出从当前点到目标点的一条理论最短路径。
    *   **特点：** 眼光长远，但不看实时变化（它不知道现在路中间停了一辆车）。
2.  **本地规划 (Local Planner)：**
    *   **类比：** 司机的眼睛。在执行全局路径时，它会盯着传感器（雷达）的实时数据。
    *   **特点：** 眼光短浅（只看周围几米），但反应极快。如果前面突然出现障碍物，它会立即扭转方向盘绕行。

---

## 2. move_base 的底层逻辑流图

了解数据是怎么在 `move_base` 内部流动的，是排查问题的关键：

<p align="center">
  <a>
    <img src="./images/move_base_logic_zh.png" width="600" height="auto">
  </a>
</p>

---

## 3. 代价地图 (Costmap)：机器人眼里的危险程度

**为什么 SLAM 的黑白图不能直接用？**
SLAM 地图只有“有墙”和“没墙”。但对机器人来说，即使没墙的地方，离墙太近也是危险的（万一转弯时屁股蹭到墙呢？）。

### A. 代价地图的“图层”逻辑
就像 Photoshop 一样，代价地图是由多个图层叠加而成的：
1.  **静态层 (Static Layer)：** 来自 SLAM 画好的黑白图，是基础。
2.  **障碍物层 (Obstacle Layer)：** 来自实时雷达数据。如果路中间突然多了一个垃圾桶，这一层会立刻更新。
3.  **膨胀层 (Inflation Layer)：** **最关键的一层。** 它在所有障碍物周围画上“警戒区”，防止机器人贴边走。

### B. 膨胀区算法：机器人的“个人空间”
代价地图通过数值（0-255）来描述危险程度。请想象障碍物是一个发热源，离它越近越烫手：

*   **致命障碍 (Lethal, 254)：** 障碍物本体。绝对不能踩。
*   **内切障碍 (Inscribed, 253)：** 距离障碍物太近，机器人转个身就会撞上。
*   **膨胀区 (Inflation)：** 代价从 252 逐渐降到 0。机器人会尽量避开高代价区域，选择“最凉快”（代价最低）的路径走。
*   **自由区 (Free, 0)：** 安全，随便走。

---

## 4. 关键交互：Action（动作）机制

`move_base` 并没有使用普通的 Topic 通讯来接收目标，而是使用了 **ActionLib**。

**为什么要用 Action 而不是 Topic？**
*   **Topic (异步发布)：** 发送目标后，你不知道机器人收没收到，也不知道它现在走到哪了，更不知道它最后撞墙了还是成功了。
*   **Action (动作机制)：** 
    1.  **目标 (Goal)：** 发送要去的地方。
    2.  **反馈 (Feedback)：** 机器人实时汇报：“我离终点还有 2 米... 1.5 米...”。
    3.  **结果 (Result)：** 结束后告诉你：“我安全到达了”或“我被堵死了，去不了”。

---

## 5. 参数配置避坑指南（小白预警）

在接下来的实验中，你会遇到很多 `.yaml` 配置文件，其中最容易出错的是：

1.  **机器人的半径 (Robot Radius)：** 
    *   如果设得太小，机器人会疯狂蹭墙。
    *   如果设得太大，机器人会觉得所有门口都太窄，进不去屋子。
2.  **膨胀半径 (Inflation Radius)：** 
    *   建议设为机器人半径的 **2-3 倍**。
3.  **最大速度/加速度：** 
    *   如果设得太高，机器人会在停下来时产生巨大的惯性，导致定位丢失或撞墙。

---

## 6. 总结：路径规划的流程

1.  **看地图：** 获取全局代价地图（Static + Inflation）。
2.  **定线路：** 全局规划器画出一条连接起终点的长线。
3.  **看眼前：** 本地规划器参考雷达数据，获取本地代价地图。
4.  **发指令：** 计算出当前的线速度和角速度，发送给 `cmd_vel`。
5.  **纠偏：** 如果本地发现全局线路被新障碍物堵死了，触发“重规划”（Re-plan）。



## 7. 动手实际体验
安装依赖：`sudo apt install ros-noetic-navigation`

#### 1. 配置导航参数
在 `~/catkin_这是整个导航教程的“终极关卡”。通过前面的学习，我们已经准备好了地图、定位算法和路径规划逻辑，现在我们要把它们全部整合在一起。

以下是为你优化后的**《7. 动手实际体验：让机器人自主驰骋》**。

---

## 7. 终极体验：配置、启动与自主导航

这部分是整个教程中配置最密集的地方，也是新手最容易“卡关”的地方。我们将通过 4 个配置文件定义机器人的**避障逻辑**、**行驶范围**和**动力性能**。

### 1. 核心配置文件解析 (YAML)
请在 `~/catkin_ws/src/robot_modeling` 下新建 `param` 文件夹。这些文件就像是给“老司机”move_base 设定的驾驶规则。

#### A. `costmap_common_params.yaml`（公用参数）
**逻辑：** 无论全局还是局部地图，机器人的**身体尺寸**和**雷达探测距离**是不变的。
```yaml
# 障碍物探测距离 (2.5m内实时避障)
obstacle_range: 2.5
# 传感器清理范围 (3.0m内如果没有障碍物，则认为路面干净)
raytrace_range: 3.0

# 机器人足迹 (轮廓坐标)：假设机器人是 40cm x 24cm 的长方形
# [左前, 左后, 右后, 右前] 坐标原点在中心
footprint: [[-0.2, -0.12], [-0.2, 0.12], [0.2, 0.12], [0.2, -0.12]]

# 膨胀半径：给障碍物画一圈 0.55m 的警戒区
inflation_radius: 0.55

# 传感器配置
observation_sources: laser_scan_sensor
laser_scan_sensor: {sensor_frame: laser_link, data_type: LaserScan, topic: scan, marking: true, clearing: true}
```

#### B. `local_costmap_params.yaml`（局部代价地图）
**逻辑：** 设置一个以机器人为中心、跟随机器人移动的“滚动窗口”，用于实时避开动态障碍物。
```yaml
local_costmap:
  global_frame: odom          # 局部规划基于里程计
  robot_base_frame: base_footprint
  update_frequency: 5.0       # 更新频率越高，避障越灵敏
  publish_frequency: 2.0
  static_map: false           # 局部地图不依赖静态图
  rolling_window: true        # 开启滚动窗口模式
  width: 4.0                  # 局部观察范围 4m x 4m
  height: 4.0
  resolution: 0.05
```

#### C. `global_costmap_params.yaml`（全局代价地图）
**逻辑：** 这种地图覆盖整个已知世界，用于计算长距离的最优路径。
```yaml
global_costmap:
  global_frame: map           # 全局规划基于地图坐标系
  robot_base_frame: base_footprint
  update_frequency: 2.0
  static_map: true            # 必须基于 SLAM 建好的静态地图
```

#### D. `base_local_planner_params.yaml`（本地规划器参数）
**逻辑：** 设定机器人的动力学极限（最高时速、加速度），防止机器人因为太快而翻车。
```yaml
TrajectoryPlannerROS:
  # 速度限制
  max_vel_x: 0.5              # 最大前进速度 0.5m/s
  min_vel_x: 0.1
  max_vel_theta: 1.0          # 最大旋转速度
  min_in_place_vel_theta: 0.4 # 原地转动的最小速度

  # 加速度限制
  acc_lim_theta: 3.2
  acc_lim_x: 2.5

  holonomic_robot: false      # 是否为全向轮机器人？（普通两轮/四轮选false）
```

---

### 2. 编写导航集成 Launch 文件
这个 Launch 文件是“总指挥”，它将地图服务器、AMCL 定位、move_base 规划器一并拉起。

```xml
<launch>
    <!-- 1. 加载地图 -->
    <arg name="map_file" default="$(find robot_modeling)/maps/my_map.yaml"/>
    <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />

    <!-- 2. 启动 AMCL (定位) -->
    <include file="$(find robot_modeling)/launch/amcl.launch" />

    <!-- 3. 启动 Move Base (路径规划) -->
    <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
        <!-- 注意：costmap_common_params 需要分别加载到全局和局部命名空间 -->
        <rosparam file="$(find robot_modeling)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find robot_modeling)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
        
        <rosparam file="$(find robot_modeling)/param/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find robot_modeling)/param/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find robot_modeling)/param/base_local_planner_params.yaml" command="load" />
    </node>
</launch>
```

---

### 3. 自主导航实操指南

#### 第一步：运行环境
```bash
# 终端 1: 启动 Gazebo 仿真环境
roslaunch robot_modeling gazebo_world.launch

# 终端 2: 启动导航总 Launch
roslaunch robot_modeling nav.launch
```

#### 第二步：Rviz 视觉配置（这是最酷的部分）
在 Rviz 中点击 **Add** 添加以下组件，并观察它们的含义：
1.  **Map:** Topic 选择 `/map`（黑白静态地图）。
2.  **Map:** Topic 选择 `/move_base/global_costmap/costmap`（彩色全局代价地图）。**你会看到墙体周围有一圈彩色的阴影（膨胀区）。**
3.  **Path:** Topic 选择 `/move_base/NavFnROS/plan`（显示的绿线就是规划出的路径）。
4.  **RobotModel & LaserScan:** 实时观测机器人状态。

#### 第三步：下达指令
1.  **2D Pose Estimate:** 初始定位。点一下按钮，在地图上给机器人指定当前的位置。
2.  **2D Nav Goal:** 发送目标。在地图任意空闲点点一下，按住并拖动箭头方向。

#### 第四步：观察机器人的“思考”
*   **全局绿线：** 一旦设定目标，你会看到一条绿线瞬间连通起终点。
*   **动态避障：** 尝试在 Gazebo 里往小车前面扔一个方块（障碍物），你会发现 Rviz 里的局部代价地图立刻变红，小车会立刻重新规划路径绕开这个方块。

---

## 4. 结语：恭喜你，ROS 玩家！
到现在为止，你已经完成了从“手画 URDF”到“机器人自主避障导航”的全流程。

*   **URDF** 给了它身体；
*   **Gmapping** 给了它记忆；
*   **AMCL** 给了它空间感；
*   **Move_base** 给了它大脑。

**进阶方向：** 尝试修改 `base_local_planner_params.yaml` 里的速度参数，看看机器人是变得更灵活了，还是更容易撞墙了？这就是调参工程师的日常！

<p align="center">
  <a>
    <img src="./images/nav.png" width="600" height="auto">
    <br>
    <em>最终导航效果：绿线为全局路径，机器人正顺着绿线自主规划并绕开障碍物。</em>
  </a>
</p>