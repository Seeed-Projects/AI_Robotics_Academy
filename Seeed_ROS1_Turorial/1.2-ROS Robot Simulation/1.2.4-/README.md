

### 第二阶段：仿真与建图 (SLAM)

#### 1. 启动 Gazebo 环境
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
*   你应该能看到小车出现在一个复杂的办公楼环境中。
*   你会看到雷达在发射蓝色的光线。

#### 2. 开启 SLAM 建图
我们需要运行 `gmapping` 算法。确保已安装：`sudo apt install ros-noetic-gmapping`。

新建 `launch/slam_gmapping.launch`：

```xml
<launch>
    <!-- 启动 Gmapping 节点 -->
    <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
        <param name="base_frame" value="base_footprint"/>
        <param name="odom_frame" value="odom"/>
        <param name="map_frame"  value="map"/>
        <param name="scan_topic" value="scan"/>
        <!-- 稍微调大一点更新间隔，降低CPU占用 -->
        <param name="map_update_interval" value="2.0"/>
        <param name="delta" value="0.05"/>
    </node>

    <!-- 启动 Rviz 用于观看建图过程 -->
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find robot_modeling)/rviz/slam.rviz"/>
</launch>
```

**体验 2 (建图实战)：**
1.  **终端 1:** 启动环境 `roslaunch robot_modeling gazebo_world.launch`
2.  **终端 2:** 启动算法 `roslaunch robot_modeling slam_gmapping.launch`
    *   *注：第一次 Rviz 需要配置：Fixed Frame 选 map，Add Map (Topic: /map), Add RobotModel, Add LaserScan。配置好后保存为 `slam.rviz`。*
3.  **终端 3:** 键盘控制 `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
4.  **操作：** 控制小车在房间里转悠。你会看到 Rviz 里的灰色未知区域逐渐变成白色（路）和黑色（墙）。
5.  **保存地图：** 地图建好后，运行：
    ```bash
    mkdir -p ~/catkin_ws/src/robot_modeling/maps
    rosrun map_server map_saver -f ~/catkin_ws/src/robot_modeling/maps/my_map
    ```

---

### 第三阶段：自主导航 (Navigation)

现在有了地图，我们要让小车自己去目的地。这需要 `move_base` 包。
安装依赖：`sudo apt install ros-noetic-navigation`

#### 1. 配置导航参数
在 `src/robot_modeling` 下新建文件夹 `param`，并创建 4 个 `yaml` 配置文件。这是最劝退新手的步骤，我为你提供了**最简配置**。

*   **`costmap_common_params.yaml`** (公用参数)
    ```yaml
    obstacle_range: 2.5
    raytrace_range: 3.0
    footprint: [[-0.2, -0.12], [-0.2, 0.12], [0.2, 0.12], [0.2, -0.12]] # 车身轮廓
    inflation_radius: 0.55
    observation_sources: laser_scan_sensor
    laser_scan_sensor: {sensor_frame: laser_link, data_type: LaserScan, topic: scan, marking: true, clearing: true}
    ```

*   **`local_costmap_params.yaml`** (局部地图 - 避障用)
    ```yaml
    local_costmap:
      global_frame: odom
      robot_base_frame: base_footprint
      update_frequency: 5.0
      publish_frequency: 2.0
      static_map: false
      rolling_window: true
      width: 6.0
      height: 6.0
      resolution: 0.05
    ```

*   **`global_costmap_params.yaml`** (全局地图 - 规划路径用)
    ```yaml
    global_costmap:
      global_frame: map
      robot_base_frame: base_footprint
      update_frequency: 2.0
      static_map: true
    ```

*   **`base_local_planner_params.yaml`** (轨迹规划器)
    ```yaml
    TrajectoryPlannerROS:
      max_vel_x: 0.5
      min_vel_x: 0.1
      max_vel_theta: 1.0
      min_in_place_vel_theta: 0.4
      acc_lim_theta: 3.2
      acc_lim_x: 2.5
      holonomic_robot: false
    ```

#### 2. 编写导航 Launch 文件
新建 `launch/nav.launch`：

```xml
<launch>
    <!-- 1. 加载地图 -->
    <arg name="map_file" default="$(find robot_modeling)/maps/my_map.yaml"/>
    <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />

    <!-- 2. 启动 AMCL (定位) -->
    <node pkg="amcl" type="amcl" name="amcl" output="screen">
        <param name="base_frame_id" value="base_footprint"/>
        <param name="odom_frame_id" value="odom"/>
        <param name="scan_topic"    value="scan"/>
    </node>

    <!-- 3. 启动 Move Base (路径规划) -->
    <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
        <rosparam file="$(find robot_modeling)/param/costmap_common_params.yaml" command="load" ns="global_costmap" />
        <rosparam file="$(find robot_modeling)/param/costmap_common_params.yaml" command="load" ns="local_costmap" />
        <rosparam file="$(find robot_modeling)/param/local_costmap_params.yaml" command="load" />
        <rosparam file="$(find robot_modeling)/param/global_costmap_params.yaml" command="load" />
        <rosparam file="$(find robot_modeling)/param/base_local_planner_params.yaml" command="load" />
    </node>

    <!-- 4. Rviz -->
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find robot_modeling)/rviz/nav.rviz" />
</launch>
```

#### 3. 终极体验：自主导航
这是整个教程的高光时刻。

1.  **终端 1:** 启动 Gazebo `roslaunch robot_modeling gazebo_world.launch`
2.  **终端 2:** 启动导航 `roslaunch robot_modeling nav.launch`
3.  **Rviz 配置 (nav.rviz):**
    *   Fixed Frame: `map`
    *   Add Map (Topic: `/map`) -> 这是你刚才建的图。
    *   Add RobotModel。
    *   Add LaserScan。
    *   Add Path (Topic: `/move_base/NavFnROS/plan`) -> 显示规划的绿线。
    *   Add PoseArray (Topic: `/particlecloud`) -> 显示红色箭头群（AMCL粒子）。

**如何玩：**
1.  **初始化定位：** 点击 Rviz 顶部的 **"2D Pose Estimate"**，在地图上绿色箭头画出机器人现在大致的位置和朝向。你会看到红色的粒子群聚拢到车周围。
2.  **发布目标：** 点击 Rviz 顶部的 **"2D Nav Goal"**，在地图上任意点个位置，按住鼠标拖动设置朝向。
3.  **见证奇迹：**
    *   一条绿色的路径线会出现。
    *   小车会自动避开障碍物，向目标点移动。
    *   Gazebo 里的小车也在同步移动。

---

### 讲义总结
通过这一章，学生完成了从“造车”到“开车”再到“自动驾驶”的全过程：
1.  **URDF+Gazebo:** 让机器人拥有了实体。
2.  **Gmapping:** 让机器人拥有了记忆（地图）。
3.  **AMCL+Move_base:** 让机器人拥有了小脑（定位与规划）。