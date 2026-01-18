# 1.5 - 常用 ROS 2 命令

在一个机器人系统中，可能会有几个到几十个节点同时运行。每个节点都有一个唯一的名称，它们通过话题（Topic）、服务（Service）、参数（Parameter）等进行通信。

ROS 2 提供了一个统一的入口命令 `ros2` 来管理整个系统。常用的命令类别如下：

- **ros2 node**：管理节点
- **ros2 topic**：管理话题
- **ros2 service**：管理服务
- **ros2 interface**：管理接口（替代了 ROS 1 的 rosmsg 和 rossrv）
- **ros2 param**：管理参数
- **ros2 pkg**：管理功能包
- **ros2 bag**：数据录制与回放

## 1. ros2 node (节点管理)
`ros2 node` 用于检索 ROS 2 节点的信息。

- `ros2 node list`：列出所有活动节点。
- `ros2 node info <node_name>`：打印有关节点的详细信息（包括它订阅/发布的话题、提供的服务等）。

**示例：**
```bash
# 列出所有节点
ros2 node list

# 查看 /turtlesim 节点的详细信息
ros2 node info /turtlesim
```

## 2. ros2 topic (话题管理)
`ros2 topic` 是最常用的调试工具，用于检查流媒体数据。

- `ros2 topic list`：列出当前活动的话题。
    - 常用参数：`-t` (显示消息类型)。
- `ros2 topic echo <topic_name>`：将话题数据打印到屏幕上。
- `ros2 topic info <topic_name>`：显示话题的详细信息（类型、发布者/订阅者数量）。
- `ros2 topic pub <topic_name> <msg_type> <args>`：手动向话题发布数据。
- `ros2 topic hz <topic_name>`：显示话题的发布频率。
- `ros2 topic bw <topic_name>`：显示话题的带宽占用。

**示例：**

1.  **查看话题列表及类型：**
    ```bash
    ros2 topic list -t
    ```

2.  **手动发布消息：**
    *注意：ROS 2 命令行发布消息需要使用 YAML 格式，通常用双引号包裹，内部数据结构用大括号。*
    ```bash
    # 格式：ros2 topic pub --once <话题> <类型> "<数据>"
    ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
    ```

## 3. ros2 interface (接口管理 - 核心变化)
在 ROS 1 中，查看 `.msg` 用 `rosmsg`，查看 `.srv` 用 `rossrv`。
**在 ROS 2 中，它们统称为“接口 (Interface)”，统一使用 `ros2 interface` 命令管理。**

- `ros2 interface list`：列出系统中所有可用的 msg、srv 和 action。
- `ros2 interface show <interface_name>`：显示接口的定义（字段结构）。
- `ros2 interface package <package_name>`：列出指定包下的所有接口。

**示例：**

1.  **查看标准字符串消息的结构：**
    ```bash
    ros2 interface show std_msgs/msg/String
    # 输出: string data
    ```

2.  **查看 Twist 速度消息的结构：**
    ```bash
    ros2 interface show geometry_msgs/msg/Twist
    ```

## 4. ros2 service (服务管理)
`ros2 service` 用于调用和检查“请求-响应”式的服务。

- `ros2 service list`：列出所有活动的服务。
    - 常用参数：`-t` (显示服务类型)。
- `ros2 service type <service_name>`：查看服务的类型。
- `ros2 service find <type_name>`：查找使用指定类型的所有服务。
- `ros2 service call <service_name> <service_type> <args>`：手动调用服务。

**示例：**

**调用小乌龟的重生服务：**
```bash
# 格式：ros2 service call <服务名> <类型> "<数据>"
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.2, name: 'turtle2'}"
```

## 5. ros2 param (参数管理)
ROS 2 中的参数归属于具体的节点，没有全局参数服务器。

- `ros2 param list`：列出所有节点及其参数。
- `ros2 param get <node_name> <param_name>`：获取指定节点的参数值。
- `ros2 param set <node_name> <param_name> <value>`：动态修改参数值。
- `ros2 param dump <node_name>`：将节点的参数保存为 YAML 文件。
- `ros2 param load <node_name> <file_path>`：从 YAML 文件加载参数。

**示例：**

1.  **修改小乌龟背景颜色：**
    ```bash
    # 查看参数列表
    ros2 param list /turtlesim
    
    # 获取当前背景里的红色分量
    ros2 param get /turtlesim background_r
    
    # 修改为 150 (颜色会立即变化)
    ros2 param set /turtlesim background_r 150
    ```

## 6. 其他实用命令

### ros2 pkg (功能包工具)
- `ros2 pkg create`：创建新功能包。
- `ros2 pkg list`：列出已安装的功能包。
- `ros2 pkg executable <package_name>`：列出包内的可执行文件（节点）。

### ros2 doctor (环境诊断)
这是 ROS 2 特有的神器。如果系统运行不正常，可以用它来检查环境配置。

```bash
ros2 doctor
```
如果一切正常，它会输出 `All required rosdeps installed successfully` 等确认信息。

---

## 总结：ROS 1 与 ROS 2 命令对照表

为了方便您快速迁移，以下是常用命令的对照关系：

| 功能 | ROS 1 命令 | **ROS 2 命令** |
| :--- | :--- | :--- |
| **运行节点** | `rosrun <pkg> <exe>` | `ros2 run <pkg> <exe>` |
| **节点列表** | `rosnode list` | `ros2 node list` |
| **节点信息** | `rosnode info` | `ros2 node info` |
| **话题列表** | `rostopic list` | `ros2 topic list` |
| **发布消息** | `rostopic pub` | `ros2 topic pub` |
| **查看消息** | `rostopic echo` | `ros2 topic echo` |
| **服务列表** | `rosservice list` | `ros2 service list` |
| **调用服务** | `rosservice call` | `ros2 service call` |
| **参数列表** | `rosparam list` | `ros2 param list` |
| **设置参数** | `rosparam set` | `ros2 param set` |
| **查看 Msg** | `rosmsg show` | **`ros2 interface show`** |
| **查看 Srv** | `rossrv show` | **`ros2 interface show`** |
| **创建包** | `catkin_create_pkg` | `ros2 pkg create` |