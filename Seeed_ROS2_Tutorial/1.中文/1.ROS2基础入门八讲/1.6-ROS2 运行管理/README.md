# 1.6 - ROS 2 运行管理

## 使用 Python Launch 文件管理节点

在 ROS 2 中，Launch 系统经历了重大的重构。虽然它仍然支持 XML 和 YAML 格式，但官方强烈推荐使用 **Python 脚本 (`.launch.py`)** 来编写启动文件。这意味着 Launch 文件不再仅仅是配置，而是真正的程序，你可以利用 Python 的所有特性（如循环、条件判断、动态路径获取）来管理复杂的机器人启动流程。

### 1. Launch 文件基础结构

一个标准的 ROS 2 Python Launch 文件通常包含以下结构：
1.  导入必要的模块 (`launch`, `launch_ros`)。
2.  定义 `generate_launch_description` 函数。
3.  返回一个 `LaunchDescription` 对象，其中包含要执行的 `Node` 或其他操作。

#### 基础示例：启动小乌龟
创建一个名为 `start_turtle.launch.py` 的文件：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',          # 功能包名称
            executable='turtlesim_node',  # 可执行文件名称 (ROS1 中的 type)
            name='sim',                   # 节点名称 (重命名)
            output='screen',              # 日志输出位置
            emulate_tty=True,             # 使得日志带有颜色 (可选)
            parameters=[{'background_r': 255}] # 设置参数
        ),
    ])
```

### 2. Node 配置详解

在 `launch_ros.actions.Node` 中，我们可以配置节点的各种属性，对应 ROS 1 XML 的标签属性：

| ROS 1 XML 属性 | ROS 2 Python 参数 | 说明 |
| :--- | :--- | :--- |
| `pkg="pkg_name"` | `package='pkg_name'` | 节点所在包 |
| `type="exe_name"` | `executable='exe_name'` | 可执行文件名 |
| `name="node_name"` | `name='node_name'` | 节点运行时的名称 |
| `ns="namespace"` | `namespace='namespace'` | 命名空间 |
| `output="screen"` | `output='screen'` | 日志输出 |
| `<remap>` 标签 | `remappings=[('/old', '/new')]` | 话题/服务重映射 |
| `<param>` 标签 | `parameters=[{'key': val}, config_file]` | 参数设置 |
| `args="--arg"` | `arguments=['--arg']` | 传递给节点的命令行参数 |

### 3. 包含其他 Launch 文件 (`Include`)

相当于 ROS 1 的 `<include>` 标签。在 ROS 2 中，通常结合 `get_package_share_directory` 来动态定位文件路径。

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取另一个包的 share 目录路径
    other_pkg_dir = get_package_share_directory('other_package')
    
    # 构建 launch 文件路径
    launch_file_path = os.path.join(other_pkg_dir, 'launch', 'other.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path),
            launch_arguments={'arg_name': 'arg_value'}.items(), # 传递参数
        )
    ])
```

### 4. 运行时参数 (`DeclareLaunchArgument`)

相当于 ROS 1 的 `<arg>` 标签，用于在命令行启动 launch 文件时动态传递配置。

**Launch 文件 (`arg_demo.launch.py`):**
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 声明参数
    bg_color_arg = DeclareLaunchArgument(
        'bg_color', default_value='255', description='Background color'
    )

    # 2. 获取参数值
    bg_color = LaunchConfiguration('bg_color')

    return LaunchDescription([
        bg_color_arg,
        LogInfo(msg=['Background color is: ', bg_color]),
    ])
```

**命令行调用：**
```bash
ros2 launch my_pkg arg_demo.launch.py bg_color:=100
```

---

## ROS 2 工作空间覆盖 (Workspace Overlay)

ROS 2 的覆盖机制与 ROS 1 类似，通过 `source` 不同工作空间的设置文件来决定优先级。

### 场景
假设你有两个工作空间：
1.  **底层**：`/opt/ros/humble` (系统安装)
2.  **上层**：`~/ros2_ws` (包含自定义的 `turtlesim` 修改版)

### 操作步骤

1.  **编译工作空间：**
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

2.  **设置环境变量 (`~/.bashrc`)：**
    ROS 2 使用 `install/setup.bash`，并且会自动处理底层依赖（如果编译时已经 source 了底层）。通常只需要 source 最上层的工作空间即可，但为了保险，可以按顺序写：

    ```bash
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ```

3.  **验证覆盖：**
    ```bash
    ros2 pkg prefix turtlesim
    ```
    **结果：** 应该输出 `~/ros2_ws/install/turtlesim`，说明系统优先使用了你自定义的包。

---

## 节点名称冲突与命名空间 (CLI & Launch)

在 ROS 2 中，处理节点重名和命名空间主要通过 **Launch 文件** 配置或 **命令行参数 (`--ros-args`)** 实现。

### 场景
我们需要启动两个小乌龟节点，如果直接运行两次 `ros2 run turtlesim turtlesim_node`，虽然后者不会像 ROS 1 那样踢掉前者（ROS 2 允许重名，但通信会混乱），但为了控制，我们通常希望它们有不同的名字。

### 方法 1：使用命令行 (`--ros-args`)

ROS 2 引入了统一的参数标志 `--ros-args` (或 `-r`) 来处理重映射。

1.  **重命名节点 (`__node`)：**
    ```bash
    # 语法：--ros-args -r __node:=新名称
    ros2 run turtlesim turtlesim_node --ros-args -r __node:=my_turtle
    ```

2.  **设置命名空间 (`__ns`)：**
    ```bash
    # 语法：--ros-args -r __ns:=/新空间
    ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/room1
    ```

3.  **组合使用：**
    ```bash
    ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/room1 -r __node:=turtle_a
    ```

### 方法 2：使用 Launch 文件

在 Launch 文件中，这非常直观：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 第一个节点
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='room1',
            name='turtle_a'
        ),
        # 第二个节点
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='room2',
            name='turtle_b'
        )
    ])
```

---

## 话题重映射 (Topic Remapping)

话题重映射是连接不同节点的关键技术。例如，将键盘控制节点发出的 `/cmd_vel` 重定向给特定的小乌龟 `/turtle1/cmd_vel`。

### 1. 使用命令行 (`ros2 run`)

语法格式：`-r 原始话题:=新话题`

**示例：用键盘控制小乌龟**

通常 `turtlesim` 订阅 `/turtle1/cmd_vel`，而键盘节点 `teleop_twist_keyboard` 发布 `/cmd_vel`。

*   **方案 A：修改键盘节点的发布话题**
    ```bash
    # 启动乌龟
    ros2 run turtlesim turtlesim_node
    
    # 启动键盘，将 /cmd_vel 重映射为 /turtle1/cmd_vel
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/turtle1/cmd_vel
    ```

*   **方案 B：修改乌龟节点的订阅话题**
    ```bash
    # 启动键盘
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    
    # 启动乌龟，将 /turtle1/cmd_vel 重映射为 /cmd_vel
    ros2 run turtlesim turtlesim_node --ros-args -r /turtle1/cmd_vel:=/cmd_vel
    ```

### 2. 使用 Launch 文件

使用 `remappings` 参数，它接受一个包含 `(原始名, 新名称)` 元组的列表。

```python
Node(
    package='turtlesim',
    executable='turtlesim_node',
    name='sim',
    remappings=[
        ('/turtle1/cmd_vel', '/cmd_vel'),
        ('/turtle1/pose', '/pose'),
    ]
)
```

### 3. 代码中的话题名称规则

在 ROS 2 代码中编写发布者/订阅者时，话题名称的解析规则如下：

*   **全局名称 (Global)**: 以 `/` 开头。
    *   `create_publisher(msg, "/chatter", 10)` -> 话题名为 `/chatter`。
    *   完全忽略节点的命名空间。

*   **相对名称 (Relative)**: 不以 `/` 开头。
    *   `create_publisher(msg, "chatter", 10)`
    *   如果节点在 `/` (根空间)：话题名为 `/chatter`。
    *   如果节点在 `/ns` 命名空间：话题名为 `/ns/chatter`。

*   **私有名称 (Private)**: 以 `~/` 开头 (在 ROS 2 中较少直接使用，更多通过参数配置)。
    *   ROS 2 更倾向于使用参数或相对路径来管理节点内部数据，直接在代码中写死 `~/` 并不常见，但 `NodeOptions` 支持自动添加节点名前缀。

**最佳实践：**
在编写代码时，尽量使用**相对名称**（如 `cmd_vel` 而不是 `/cmd_vel`），这样用户可以通过 Launch 文件或命令行轻松地将其放入不同的命名空间中，增加代码的复用性。