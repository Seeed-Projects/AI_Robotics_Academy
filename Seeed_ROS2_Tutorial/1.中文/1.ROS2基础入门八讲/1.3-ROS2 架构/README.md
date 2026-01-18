# 1.3 - ROS 2 架构与文件系统

## ROS 2 文件系统

ROS 2 的工作空间结构与 ROS 1 略有不同，最显著的变化是引入了 `install` 目录来替代 `devel` 目录，并新增了 `log` 目录。

硬盘上的 ROS 2 工作空间结构组织如下：

```text
WorkSpace --- 自定义工作空间 (例如 ~/ros2_ws)

    |--- build: 编译空间。存储中间构建文件。每个软件包都会在此目录下有一个独立的子文件夹。
    |--- install: 安装空间。这是 ROS 2 的核心变化。
    |        |--- setup.bash: 环境设置脚本（运行程序前必须 source 这个文件）。
    |        |--- <package_name>: 编译生成的可执行文件、库、头文件和 Launch 文件都存放在这里。
    |--- log: 日志空间。Colcon 构建过程的日志和 ROS 2 运行时的日志文件。
    |--- src: 源码空间 (与 ROS 1 类似)

        |-- <package_name>: ROS 2 功能包
            |-- package.xml: 包清单文件（定义依赖、版本等）。
            |-- CMakeLists.txt: (C++ 包专用) 编译规则配置。
            |-- setup.py: (Python 包专用) Python 包的安装配置。
            |-- src: (C++ 包) 存放 .cpp 源文件。
            |-- <package_name>: (Python 包) 存放 .py 源文件。
            |-- include: 头文件目录。
            |-- msg/srv/action: 自定义通信接口定义目录。
            |-- launch: 存放 Python (.py) 或 XML (.xml) 格式的启动文件。
            |-- config: 存放 .yaml 参数配置文件。
```

### package.xml (包清单)

`package.xml` 定义了功能包的元数据和依赖关系。**注意：ROS 2 使用 `format="3"`**。

C++ 包与 Python 包在 `package.xml` 中的主要区别在于 `<buildtool_depend>` 标签。

**示例 (C++ 包 `package.xml`)：**
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>hello_cpp</name>
  <version>0.0.0</version>
  <description>ROS 2 C++ example package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache-2.0</license>

  <!-- 构建工具依赖：C++ 使用 ament_cmake -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- 依赖项 -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

如果是 Python 包，`<buildtool_depend>` 通常是 `ament_python`（尽管有时隐式包含），且 `<export><build_type>` 会是 `ament_python`。

### 构建配置文件 (区分语言)

在 ROS 1 中，所有包都依赖 `CMakeLists.txt`。但在 ROS 2 中，根据语言不同，配置文件也不同：

#### 1. C++ 包: `CMakeLists.txt`
使用 `ament_cmake` 构建系统。

```cmake
cmake_minimum_required(VERSION 3.8)
project(hello_cpp)

# 1. 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 2. 添加可执行文件
add_executable(hello_node src/hello_node.cpp)
ament_target_dependencies(hello_node rclcpp)

# 3. 安装规则 (ROS 2 必须显式安装目标)
install(TARGETS
  hello_node
  DESTINATION lib/${PROJECT_NAME}
)

# 4. 标记包结束
ament_package()
```

#### 2. Python 包: `setup.py`
使用标准的 Python `setuptools` 机制。

```python
from setuptools import setup

package_name = 'hello_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    description='ROS 2 Python example',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 格式: 可执行名 = 包名.模块名:函数名
            'hello_node_py = hello_py.hello_node:main',
        ],
    },
)
```

## ROS 2 常用命令 (CLI)

ROS 2 的命令行工具进行了统一，全部以 `ros2` 开头，格式为 `ros2 <verb> <sub-verb>`。

| 操作 | ROS 1 命令 | **ROS 2 命令** |
| :--- | :--- | :--- |
| **创建包** | `catkin_create_pkg <name> <deps>` | `ros2 pkg create <name> --build-type <type> --dependencies <deps>` |
| **编译** | `catkin_make` | `colcon build` (需在工作空间根目录) |
| **列出包** | `rospack list` | `ros2 pkg list` |
| **查找包路径**| `rospack find <pkg>` | `ros2 pkg prefix <pkg>` |
| **运行节点** | `rosrun <pkg> <node>` | `ros2 run <pkg> <executable>` |
| **运行 Launch**| `roslaunch <pkg> <file>` | `ros2 launch <pkg> <file>` |
| **核心服务** | `rocore` | **(无)** ROS 2 自动发现，无需 Master |
| **节点列表** | `rosnode list` | `ros2 node list` |
| **话题列表** | `rostopic list` | `ros2 topic list` |

**实用技巧：**
*   **安装包：** `sudo apt install ros-humble-<package_name>` (将下划线替换为短横线，例如 `ros-humble-turtlesim`)。
*   **自动补全：** ROS 2 的命令支持 Tab 键自动补全，非常方便。

## ROS 2 计算图 (Computational Graph)

ROS 2 的计算图依然展示了节点（Nodes）和话题（Topics）之间的拓扑关系。但其底层不再依赖 Master 节点，而是基于 DDS（数据分发服务）进行去中心化的自动发现。

### 计算图演示

我们将使用小乌龟仿真来演示 ROS 2 的计算图。

1.  **运行小乌龟节点：**
    打开终端 1：
    ```bash
    ros2 run turtlesim turtlesim_node
    ```

2.  **运行键盘控制节点：**
    打开终端 2：
    ```bash
    ros2 run turtlesim turtle_teleop_key
    ```

3.  **查看计算图：**
    打开终端 3，输入：
    ```bash
    rqt_graph
    ```
    *(如果提示未找到命令，请安装：`sudo apt install ros-humble-rqt ros-humble-rqt-graph`)*

<p align="center">
  <a>
    <img src="./images/computatioinal.png" width="600" height="auto" alt="rqt_graph">
  </a>
</p>

### 图解说明
*   **椭圆 (Nodes):** 代表运行的进程（如 `/turtlesim` 和 `/teleop_turtle`）。
*   **矩形 (Topics):** 代表传输的数据通道（如 `/turtle1/cmd_vel`）。
*   **箭头:** 表示数据的流向。你可以清晰地看到 `teleop_turtle` 节点发布了速度指令，通过 `/turtle1/cmd_vel` 话题，流向了 `turtlesim` 节点。

---

**下一节预告：** 在掌握了基本的架构和文件系统后，我们将进入 **2. ROS 2 通信机制**，深入学习节点之间如何通过 Topic（话题）、Service（服务）和 Interface（接口）进行数据交换。