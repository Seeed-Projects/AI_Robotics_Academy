### ROS 架构

#### ROS 文件系统

硬盘上的 ROS 文件系统结构组织如下：

  <p align="center">
    <a href="https://wiki.seeedstudio.com/reComputer_Intro/">
    <img src="./images/filesystem.jpg" alt="J3010" width="600" height="auto">
    </a>
  </p>

```
WorkSpace --- 自定义工作空间

    |--- build: 编译空间，用于存储 CMake 和 catkin 的缓存、配置及其他中间文件。
    |--- devel: 开发空间，用于存储编译后的目标文件，包括头文件、动态和静态库、可执行文件等。
    |--- src: 源码空间

        |-- package: ROS 功能包（ROS 的基本单元），包含多个节点、库和配置文件。功能包名称应为小写，由字母、数字和下划线组成。
            |-- CMakeLists.txt: 编译规则配置，包括源文件、依赖项和目标文件。
            |-- package.xml: 软件包信息，如名称、版本、作者、依赖项等。
            |-- scripts: 存放 Python 文件的目录。
            |-- src: 存放 C++ 源文件的目录。
            |-- include: 头文件目录。
            |-- msg: 消息通信格式文件目录。
            |-- srv: 服务通信格式文件目录。
            |-- action: 动作格式文件目录。
            |-- launch: 用于一次性运行多个节点的 launch 文件目录。
            |-- config: 配置文件目录。

        |-- CMakeLists.txt: 编译的基础配置文件。
```

其中一些目录和文件已经讨论过，例如创建功能包、在 `src` 和 `scripts` 目录中编写 C++ 和 Python 文件，以及在 `launch` 目录中创建 launch 文件。`package.xml` 和 `CMakeLists.txt` 文件也已经进行了配置。其他目录将在后续教程中介绍。

#### package.xml

`package.xml` 文件定义了功能包的属性，例如名称、版本、作者、维护者和依赖项。格式如下：

```xml
<?xml version="1.0"?>
<package format="2">
  <name>hello_world</name>
  <version>0.0.0</version>
  <description>The hello_world package</description>
  <maintainer email="xuzuo@todo.todo">xuzuo</maintainer>
  <license>TODO</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <export>
  </export>
</package>
```

#### CMakeLists.txt

`CMakeLists.txt` 文件是 CMake 构建系统的输入，用于构建功能包。它包含了编译 C++ 和 Python 文件的配置，以及依赖项的定义。

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(demo01_hello_vscode)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hellow src/hello.cpp)

add_dependencies(hellow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hellow
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/hello.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

#### ROS 文件系统命令

与 ROS 文件系统交互的常用命令：

- **创建功能包：** `catkin_create_pkg <package_name> <dependency_1> <dependency_2> ...`
- **安装功能包：** `sudo apt install <package_name>`
- **删除功能包：** `sudo apt purge <package_name>`
- **列出功能包：** `rospack list`
- **查找功能包：** `rospack find <package_name>`
- **跳转到功能包：** `roscd <package_name>`
- **列出包内文件：** `rosls <package_name>`
- **搜索功能包：** `apt search <package_name>`
- **编辑包内文件：** `rosed <package_name> <file_name>`

#### 执行 ROS 命令

- **启动 ROS 核心：** `roscore`
- **运行 ROS 节点：** `rosrun <package_name> <executable_file_name>`
- **启动 ROS 文件：** `roslaunch <package_name> <launch_file_name>`

#### ROS 计算图

ROS 中的计算图代表了 ROS 系统的运行时结构，展示了不同节点之间的数据流。可以使用 `rqt_graph` 进行可视化：

```bash
rosrun rqt_graph rqt_graph
```

如果未安装：
```bash
sudo apt install ros-<distro>-rqt
sudo apt install ros-<distro>-rqt-common-plugins
```

请将 `<distro>` 替换为您对应的 ROS 版本（例如 kinetic、melodic、noetic）。

### 计算图演示

接下来，我们将使用 ROS 内置的小乌龟仿真演示计算图。

1. **运行示例：**
   按照之前的说明运行小乌龟仿真。

2. **查看计算图：**
   打开一个新终端并输入：
   ```bash
   rqt_graph
   ```
   或者
   ```bash
   rosrun rqt_graph rqt_graph
   ```

您将看到一个网络拓扑图，显示了不同节点之间的关系，如下图所示。

<p align="center">
  <a>
    <img src="./images/computatioinal.png" width="400" height="auto">
  </a>
</p>