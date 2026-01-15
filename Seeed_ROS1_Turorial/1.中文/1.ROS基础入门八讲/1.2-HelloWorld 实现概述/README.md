# HelloWorld 实现概述

ROS 编程主要使用 C++ 和 Python。大多数程序都可以用这两种语言实现。每个教程都将展示 C++ 和 Python 的示例，允许用户选择最适合自己的实现方式。

在 ROS 中，不同语言的通用实现流程是相似的。以 HelloWorld 程序为例，步骤如下：

1. 创建工作空间。
2. 创建软件包。
3. 编辑源文件。
4. 编辑配置文件。
5. 编译并执行。

C++ 和 Python 的主要区别在于步骤 3 和 4。以下章节详细介绍了这两种实现方式的共有步骤，并针对 C++ 和 Python 设有专门的部分。

## HelloWorld (C++ 版本)

1. **创建并初始化工作空间**
    ```bash
    mkdir -p <workspace_name>/src
    cd <workspace_name>
    catkin_make
    ```
    例如：
    ```bash
    mkdir -p seeed_ws/src
    cd seeed_ws
    catkin_make
    ```

2. **创建 ROS 软件包并添加依赖项**
      ```bash
      cd src
      catkin_create_pkg <package_name> roscpp rospy std_msgs
      ```
    例如：
      ```bash
      cd src
      catkin_create_pkg hello_world roscpp rospy std_msgs
      ```
3. **编辑源文件**

    导航到软件包的 `src` 目录并创建一个新的 C++ 源文件（例如 `hello.cpp`）：
    ```bash
    cd ~/<workspace_name>/src/<package_name>/src
    touch hello.cpp
    ```
    例如：
    ```bash
    cd ~/seeed_ws/src/hello_world/src
    touch hello.cpp
    ```
    将以下代码复制到 `hello.cpp` 中：
    ```cpp
    #include "ros/ros.h"

    int main(int argc, char *argv[])
    {
        ros::init(argc, argv, "hello");
        ros::NodeHandle n;
        ROS_INFO("Hello World!");

        return 0;
    }
    ```
      <p align="center">
        <a href="https://wiki.seeedstudio.com/reComputer_Intro/">
        <img src="./images/hello_world_c.png" alt="J3010" width="600" height="auto">
        </a>
      </p>

4. **编辑 `CMakeLists.txt`**
      
      在软件包的 `CMakeLists.txt` 末尾添加以下内容：
      ```cmake
      add_executable(<node_name> src/hello.cpp)
      target_link_libraries(<node_name> ${catkin_LIBRARIES})
      ```
      例如：
      ```cmake
      add_executable(hello src/hello.cpp)
      target_link_libraries(hello ${catkin_LIBRARIES})
      ```
      <p align="center">
      <a href="https://wiki.seeedstudio.com/reComputer_Intro/">
      <img src="./images/cmakelists_dir.png" alt="J3010"width="600" height="auto">
      </a>
      </p>

      <p align="center">
      <a href="https://wiki.seeedstudio.com/reComputer_Intro/">
      <img src="./images/cmakelists.png" alt="J3010" width="600" height="auto">
      </a>
      </p>

      **注意：此处提到的 `CMakeLists.txt` 文件位于创建的软件包目录中，而不是工作空间根目录中。**

5. **编译工作空间**
      ```bash
      cd <workspace_name>
      catkin_make
      ```
      例如：
      ```bash
        cd ~/seeed_ws
        catkin_make
      ```
6. **运行程序**
    
    打开一个终端并启动 ROS 核心（roscore）：
    ```bash
    roscore
    ```
    打开另一个终端，加载工作空间环境并运行节点：
    ```bash
    cd <workspace_name>
    source devel/setup.bash
    rosrun <package_name> hello
    ```
    例如：
    ```bash
    cd ~/seeed_ws
    source devel/setup.bash
    rosrun hello_world hello
    ```
    <p align="center">
      <a href="https://wiki.seeedstudio.com/reComputer_Intro/">
      <img src="./images/hello_world_result_c.png" alt="J3010" width="600" height="auto">
      </a>
    </p>

您应该会看到输出：`Hello World!`

## HelloWorld (Python 版本)

1. **创建并初始化工作空间**
    ```bash
    mkdir -p <workspace_name>/src
    cd <workspace_name>
    catkin_make
    ```
    例如：
    ```bash
    mkdir -p seeed_ws/src
    cd seeed_ws
    catkin_make
    ```

2. **创建 ROS 软件包并添加依赖项**
    ```bash
    cd src
    catkin_create_pkg <package_name> roscpp rospy std_msgs
    ```
    例如：
      ```bash
      cd src
      catkin_create_pkg hello_world roscpp rospy std_msgs
      ```
3. **添加 `scripts` 目录并创建 Python 文件**

    导航到软件包目录，创建一个 `scripts` 目录，并创建一个新的 Python 文件（例如 `hello.py`）：

    例如：
    ```bash
    mkdir ~/seeed_ws/src/hello_world/scripts
    cd ~/seeed_ws/src/hello_world/scripts
    touch hello.py
    ```

    将以下代码粘贴到文件中：
    ```python
    #!/usr/bin/env python

    import rospy

    if __name__ == "__main__":
        rospy.init_node("hello")
        rospy.loginfo("Hello World!")
    ```

4. **添加可执行权限**
    ```bash
    sudo chmod +x hello.py
    ```

5. **编辑 `CMakeLists.txt`**
    
    在软件包的 `CMakeLists.txt` 末尾添加以下内容：
    ```cmake
    catkin_install_python(PROGRAMS scripts/hello.py
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    ```
    <p align="center">
      <a href="https://wiki.seeedstudio.com/reComputer_Intro/">
      <img src="./images/cmakelists_dir.png" alt="J3010"  width="600" height="auto">
      </a>
    </p>
6. **编译工作空间**
    ```bash
    cd <workspace_name>
    catkin_make
    ```
    例如：
    ```bash
    cd ~/seeed_ws
    catkin_make
    ```
7. **运行程序**
    打开一个终端并启动 ROS 核心：
    ```bash
    roscore
    ```
    打开另一个终端，加载工作空间环境并运行节点：
    ```bash
    cd <workspace_name>
    source devel/setup.bash
    rosrun <package_name> hello.py
    ```
    例如：
    ```bash
    cd ~/seeed_ws
    source devel/setup.bash
    rosrun hello_world hello.py
    ```
    您应该会看到输出：`Hello World!`

## 注意
为了更方便地加载工作空间设置文件，可以将其添加到您的 `.bashrc` 中：
```bash
echo "source ~/<workspace_name>/devel/setup.bash" >> ~/.bashrc
```
这将确保在每次打开新终端时，系统都会自动加载该工作空间的环境。