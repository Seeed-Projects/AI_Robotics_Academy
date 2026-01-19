**ROS 1 -> ROS 2 Core Changes:**
1.  **Build System**: Changed from `catkin_make` to `colcon build`.
2.  **Package Types**: ROS 2 recommends separating C++ and Python into different package types (`ament_cmake` vs `ament_python`), which is the biggest structural difference from ROS 1.
3.  **No Master**: The `roscore` step has been removed.
4.  **Configuration Files**: Python packages no longer use `CMakeLists.txt`; instead, they use standard Python `setup.py` to configure entry points.

# 1.2 - HelloWorld Realization Overview

ROS 2 programming primarily uses C++ and Python. In ROS 2, for better dependency management and build efficiency, it is generally recommended to choose different **Build Types** based on the programming language: C++ packages use `ament_cmake`, while Python packages use `ament_python`.

This tutorial will demonstrate how to create a C++ package and a Python package respectively to implement HelloWorld.

The main workflow is as follows:
1.  Create a Colcon Workspace.
2.  Create a Package (specify the build type).
3.  Edit the source files.
4.  Edit the configuration files (`CMakeLists.txt` or `setup.py`).
5.  Build and execute.

---

## HelloWorld (C++ Version)

### 1. Create and Initialize the Workspace
ROS 2 uses `colcon` as the build tool.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```
This will generate three directories in the workspace: `build`, `install`, and `log`.

To easily manage files, you can enter the following command in the terminal to open the VSCode editor.
```bash
cd ~/ros2_ws
code .
```

<p align="center">
  <a>
    <img src="./images/vscode.png" alt="ros"  width="600" height="auto">
  </a>
</p>
    
Then press `Ctrl+Shift+` on your keyboard to open the terminal within VSCode.

<p align="center">
  <a>
    <img src="./images/vscode_terminal.png" alt="ros"  width="600" height="auto">
  </a>
</p>
    

### 2. Create a C++ Package
We use the `ros2 pkg create` command and specify the build type as `ament_cmake`. Enter the following command in the terminal:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --dependencies rclcpp --node-name hello_node hello_cpp
```
*   `--build-type ament_cmake`: Specifies this as a C++ package.
*   `--dependencies rclcpp`: Automatically adds the ROS 2 C++ client library dependency.
*   `--node-name hello_node`: Automatically creates a source file named `hello_node.cpp` (we will modify it manually below, but this parameter is convenient).
*   `hello_cpp`: The name of the package.

### 3. Edit the Source File
Navigate to the `src` directory of the package. If you used `--node-name` in the previous step, there is already a `hello_node.cpp` inside. We can edit it directly or create a new one.

File path: `~/ros2_ws/src/hello_cpp/src/hello_node.cpp`

Modify the content to:
```cpp
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    // 1. Initialize ROS 2 communication
    rclcpp::init(argc, argv);

    // 2. Create a node
    auto node = rclcpp::Node::make_shared("hello_world_node");

    // 3. Print log (In ROS 2, use RCLCPP_INFO instead of ROS_INFO)
    // get_logger() retrieves the node's logger
    RCLCPP_INFO(node->get_logger(), "Hello World from C++!");

    // 4. Shutdown ROS 2 communication
    rclcpp::shutdown();
    return 0;
}
```

<p align="center">
  <a>
  <img src="./images/hello_node_c.png" alt="cmake_ros2" width="600" height="auto">
  </a>
</p>


### 4. Edit `CMakeLists.txt`
Usually, `ros2 pkg create` generates most of the content automatically, but we need to ensure the executable is correctly compiled and **installed**.

Open `~/ros2_ws/src/hello_cpp/CMakeLists.txt` and confirm it contains the following key sections (usually after `find_package`):

```cmake
# Add the executable
add_executable(hello_node src/hello_node.cpp)

# Add target dependencies (This step is crucial to link rclcpp)
ament_target_dependencies(hello_node rclcpp)

# Installation rules (ROS 2 must explicitly install targets, otherwise ros2 run won't find them)
install(TARGETS
  hello_node
  DESTINATION lib/${PROJECT_NAME}
)
```
<p align="center">
  <a>
  <img src="./images/cmakelists_cpp_ros2.png" alt="cmake_ros2" width="600" height="auto">
  </a>
</p>

### 5. Build the Workspace
Return to the workspace root directory to compile.
```bash
cd ~/ros2_ws
colcon build --packages-select hello_cpp
```
*   `--packages-select`: Only compiles the specified package to save time. If omitted, all packages are compiled.

### 6. Run the Program
**ROS 2 does not require running roscore.**

Open a terminal, source the environment, and run:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run hello_cpp hello_node
```
You should see the output:
`[INFO] [1678945200.123456789] [hello_world_node]: Hello World from C++!`

<p align="center">
  <a>
  <img src="./images/hello_c.png" alt="cmake_ros2" width="600" height="auto">
  </a>
</p>


---

## HelloWorld (Python Version)

The Python package structure in ROS 2 differs from ROS 1; it uses `setup.py` for management, following standard Python package specifications.

### 1. Create a Python Package
Create a new package in the `src` directory, specifying the build type as `ament_python`. Enter the following command in the terminal:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --dependencies rclpy --node-name hello_node hello_py
```
*   `hello_py`: The name of the package.

### 2. Edit the Python Source File
The source code for a ROS 2 Python package is located in a subdirectory with the same name as the package: `~/ros2_ws/src/hello_py/hello_py/hello_node.py`.

Edit that file:
```python
import rclpy
from rclpy.node import Node

def main(args=None):
    # 1. Initialize ROS 2
    rclpy.init(args=args)
    
    # 2. Create a node
    # In ROS 2, inheriting from the Node class is not strictly required, 
    # but using an object-oriented approach is recommended for future scalability.
    node = Node("hello_world_node_py")
    
    # 3. Print log
    node.get_logger().info("Hello World from Python!")
    
    # 4. Destroy the node and shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<p align="center">
  <a>
  <img src="./images/hello_node_py.png" alt="cmake_ros2" width="600" height="auto">
  </a>
</p>


### 3. Edit `setup.py` (Configure Entry Points)
This is the core configuration file for ROS 2 Python packages. Open `~/ros2_ws/src/hello_py/setup.py`.

Locate the `entry_points` field and configure the mapping between the executable name and the Python function:

```python
    entry_points={
        'console_scripts': [
            # Format: 'executable_name = package_name.file_name:function_name'
            'hello_node_py = hello_py.hello_node:main',
        ],
    },
```
<p align="center">
  <a>
  <img src="./images/setup_py.png" alt="setup_py" width="600" height="auto">
  </a>
</p>

### 4. Build the Workspace
Even for Python, it is recommended to use `colcon` for building in ROS 2 so that scripts are correctly installed into the system path.

```bash
cd ~/ros2_ws
# --symlink-install is a lifesaver for Python development; it uses symbolic links for installation.
# This allows changes to Python code to take effect immediately without recompiling.
colcon build --packages-select hello_py --symlink-install
```

### 5. Run the Program
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run hello_py hello_node_py
```
You should see the output:
`[INFO] [1678945211.987654321] [hello_world_node_py]: Hello World from Python!`

<p align="center">
  <a>
  <img src="./images/hello_py.png" alt="setup_py" width="600" height="auto">
  </a>
</p>

---

## Summary of Key Differences (ROS 1 vs ROS 2)

| Feature | ROS 1 (Noetic) | ROS 2 (Humble) |
| :--- | :--- | :--- |
| **Build Tool** | `catkin_make` | `colcon build` |
| **Master Node** | Requires `roscore` | **Not required** (Decentralized) |
| **Environment Sourcing** | `source devel/setup.bash` | `source install/setup.bash` |
| **Python Config** | `CMakeLists.txt` | `setup.py` |
| **Node Handle** | `ros::NodeHandle` | `rclcpp::Node` / `self` |
| **Log Output** | `ROS_INFO(...)` | `RCLCPP_INFO(logger, ...)` |

## 💡 Development Tips

To avoid manually sourcing the environment file every time you open a terminal, you can add the ROS 2 environment settings to your `.bashrc`:

```bash
# 1. Add ROS 2 system environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 2. Add your workspace environment
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

source ~/.bashrc
```