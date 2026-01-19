# 1.6 - ROS 2 Runtime Management

## Managing Nodes with Python Launch Files

In ROS 2, the Launch system has undergone a significant refactoring. While it still supports XML and YAML formats, the official recommendation is to use **Python scripts (`.launch.py`)** to write launch files. This means launch files are no longer just configuration; they are real programs. You can leverage all Python features (such as loops, conditionals, and dynamic path retrieval) to manage complex robot startup workflows.

### 1. Launch File Basic Structure

A standard ROS 2 Python launch file typically contains the following structure:
1.  Import necessary modules (`launch`, `launch_ros`).
2.  Define the `generate_launch_description` function.
3.  Return a `LaunchDescription` object containing the `Node` actions or other operations to be executed.

#### Basic Example: Starting Turtlesim
Create a file named `start_turtle.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',          # Package name
            executable='turtlesim_node',  # Executable name (replaces 'type' in ROS 1)
            name='sim',                   # Node name (renaming)
            output='screen',              # Log output location
            emulate_tty=True,             # Enables colored logs (optional)
            parameters=[{'background_r': 255}] # Setting parameters
        ),
    ])
```

### 2. Node Configuration Details

In `launch_ros.actions.Node`, you can configure various node attributes, which correspond to ROS 1 XML tag attributes:

| ROS 1 XML Attribute | ROS 2 Python Parameter | Description |
| :--- | :--- | :--- |
| `pkg="pkg_name"` | `package='pkg_name'` | Package where the node is located |
| `type="exe_name"` | `executable='exe_name'` | Executable filename |
| `name="node_name"` | `name='node_name'` | Name of the node at runtime |
| `ns="namespace"` | `namespace='namespace'` | Namespace |
| `output="screen"` | `output='screen'` | Log output destination |
| `<remap>` tag | `remappings=[('/old', '/new')]` | Topic/Service remapping |
| `<param>` tag | `parameters=[{'key': val}, config_file]` | Parameter settings |
| `args="--arg"` | `arguments=['--arg']` | Command-line arguments passed to the node |

### 3. Including Other Launch Files (`Include`)

This is equivalent to the `<include>` tag in ROS 1. In ROS 2, `get_package_share_directory` is typically combined with it to locate file paths dynamically.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the share directory path of another package
    other_pkg_dir = get_package_share_directory('other_package')
    
    # Construct the launch file path
    launch_file_path = os.path.join(other_pkg_dir, 'launch', 'other.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path),
            launch_arguments={'arg_name': 'arg_value'}.items(), # Passing arguments
        )
    ])
```

### 4. Runtime Arguments (`DeclareLaunchArgument`)

Equivalent to the `<arg>` tag in ROS 1, used to dynamically pass configurations when launching a file from the command line.

**Launch File (`arg_demo.launch.py`):**
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Declare the argument
    bg_color_arg = DeclareLaunchArgument(
        'bg_color', default_value='255', description='Background color'
    )

    # 2. Retrieve the argument value
    bg_color = LaunchConfiguration('bg_color')

    return LaunchDescription([
        bg_color_arg,
        LogInfo(msg=['Background color is: ', bg_color]),
    ])
```

**Command-line call:**
```bash
ros2 launch my_pkg arg_demo.launch.py bg_color:=100
```

---

## ROS 2 Workspace Overlay

The overlay mechanism in ROS 2 is similar to ROS 1. Priority is determined by the order in which you `source` the setup files of different workspaces.

### Scenario
Suppose you have two workspaces:
1.  **Underlay**: `/opt/ros/humble` (System installation)
2.  **Overlay**: `~/ros2_ws` (Contains a custom modified version of `turtlesim`)

### Operational Steps

1.  **Build the workspace:**
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

2.  **Set environment variables (`~/.bashrc`):**
    ROS 2 uses `install/setup.bash` and automatically handles underlay dependencies (if the underlay was sourced during build). Typically, you only need to source the topmost workspace, but for safety, you can source them in order:

    ```bash
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ```

3.  **Verify the overlay:**
    ```bash
    ros2 pkg prefix turtlesim
    ```
    **Result:** It should output `~/ros2_ws/install/turtlesim`, indicating that the system prioritized your custom package.

---

## Node Name Conflicts and Namespaces (CLI & Launch)

In ROS 2, handling duplicate node names and namespaces is mainly achieved through **Launch file** configuration or **command-line arguments (`--ros-args`)**.

### Scenario
We need to start two Turtlesim nodes. If we simply run `ros2 run turtlesim turtlesim_node` twice, the second one will not "kick out" the first as in ROS 1 (ROS 2 allows duplicate names, but communication will be chaotic). To maintain control, we usually want them to have different names.

### Method 1: Using the Command Line (`--ros-args`)

ROS 2 introduces a unified parameter flag `--ros-args` (or `-r`) to handle remapping.

1.  **Renaming a Node (`__node`):**
    ```bash
    # Syntax: --ros-args -r __node:=NewName
    ros2 run turtlesim turtlesim_node --ros-args -r __node:=my_turtle
    ```

2.  **Setting a Namespace (`__ns`):**
    ```bash
    # Syntax: --ros-args -r __ns:=/NewNamespace
    ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/room1
    ```

3.  **Combined Use:**
    ```bash
    ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/room1 -r __node:=turtle_a
    ```

### Method 2: Using a Launch File

In a Launch file, this is very intuitive:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # First node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='room1',
            name='turtle_a'
        ),
        # Second node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='room2',
            name='turtle_b'
        )
    ])
```

---

## Topic Remapping

Topic remapping is a key technology for connecting different nodes. For example, redirecting the `/cmd_vel` published by a keyboard control node to a specific turtle `/turtle1/cmd_vel`.

### 1. Using the Command Line (`ros2 run`)

Syntax format: `-r original_topic:=new_topic`

**Example: Controlling a Turtle with a Keyboard**

Typically, `turtlesim` subscribes to `/turtle1/cmd_vel`, while the keyboard node `teleop_twist_keyboard` publishes to `/cmd_vel`.

*   **Option A: Modify the keyboard node's publishing topic**
    ```bash
    # Start the turtle
    ros2 run turtlesim turtlesim_node
    
    # Start the keyboard, remapping /cmd_vel to /turtle1/cmd_vel
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/turtle1/cmd_vel
    ```

*   **Option B: Modify the turtle node's subscribing topic**
    ```bash
    # Start the keyboard
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    
    # Start the turtle, remapping /turtle1/cmd_vel to /cmd_vel
    ros2 run turtlesim turtlesim_node --ros-args -r /turtle1/cmd_vel:=/cmd_vel
    ```

### 2. Using a Launch File

Use the `remappings` parameter, which accepts a list containing `(original_name, new_name)` tuples.

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

### 3. Topic Naming Rules in Code

When writing publishers/subscribers in ROS 2 code, the topic name resolution rules are as follows:

*   **Global Names**: Start with `/`.
    *   `create_publisher(msg, "/chatter", 10)` -> Topic name is `/chatter`.
    *   Completely ignores the node's namespace.

*   **Relative Names**: Do not start with `/`.
    *   `create_publisher(msg, "chatter", 10)`
    *   If the node is in the `/` (root) namespace: Topic name is `/chatter`.
    *   If the node is in the `/ns` namespace: Topic name is `/ns/chatter`.

*   **Private Names**: Start with `~/` (rarely used directly in ROS 2; more often handled through parameters).
    *   ROS 2 prefers using parameters or relative paths to manage internal node data. Directly hardcoding `~/` is uncommon, though `NodeOptions` supports automatic node name prefixing.

**Best Practice:**
When writing code, try to use **relative names** (e.g., `cmd_vel` instead of `/cmd_vel`). This allows users to easily place the node into different namespaces via Launch files or the command line, increasing code reusability.