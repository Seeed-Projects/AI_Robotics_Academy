# ROS 运行管理

## 使用 Launch 文件管理 ROS 节点

ROS 中的 Launch 文件是 XML 格式的文件，用于高效地启动和管理多个 ROS 节点。本节将介绍 Launch 文件中可用的各种标签，包括它们的属性和使用案例。

### `<launch>` 标签

`<launch>` 标签是每个 Launch 文件的根标签，充当所有其他标签的容器。

#### 1. 属性
- **deprecated="弃用声明"**  
  向用户指示当前的 Launch 文件已弃用。

#### 2. 子标签
- Launch 文件中的所有其他标签都是 `<launch>` 标签的子元素。

#### 示例：
```xml
<launch>
    <!-- 其他标签在此处 -->
</launch>
```

### `<node>` 标签

`<node>` 标签用于指定要启动的 ROS 节点。它是 Launch 文件中最常用的标签之一。请注意，`roslaunch` 命令并不保证节点会按照声明的顺序启动，因为节点的启动过程是多线程的。

#### 1. 属性
- **pkg="功能包名称"**  
  指定节点所属的功能包。

- **type="节点类型"**  
  节点的类型，对应于可执行文件的名称。

- **name="节点名称"**  
  节点在 ROS 网络拓扑中的名称。

- **args="xxx xxx xxx"**（可选）  
  向节点传递参数。

- **machine="机器名称"**  
  指定要在哪台机器上启动该节点。

- **respawn="true | false"**（可选）  
  确定节点退出后是否应自动重启。

- **respawn_delay="N"**（可选）  
  如果 `respawn` 设置为 true，则定义节点重启前延迟 N 秒。

- **required="true | false"**（可选）  
  指示该节点是否为关键节点。如果设置为 true，一旦该节点退出，整个 `roslaunch` 进程都将终止。

- **ns="命名空间"**（可选）  
  在指定的命名空间内启动节点。

- **clear_params="true | false"**（可选）  
  在节点启动前清除其私有命名空间中的所有参数。

- **output="log | screen"**（可选）  
  确定日志输出的位置：发送到日志文件或屏幕。默认为 `log`。

#### 2. 子标签
- **env**：用于设置环境变量。
- **remap**：用于重映射话题或服务名称。
- **rosparam**：用于设置参数。
- **param**：用于设置参数。

#### 示例：
```xml
<launch>
    <node name="node1" pkg="my_package" type="node_executable" output="screen" respawn="true" respawn_delay="5">
        <param name="param_name" value="param_value"/>
        <remap from="/old_topic" to="/new_topic"/>
    </node>
</launch>
```

### `<include>` 标签

`<include>` 标签用于将另一个 XML 格式的 Launch 文件包含到当前文件中。这实现了配置的模块化和可重用性。

#### 1. 属性
- **file="$(find package_name)/path/to/file.launch"**  
  指定要包含的 Launch 文件的路径。

- **ns="命名空间"**（可选）  
  在指定的命名空间下包含该文件。

#### 2. 子标签
- **env**：用于设置环境变量。
- **arg**：用于向被包含的 Launch 文件传递参数。

#### 示例：
```xml
<launch>
    <include file="$(find my_package)/launch/another_launch_file.launch" ns="my_namespace"/>
</launch>
```

### `<remap>` 标签

`<remap>` 标签用于重映射 ROS 话题或服务名称。这对于避免名称冲突或在不同节点间标准化名称非常有用。

#### 1. 属性
- **from="xxx"**  
  原始话题或服务名称。

- **to="yyy"**  
  话题或服务的新名称。

#### 2. 子标签
- 无

#### 示例：
```xml
<launch>
    <node name="node1" pkg="my_package" type="node_executable">
        <remap from="/old_topic" to="/new_topic"/>
    </node>
</launch>
```

### `<param>` 标签

`<param>` 标签用于在 ROS 参数服务器上设置参数。参数源可以直接在标签中指定，也可以从外部文件加载。当在 `<node>` 标签内部使用时，参数将被设置在节点的私有命名空间中。

#### 1. 属性
- **name="命名空间/参数名称"**  
  参数的名称，可以包含命名空间。

- **value="xxx"**（可选）  
  定义参数的值。如果省略，则必须指定外部文件作为参数源。

- **type="str | int | double | bool | yaml"**（可选）  
  指定参数的类型。如果不指定，`roslaunch` 将尝试根据值推断类型：
  - 包含 `.` 的数字被解析为浮点数 (double)。
  - 字符串 "true" 和 "false" 被解析为布尔值（不区分大小写）。
  - 其他所有内容均被解析为字符串。

#### 2. 子标签
- 无

#### 示例：
```xml
<launch>
    <node name="node1" pkg="my_package" type="node_executable">
        <param name="namespace/param_name" value="param_value" type="str"/>
    </node>
</launch>
```

### `<rosparam>` 标签

`<rosparam>` 标签允许从 YAML 文件加载参数、将参数导出到 YAML 文件或删除参数。当在 `<node>` 标签内部使用时，参数被视为私有的。

#### 1. 属性
- **command="load | dump | delete"**（可选，默认为 `load`）  
  指定要执行的操作：从文件加载参数、导出参数到文件或删除参数。

- **file="$(find package_name)/path/to/file.yaml"**  
  指定要加载或导出参数的 YAML 文件。

- **param="参数名称"**  
  参数的名称。

- **ns="命名空间"**（可选）  
  为参数指定命名空间。

#### 2. 子标签
- 无

#### 示例：
```xml
<launch>
    <rosparam file="$(find my_package)/config/params.yaml" command="load" ns="my_namespace"/>
</launch>
```

## `<group>` 标签

`<group>` 标签用于对节点和其他标签进行分组，它允许对整个组应用命名空间或其他设置。

#### 1. 属性
- **ns="命名空间"**（可选）  
  对组内的所有节点和参数应用命名空间。

- **clear_params="true | false"**（可选）  
  在启动该组之前，清除该组命名空间中的所有参数。请谨慎使用，因为这可能会移除关键参数。

#### 2. 子标签
- 除了 `<launch>` 标签外的任何标签都可以作为 `<group>` 的子标签。

#### 示例：
```xml
<launch>
    <group ns="my_namespace" clear_params="true">
        <node name="node1" pkg="my_package" type="node_executable"/>
        <node name="node2" pkg="my_package" type="node_executable"/>
    </group>
</launch>
```

### `<arg>` 标签

`<arg>` 标签用于定义可以在运行时传递给 Launch 文件的动态参数，类似于函数参数。这增加了 Launch 文件的灵活性。

#### 1. 属性
- **name="参数名称"**  
  参数的名称。

- **default="默认值"**（可选）  
  指定参数的默认值。

- **value="值"**（可选）  
  指定参数的具体值。不能与 `default` 同时使用。

- **doc="描述"**  
  提供参数的描述。

#### 2. 子标签
- 无

#### 3. 示例
带有参数语法的 Launch 文件 `hello.launch`：

```xml
<launch>
    <arg name="robot_name" default="my_robot"/>
    <param name="robot_name" value="$(arg robot_name)"/>
</launch>
```

带参数传递的命令行调用：

```bash
roslaunch hello.launch robot_name:=robot_value
```

## ROS 工作空间覆盖 (Workspace Overlay)

想象一下，你有两个自定义工作空间，工作空间 A 和工作空间 B，它们都包含一个名为 `turtlesim` 的功能包。此外，系统内置的工作空间也有一个名为 `turtlesim` 的功能包。当你调用 `turtlesim` 功能包时，系统会使用哪一个？

### 实现步骤

#### 步骤 0：创建工作空间 A 和 B
首先，创建两个独立的工作空间 A 和 B。在每个工作空间中，创建一个名为 `turtlesim` 的功能包。

#### 步骤 1：修改 `~/.bashrc` 文件
将以下行添加到您的 `~/.bashrc` 文件中，以加载两个工作空间的设置文件：

```bash
source /home/user/path/to/workspaceA/devel/setup.bash
source /home/user/path/to/workspaceB/devel/setup.bash
```

将 `/home/user/path/to/` 替换为您工作空间的实际路径。

#### 步骤 2：加载环境变量
打开新终端并运行以下命令以加载更新后的环境变量：

```bash
source ~/.bashrc
```

#### 步骤 3：检查 ROS 环境变量
要验证 ROS 功能包路径，请运行：

```bash
echo $ROS_PACKAGE_PATH
```

**结果：** 输出将按以下顺序显示路径：工作空间 B → 工作空间 A → 系统内置工作空间。

#### 步骤 4：调用 `turtlesim` 功能包
现在，运行以下命令导航到 `turtlesim` 功能包：

```bash
roscd turtlesim
```

**结果：** 您将被引导至工作空间 B 中的 `turtlesim` 功能包。

## 处理 ROS 节点名称冲突

### 场景
在 ROS 中，每个节点都有一个名称，该名称在节点初始化期间定义。在 C++ 中，这是通过 `ros::init(argc, argv, "node_name");` API 完成的，而在 Python 中，则是通过 `rospy.init_node("node_name")` 完成的。在 ROS 网络拓扑中，节点名称必须是唯一的，因为如果多个节点共享相同的名称，会导致调用混乱。具体来说，如果启动了一个重名的节点，原本存在的同名节点将自动关闭。但如果你需要运行同一个节点的多个实例，或需要处理名称冲突该怎么办？

ROS 提供了两种策略来处理这种情况：**命名空间 (namespaces)** 和 **名称重映射 (name remapping)**。

- **命名空间**为节点名称添加前缀。
- **名称重映射**为节点名称分配一个别名。

这两种策略都可以解决节点名称冲突，并可以通过几种方式实现：

1. 使用 `rosrun` 命令。
2. 通过 Launch 文件。
3. 在节点的代码中实现。

本节将演示如何使用这三种方法来避免节点名称冲突。

### 示例场景
让我们启动两个 `turtlesim_node` 节点。如果你打开两个终端并在不进行任何更改的情况下直接启动节点，启动第二个节点时，第一个节点将会关闭。你会看到一条警告信息：

```plaintext
[ WARN] [1578812836.351049332]: Shutdown request received.
[ WARN] [1578812836.351207362]: Reason given for shutdown: [new node registered with same name]
```

由于节点不能共享相同的名称，我们将探索几种策略来解决这个问题。

## 使用 `rosrun` 设置命名空间和重映射

### 1. 使用 `rosrun` 设置命名空间

您可以使用以下语法为节点设置命名空间：

```bash
rosrun 功能包名称 节点名称 __ns:=/新命名空间
```

#### 示例：
```bash
rosrun turtlesim turtlesim_node __ns:=/xxx
rosrun turtlesim turtlesim_node __ns:=/yyy
```

通过这些命令，两个节点都可以正常运行。

#### 结果：
使用 `rosnode list` 检查节点：

```plaintext
/xxx/turtlesim
/yyy/turtlesim
```

### 2. 使用 `rosrun` 重映射节点名称

您还可以重映射节点的名称（相当于给它起个别名），语法如下：

```bash
rosrun 功能包名称 节点名称 __name:=新名称
```

#### 示例：
```bash
rosrun turtlesim turtlesim_node __name:=t1
rosrun turtlesim turtlesim_node __name:=t2
```

通过这些命令，两个节点将以新名称同时运行。

#### 结果：
使用 `rosnode list` 检查节点：

```plaintext
/t1
/t2
```

### 3. 使用 `rosrun` 组合命名空间和名称重映射

您可以同时结合这两种技术，在设置命名空间的同时重映射节点名称：

```bash
rosrun 功能包名称 节点名称 __ns:=/新命名空间 __name:=新名称
```

#### 示例：
```bash
rosrun turtlesim turtlesim_node __ns:=/xxx __name:=tn
```

#### 结果：
使用 `rosnode list` 检查节点：

```plaintext
/xxx/tn
```

或者，您也可以在启动节点之前通过环境变量设置命名空间：

```bash
export ROS_NAMESPACE=xxxx
```

## 使用 Launch 文件设置命名空间和重映射

在 Launch 文件中，`<node>` 标签包含两个重要的属性：`name` 和 `ns`。它们分别用于名称重映射和设置命名空间。使用 Launch 文件来处理命名空间和名称重映射非常简单。

### 1. Launch 文件示例

以下是在 Launch 文件中设置命名空间和名称重映射的方法：

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="t1" />
    <node pkg="turtlesim" type="turtlesim_node" name="t2" />
    <node pkg="turtlesim" type="turtlesim_node" name="t1" ns="hello"/>
</launch>
```

在此示例中，`name` 属性是必填的，而 `ns` 是可选的。

### 2. 运行 Launch 文件

运行 Launch 文件，然后使用 `rosnode list` 查看结果：

```plaintext
/t1
/t2
/hello/t1
```

## 在代码中设置命名空间和重映射

如果您正在实现自定义节点，则可以更灵活地直接在代码中设置命名空间和名称重映射。

### 1. C++ 实现：名称重映射

您可以使用以下代码设置名称别名：

```cpp
ros::init(argc, argv, "zhangsan", ros::init_options::AnonymousName);
```

#### 执行效果：
这将在节点名称后附加一个时间戳，确保其唯一性。

### 2. C++ 实现：设置命名空间

您可以像这样在代码中直接设置命名空间：

```cpp
std::map<std::string, std::string> map;
map["__ns"] = "xxxx";
ros::init(map, "wangqiang");
```

#### 执行效果：
这为节点设置了命名空间，允许其在不发生冲突的情况下运行。

### 3. Python 实现：名称重映射

在 Python 中，您可以通过以下代码实现类似的功能：

```python
rospy.init_node("lisi", anonymous=True)
```
---
## ROS 中的话题名称重映射

在 ROS 中，话题名称重映射允许您在不修改节点代码的情况下，更改节点订阅或发布的话题名称。这在集成多个需要通过不同话题名称通信的节点时特别有用。ROS 中主要有三种重映射话题名称的方法：

1. 使用 `rosrun` 命令。
2. 通过 Launch 文件。
3. 直接在 C++ 或 Python 代码中修改。

### 使用 `rosrun` 重映射话题

使用 `rosrun` 重映射话题名称的语法是：

```bash
rosrun 功能包名称 节点名称 原始话题名称:=新话题名称
```

### 示例：集成 `teleop_twist_keyboard` 与 `turtlesim`

有两种方法可以在键盘控制节点 `teleop_twist_keyboard` 和显示节点 `turtlesim` 之间建立通信：

#### 1. 方案 1：重映射 `teleop_twist_keyboard` 的话题

在这种方法中，我们将键盘控制节点的话题重映射为 `/turtle1/cmd_vel`。

- **启动键盘控制节点：**

  ```bash
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/turtle1/cmd_vel
  ```

- **启动 turtlesim 显示节点：**

  ```bash
  rosrun turtlesim turtlesim_node
  ```

两个节点将通过 `/turtle1/cmd_vel` 话题正确通信。

#### 2. 方案 2：重映射 `turtlesim` 的话题

或者，我们可以将 `turtlesim` 节点的话题重映射为 `/cmd_vel`。

- **启动键盘控制节点：**

  ```bash
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  ```

- **启动 turtlesim 显示节点：**

  ```bash
  rosrun turtlesim turtlesim_node /turtle1/cmd_vel:=/cmd_vel
  ```

两个节点将通过 `/cmd_vel` 话题正确通信。

### 使用 Launch 文件重映射话题

您也可以在 Launch 文件中重映射话题。语法如下：

```xml
<node pkg="功能包名称" type="节点类型" name="节点名称">
    <remap from="原始话题" to="新话题" />
</node>
```

### 示例：使用 Launch 文件集成 `teleop_twist_keyboard` 与 `turtlesim`

同样有两种方案：

#### 1. 方案 1：重映射 `teleop_twist_keyboard` 的话题

我们将键盘控制节点的话题重映射为 `/turtle1/cmd_vel`。

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="t1" />
    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="key">
        <remap from="/cmd_vel" to="/turtle1/cmd_vel" />
    </node>
</launch>
```

两个节点将正常通信。

#### 2. 方案 2：重映射 `turtlesim` 的话题

我们将 `turtlesim` 节点的话题重映射为 `/cmd_vel`。

```xml
<launch>
    <node pkg="turtlesim" type="turtlesim_node" name="t1">
        <remap from="/turtle1/cmd_vel" to="/cmd_vel" />
    </node>
    <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py" name="key" />
</launch>
```

两个节点将正常通信。

### 在代码中重映射话题

ROS 中的话题名称受节点的命名空间、节点的名称以及话题自身名称的影响。话题名称通常可以分为三类：

1. **全局 (Global)：** 话题名称是绝对的，以 `/` 开头，使其独立于节点的命名空间。
2. **相对 (Relative)：** 话题名称是相对的，不以 `/` 开头，这意味着它在节点的命名空间内被解析。
3. **私有 (Private)：** 话题名称是私有的，以 `~` 开头，这意味着它是相对于节点的私有命名空间进行解析的。

让我们通过 C++ 和 Python 的示例来探索这些概念。

### 1. C++ 实现

#### 示例准备：

1. **初始化具有名称的节点：**

   ```cpp
   ros::init(argc, argv, "hello");
   ```

2. **设置不同类型的话题名称。**
3. **启动节点时传递 `__ns:=xxx` 参数。**
4. **节点启动后，使用 `rostopic` 检查话题信息。**

#### 全局话题名称

全局话题名称以 `/` 开头，独立于节点名称或命名空间。

- **示例 1：**

  ```cpp
  ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter", 1000);
  ```

  **结果：** `/chatter`

- **示例 2：**

  ```cpp
  ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter/money", 1000);
  ```

  **结果：** `/chatter/money`

#### 相对话题名称

相对话题名称不以 `/` 开头，并相对于节点的命名空间进行解析。

- **示例 1：**

  ```cpp
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ```

  **结果：** `xxx/chatter`

- **示例 2：**

  ```cpp
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter/money", 1000);
  ```

  **结果：** `xxx/chatter/money`

#### 私有话题名称

私有话题名称以 `~` 开头，相对于节点的私有命名空间进行解析。

- **示例 1：**

  ```cpp
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ```

  **结果：** `/xxx/hello/chatter`

- **示例 2：**

  ```cpp
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<std_msgs::String>("chatter/money", 1000);
  ```

  **结果：** `/xxx/hello/chatter/money`

- **特殊情况：** 使用 `~` 时，如果话题名称以 `/` 开头，则该话题名称被视为绝对名称。

  ```cpp
  ros::NodeHandle nh("~");
  ros::Publisher pub = nh.advertise<std_msgs::String>("/chatter/money", 1000);
  ```

  **结果：** `/chatter/money`

### Python 实现

#### 示例准备：

1. **初始化具有名称的节点：**

   ```python
   rospy.init_node("hello")
   ```

2. **设置不同类型的话题名称。**
3. **启动节点时传递 `__ns:=xxx` 参数。**
4. **节点启动后，使用 `rostopic` 检查话题信息。**

#### 全局话题名称

全局话题名称以 `/` 开头，独立于节点名称或命名空间。

- **示例 1：**

  ```python
  pub = rospy.Publisher("/chatter", String, queue_size=1000)
  ```

  **结果：** `/chatter`

- **示例 2：**

  ```python
  pub = rospy.Publisher("/chatter/money", String, queue_size=1000)
  ```

  **结果：** `/chatter/money`

### 相对话题名称

相对话题名称不以 `/` 开头，并相对于节点的命名空间进行解析。

- **示例 1：**

  ```python
  pub = rospy.Publisher("chatter", String, queue_size=1000)
  ```

  **结果：** `xxx/chatter`

- **示例 2：**

  ```python
  pub = rospy.Publisher("chatter/money", String, queue_size=1000)
  ```

  **结果：** `xxx/chatter/money`

#### 私有话题名称

私有话题名称以 `~` 开头，相对于节点的私有命名空间进行解析。

- **示例 1：**

  ```python
  pub = rospy.Publisher("~chatter", String, queue_size=1000)
  ```

  **结果：** `/xxx/hello/chatter`

- **示例 2：**

  ```python
  pub = rospy.Publisher("~chatter/money", String, queue_size=1000)
  ```

  **结果：** `/