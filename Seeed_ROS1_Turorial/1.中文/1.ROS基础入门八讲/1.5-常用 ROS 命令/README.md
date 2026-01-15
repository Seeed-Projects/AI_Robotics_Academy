# 常用 ROS 命令

在一个机器人系统中，可能会有几个到几十个节点同时运行。每个节点都有一个唯一的名称，它们通过话题（Topic）、服务（Service）、消息（Message）、参数（Parameter）等进行通信。一个常见的问题随之而来：当你需要自定义一个节点与现有的另一个节点通信时，如何获取该节点正在使用的话题和消息格式？

ROS 提供了一系列非常有用的命令行工具，用于获取不同节点的各种信息。常用的命令如下：

- **rosnode**：管理节点
- **rostopic**：管理话题
- **rosservice**：管理服务
- **rosmsg**：管理消息类型 (msg)
- **rossrv**：管理服务消息类型 (srv)
- **rosparam**：管理参数

这些命令是动态的，不同于静态的文件系统命令。在 ROS 程序启动后，通过这些命令可以动态地检索运行中的节点或参数的信息。

## rosnode
`rosnode` 用于检索 ROS 节点的信息。

- `rosnode ping`：测试到节点的连接状态
- `rosnode list`：列出所有活动节点
- `rosnode info`：打印有关节点的信息
- `rosnode machine`：列出在特定机器上运行的节点
- `rosnode kill`：终止一个节点
- `rosnode cleanup`：清除无法连接的节点

## rostopic
`rostopic` 包含用于显示 ROS 话题调试信息的命令行工具，例如发布者、订阅者、发布频率和 ROS 消息。它还包含一个实验性的 Python 库，用于动态检索话题信息并与其交互。

- `rostopic bw`：显示话题使用的带宽
- `rostopic delay`：显示带有 header 的话题延迟
- `rostopic echo`：将消息打印到屏幕
- `rostopic find`：按类型查找话题
- `rostopic hz`：显示话题的发布频率
- `rostopic info`：显示有关话题的信息
- `rostopic list`：列出所有活动话题
- `rostopic pub`：向话题发布数据
- `rostopic type`：打印话题的类型

示例：

- `rostopic list(-v)`：打印当前运行的话题名称（使用 `-v` 可显示详细信息，如发布者和订阅者的数量）。
- `rostopic pub /话题名称 消息类型 "消息内容"`：向话题发布一条消息。
- `rostopic echo /话题名称`：获取并打印话题当前发布的消息。
- `rostopic info /话题名称`：获取话题的详细信息，包括消息类型、发布者和订阅者信息。
- `rostopic hz /话题名称`：显示话题的发布频率。
- `rostopic bw /话题名称`：显示话题的带宽占用情况。

## 2.4.3 rosmsg
`rosmsg` 是一个用于显示 ROS 消息类型信息的命令行工具。

- `rosmsg show`：显示消息的描述
- `rosmsg info`：显示有关消息的详细信息
- `rosmsg list`：列出所有消息类型
- `rosmsg md5`：显示消息的 MD5 校验码
- `rosmsg package`：列出功能包中的所有消息
- `rosmsg packages`：列出所有包含消息的功能包

示例：

- `rosmsg list`：列出当前 ROS 环境中的所有消息类型。
- `rosmsg packages`：列出所有包含消息类型的功能包。
- `rosmsg package <package_name>`：列出特定功能包中的所有消息。
- `rosmsg show <msg_name>`：显示特定消息的描述。
- `rosmsg info <msg_name>`：类似于 `rosmsg show`，提供消息类型的信息。
- `rosmsg md5 <msg_name>`：生成消息的 MD5 校验码，用于数据完整性检查。

## 2.4.4 rosservice
`rosservice` 包含用于列出和查询 ROS 服务的命令行工具。

- `rosservice args`：打印服务所需的参数
- `rosservice call`：使用提供的参数调用服务
- `rosservice find`：按类型查找服务
- `rosservice info`：打印有关服务的信息
- `rosservice list`：列出所有活动服务
- `rosservice type`：打印服务的类型
- `rosservice uri`：打印服务的 ROSRPC URI

示例：

- `rosservice list`：列出所有活动服务。
- `rosservice args /服务名称`：打印特定服务所需的参数。
- `rosservice call /服务名称 "参数"`：使用提供的参数调用服务。
- `rosservice find <服务类型>`：根据消息类型查找服务。
- `rosservice info /服务名称`：获取有关服务的详细信息。
- `rosservice type /服务名称`：获取服务的类型。
- `rosservice uri /服务名称`：获取服务的 URI。

## 2.4.5 rossrv
`rossrv` 是一个用于显示 ROS 服务消息类型信息的命令行工具。其语法与 `rosmsg` 非常相似。

- `rossrv show`：显示服务消息的描述
- `rossrv info`：显示有关服务消息的详细信息
- `rossrv list`：列出所有服务消息类型
- `rossrv md5`：显示服务消息的 MD5 校验码
- `rossrv package`：列出功能包中的所有服务消息
- `rossrv packages`：列出所有包含服务消息的功能包

示例：

- `rossrv list`：列出当前 ROS 环境中的所有服务消息类型。
- `rossrv packages`：列出所有包含服务消息的功能包。
- `rossrv package <package_name>`：列出特定功能包中的所有服务消息。
- `rossrv show <srv_name>`：显示特定服务消息的描述。
- `rossrv info <srv_name>`：类似于 `rossrv show`，提供服务消息类型的信息。
- `rossrv md5 <srv_name>`：生成服务消息的 MD5 校验码，用于数据完整性检查。

## 2.4.6 rosparam
`rosparam` 包含用于在参数服务器上获取和设置 ROS 参数的命令行工具，使用 YAML 编码的文件。

- `rosparam set`：设置参数
- `rosparam get`：获取参数
- `rosparam load`：从外部文件加载参数
- `rosparam dump`：将参数转储到外部文件
- `rosparam delete`：删除参数
- `rosparam list`：列出所有参数

示例：

- `rosparam list`：列出参数服务器上的所有参数。
- `rosparam set <param_name> <value>`：为参数设置特定的值。
- `rosparam get <param_name>`：获取特定参数的值。
- `rosparam delete <param_name>`：删除特定参数。
- `rosparam load <file_name.yaml>`：从 YAML 文件加载参数。
- `rosparam dump <file_name.yaml>`：将当前参数转储到 YAML 文件。

通过这些命令，你可以动态地与 ROS 生态系统的不同组件进行交互，从而为管理和监控基于 ROS 的机器人系统提供强大的支持。