# ROS 通信机制

## 话题通信教程 (Topic Communication)

### 话题通信简介

在 ROS 中，话题通信是节点之间交换信息的最基本方式之一。本教程将指导您使用 C++ 和 Python 设置基础的话题通信。我们将实现一个简单的“发布者-订阅者”（Publisher-Subscriber）模型，其中发布者以 10Hz 的频率发送文本消息，订阅者接收并打印这些消息。

  <p align="center">
    <a>
    <img src="./images/Topic.png" alt="J3010">
  </a>
</p>

#### 1. 理论模型

话题通信涉及三个主要组件：
- **ROS Master**：管理节点的注册和连接。
- **Talker**（发布者）：发送消息。
- **Listener**（订阅者）：接收消息。

ROS Master 协助在发布者和订阅者之间建立连接。以下是通信发生的具体步骤：

- **发布者注册**：发布者向 ROS Master 注册自身信息，包括其发送消息的话题名称。
- **订阅者注册**：订阅者向 ROS Master 注册自身信息，指定其想要订阅的话题名称。
- **匹配**：ROS Master 根据话题名称匹配发布者和订阅者，并发送必要的连接信息。
- **建立连接**：订阅者请求与发布者连接，发布者进行确认。
- **消息交换**：连接建立后，发布者开始向订阅者发送消息。

**关键点**：
- ROS Master 仅在建立连接时需要。
- 连接建立后，即使关闭 ROS Master，通信仍可继续。
- 可以存在多个发布者和订阅者，且它们的启动顺序不限。

#### 2. 基础话题通信操作 (C++)

**目标**：创建一个以 10Hz 发送文本消息的发布者节点，以及一个打印接收到的消息的订阅者节点。

**步骤**：

0. **[创建功能包](../6.1.2-Quick%20Experience%20with%20HelloWorld%20for%20ROS/README.md)**
    ```bash
    cd ~/seeed_ws/src/
    catkin_create_pkg listener_and_talker roscpp rospy std_msgs
    cd ~/seeed_ws/src/listener_and_talker/src
    touch listener.cpp talker.cpp
    ```

1. **发布者实现**：
  
    `talker.cpp`
   ```cpp
   #include "ros/ros.h"
   #include "std_msgs/String.h"
   #include <sstream>

   int main(int argc, char *argv[]) {
       // 设置区域设置以支持在本地语言环境下打印消息
       setlocale(LC_ALL, "");
       // 使用唯一名称初始化 ROS 节点
       ros::init(argc, argv, "talker");
       // 创建 ROS 节点句柄
       ros::NodeHandle nh;
       // 创建发布者对象
       ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);

       std_msgs::String msg;
       std::string msg_front = "Hello Seeed";
       int count = 0;
       ros::Rate r(10); // 10Hz

       while (ros::ok()) {
           std::stringstream ss;
           ss << msg_front << count;
           msg.data = ss.str();
           pub.publish(msg);
           ROS_INFO("发送的消息: %s", msg.data.c_str());
           r.sleep();
           count++;
       }
       return 0;
   }
   ```

2. **订阅者实现**：

    `listener.cpp`
   ```cpp
   #include "ros/ros.h"
   #include "std_msgs/String.h"

   void doMsg(const std_msgs::String::ConstPtr& msg_p) {
       ROS_INFO("听到: %s", msg_p->data.c_str());
   }

   int main(int argc, char *argv[]) {
       setlocale(LC_ALL, "");
       ros::init(argc, argv, "listener");
       ros::NodeHandle nh;
       ros::Subscriber sub = nh.subscribe<std_msgs::String>("chatter", 10, doMsg);
       ros::spin();
       return 0;
   }
   ```

3. **CMakeLists.txt 配置**：
  
    在功能包的 `CMakeLists.txt` 末尾添加以下代码：
    ```cmake
    add_executable(listener src/listener.cpp)
    add_executable(talker src/talker.cpp)

    target_link_libraries(listener ${catkin_LIBRARIES})
    target_link_libraries(talker ${catkin_LIBRARIES})
    ```

    <p align="center">
      <a>
      <img src="./images/CMakeLists.png" alt="J3010"  width="600" height="auto">
      </a>
    </p>

4. **运行代码**：
  - 打开终端并启动 `roscore`：
    ```bash
    roscore
    ```
  - 在新终端中，导航到工作空间并运行发布者节点：
    ```bash
    rosrun listener_and_talker listener
    ```
  - 在另一个终端中，运行订阅者节点：
    ```bash
    rosrun listener_and_talker talker
    ```
    <p align="center">
      <a>
      <img src="./images/run_listener_and_talker.png" alt="J3010"  width="600" height="auto">
      </a>
    </p>

      <p align="center">
      <a>
      <img src="./images/run_listener_and_talker_result.png" alt="J3010"  width="600" height="auto">
      </a>
    </p>
    
您应该会在终端中看到正在发布和接收的消息。

#### 3. 基础话题通信操作 (Python)

**目标**：创建一个以 10Hz 发送文本消息的发布者节点，以及一个打印接收到的消息的订阅者节点。

**步骤**：

0. **[创建功能包](../6.1.2-Quick%20Experience%20with%20HelloWorld%20for%20ROS/README.md)**
    ```bash
    cd ~/seeed_ws/src/
    catkin_create_pkg listener_and_talker roscpp rospy std_msgs
    mkdir ~/seeed_ws/src/listener_and_talker/scripts
    cd ~/seeed_ws/src/listener_and_talker/scripts
    touch listener.py talker.py
    ```

1. **发布者实现**：

    `talker.py`
   ```python
   #!/usr/bin/env python
   import rospy
   from std_msgs.msg import String

   if __name__ == "__main__":
       rospy.init_node("talker_p")
       pub = rospy.Publisher("chatter", String, queue_size=10)
       msg = String()
       msg_front = "hello 你好"
       count = 0
       rate = rospy.Rate(10)  # 10Hz

       while not rospy.is_shutdown():
           msg.data = msg_front + str(count)
           pub.publish(msg)
           rospy.loginfo("发送的消息: %s", msg.data)
           rate.sleep()
           count += 1
   ```

2. **订阅者实现**：

    `listener.py`
   ```python
   #!/usr/bin/env python
   import rospy
   from std_msgs.msg import String

   def doMsg(msg):
       rospy.loginfo("听到: %s", msg.data)

   if __name__ == "__main__":
       rospy.init_node("listener_p")
       sub = rospy.Subscriber("chatter", String, doMsg, queue_size=10)
       rospy.spin()
   ```

3. **添加可执行权限**：
   ```bash
    sudo chmod +x *.py
   ```

4. **CMakeLists.txt 配置**：

    在功能包的 `CMakeLists.txt` 末尾添加以下代码：
   ```cmake
   catkin_install_python(PROGRAMS
     scripts/talker.py
     scripts/listener.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   )
   ```

5. **运行代码**：
   - 打开终端并启动 `roscore`：
     ```bash
     roscore
     ```
   - 在新终端中，导航到工作空间并运行发布者节点：
     ```bash
     rosrun listener_and_talker talker.py
     ```
   - 在另一个终端中，运行订阅者节点：
     ```bash
     rosrun listener_and_talker listener.py
     ```

您应该会在终端中看到正在发布和接收的消息。

### ROS 话题常用命令

- `rostopic bw`：显示话题使用的带宽
- `rostopic delay`：显示带有 header 的话题延迟
- `rostopic echo`：将消息打印到屏幕
- `rostopic find`：按类型查找话题
- `rostopic hz`：显示话题的发布频率
- `rostopic info`：显示有关话题的信息
- `rostopic list`：列出所有活动话题
- `rostopic pub`：向话题发布数据
- `rostopic type`：打印话题的类型

----
## 服务通信简介 (Service Communication)

ROS 中的服务通信与话题通信的不同之处在于它是双向的。它不仅允许发送消息，还允许接收反馈。该模型由两个主要部分组成：
1. **客户端 (Client)**：发送请求的实体。
2. **服务端 (Server)**：处理请求并发送回响应的实体。

当客户端向服务端发送请求时，它会等待服务端处理请求并返回响应。这种机制遵循“请求-响应”结构，从而完成通信。

#### 工作原理
- **节点 B**（服务端）提供一个服务接口，通常命名为 `/service_name` 类似的形式。
- **节点 A**（客户端）向节点 B 发送请求。
- 节点 B 处理请求并发送回响应。
  
通信过程可以图示如下：

1. **发布者节点通过 ROS Master 广告服务：**
   - 发布者节点通过 ROS Master 广告一个服务（例如 `advertiseService("bar", foo:1234)`），表明其可用性。

2. **订阅者节点通过 ROS Master 查找服务：**
   - 订阅者节点向 ROS Master 发送请求以查找服务（例如 `lookupService("bar")`）。

3. **ROS Master 返回服务地址：**
   - ROS Master 返回服务地址（例如 `foo:3456`），供订阅者节点连接。

4. **订阅者节点向发布者节点请求数据：**
   - 订阅者节点向发布者节点发送服务请求，使用 XML/RPC 进行通信。

5. **发布者节点回复请求的数据：**
   - 发布者节点处理请求并通过 TCP 发送回回复数据。

    <p align="center">
      <a>
      <img src="./images/Service.png" alt="J3010"  width="600" height="auto">
    </a>

**关键点**：
- 客户端在收到服务端的响应之前会被阻塞。
- 服务通信是高效的，因为它仅在需要时（即发出请求时）才消耗资源。

#### 理论模型

服务通信模型涉及三个关键组件：

1. **ROS Master**：管理服务端和客户端的注册，帮助根据匹配的服务名称建立连接。
2. **服务端 (Server)**：提供服务。
3. **客户端 (Client)**：请求服务。

**流程概述**：

1. **服务端注册**： 
   - 服务端向 ROS Master 注册自身，包括其提供的服务名称。
   
2. **客户端注册**：
   - 客户端向 ROS Master 注册自身，指定其想要使用的服务。

3. **匹配与连接**：
   - ROS Master 根据服务名称匹配客户端和服务端，并促成连接。

4. **请求-响应循环**：
   - 客户端向服务端发送请求，服务端处理请求并返回响应。

#### 话题 vs. 服务通信

让我们对比这两种最常用的 ROS 通信方法，以加深理解：

| **方面**           | **话题通信**             | **服务通信**                 |
|--------------------|-------------------------|---------------------------|
| 通信类型           | 异步                     | 同步                       |
| 协议               | TCP/IP                  | TCP/IP                    |
| 通信模型           | 发布-订阅                | 请求-响应                   |
| 关系               | 多对多                   | 一对多                     |
| 特性               | 基于回调                 | 远程过程调用 (RPC)          |
| 使用场景           | 连续、高频数据           | 低频、特定任务              |
| 示例               | 发布激光雷达数据          | 触发传感器或拍照            |

**注意**：远程过程调用 (RPC) 指的是在不同的进程上执行函数，就像在本地执行一样。

#### 5. 在 ROS 中创建自定义服务 (srv)

让我们通过一个动手示例来深入了解：创建一个自定义服务，用于对客户端发送的两个整数求和。服务端将处理此请求并将总和返回给客户端。

**实现步骤**：
1. **创建一个新功能包**
    ```bash
    cd ~/seeed_ws/src
    catkin_create_pkg service_communication roscpp rospy std_msgs
    cd ~/seeed_ws
    catkin_make
    ```
1. **定义 srv 文件**：
   `srv` 文件定义了请求和响应的结构。在本例中，请求将包含两个整数，响应将包含它们的总和。
   在功能包中创建一个名为 `srv` 的新目录，并添加一个名为 `AddInts.srv` 的文件：
   ```bash
    mkdir ~/seeed_ws/src/service_communication/srv
    cd ~/seeed_ws/src/service_communication/srv
    touch AddInts.srv
   ```
   将以下内容复制到 `AddInts.srv` 中：
     ```srv
     int32 num1
     int32 num2
     ---
     int32 sum
     ```
      <p align="center">
        <a>
        <img src="./images/srv_code.png" alt="J3010">
        </a>
      </p>

2. **更新 package.xml**：
   在功能包的 `package.xml` 中添加生成消息文件所需的依赖项：
    ```xml
    <build_depend>message_generation</build_depend>
    <exec_depend>message_runtime</exec_depend>
    ```
    <p align="center">
      <a>
      <img src="./images/package_xml.png" alt="J3010"  width="600" height="auto">
    </a>

3. **更新 CMakeLists.txt**：
   - 在功能包的 `CMakeLists.txt` 中包含生成服务文件所需的配置：
     ```cmake
     find_package(catkin REQUIRED COMPONENTS
       roscpp
       rospy
       std_msgs
       message_generation
     )

     add_service_files(
       FILES
       AddInts.srv
     )

     generate_messages(
       DEPENDENCIES
       std_msgs
     )
     ```
      <p align="center">
        <a>
        <img src="./images/srv_cmakelists.png" alt="J3010">
        </a>
      </p>


4. **编译功能包**：
   - 编译功能包以生成服务消息头文件：
     ```bash
     cd ~/seeed_ws
     catkin_make
     source devel/setup.bash
     ```

### 实现服务通信 (C++)

此示例演示如何使用 C++ 在 ROS 中实现服务通信。我们将创建一个简单的服务，服务端将客户端提供的两个整数相加并返回总和。

**1. 服务端实现：**

`add_two_ints_server.cpp`
```cpp
#include "ros/ros.h"
#include "service_communication/AddInts.h"

// 处理客户端请求的回调函数
bool add(service_communication::AddInts::Request &req,
         service_communication::AddInts::Response &res) {
    res.sum = req.num1 + req.num2;  // 计算总和
    ROS_INFO("请求内容: a=%ld, b=%ld", (long int)req.num1, (long int)req.num2);
    ROS_INFO("发送回响应: [%ld]", (long int)res.sum);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle nh;

    // 向 ROS Master 广告服务
    ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
    ROS_INFO("准备进行两个整数求和。");
    ros::spin();

    return 0;
}
```

**2. 客户端实现：**

`add_two_ints_client.cpp`
```cpp
#include "ros/ros.h"
#include "service_communication/AddInts.h"
#include <cstdlib>

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_two_ints_client");
    if (argc != 3) {
        ROS_INFO("用法: add_two_ints_client X Y");
        return 1;
    }

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<service_communication::AddInts>("add_two_ints");

    // 准备服务请求
    service_communication::AddInts srv;
    srv.request.num1 = atoll(argv[1]);
    srv.request.num2 = atoll(argv[2]);

    // 调用服务并检查是否成功
    if (client.call(srv)) {
        ROS_INFO("总和: %ld", (long int)srv.response.sum);
    } else {
        ROS_ERROR("调用服务 add_two_ints 失败");
        return 1;
    }

    return 0;
}
```

**CMakeLists.txt 配置：**

确保在 `CMakeLists.txt` 末尾添加以下行，以编译服务端和客户端：

```cmake
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
add_executable(add_two_ints_client src/add_two_ints_client.cpp)

add_dependencies(add_two_ints_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(add_two_ints_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
```

#### 编译并运行演示
打开一个终端：
```bash
cd ~/seeed_ws
catkin_make
roscore
```
打开另一个终端：
```bash
cd ~/seeed_ws
source devel/setup.bash
 rosrun service_communication add_two_ints_server
```

打开第三个终端：
```bash
cd ~/seeed_ws
source devel/setup.bash
rosrun service_communication add_two_ints_client 1 5
```

<p align="center">
  <a>
  <img src="./images/run_service_c.png" alt="J3010"   width="600" height="auto">
  </a>
</p>

#### 实现服务通信 (Python)

此 Python 示例实现了与 C++ 示例相同的功能，即使用服务来对两个整数求和。

在功能包下创建一个 `scripts` 文件夹，并在其中创建 `add_two_ints_server.py` 和 `add_two_ints_client.py` 文件。
```bash
cd ~/seeed/src/service_communication/
mkdir scripts
cd scripts
touch add_two_ints_server.py add_two_ints_client.py
```

**1. 服务端实现：**

`add_two_ints_server.py`
```python
#!/usr/bin/env python
import rospy
from service_communication.srv import AddInts,AddIntsRequest, AddIntsResponse
def doReq(req):
    sum = req.num1 + req.num2
    rospy.loginfo("数据: num1 = %d, num2 = %d, 总和 = %d",req.num1, req.num2, sum)
    resp = AddIntsResponse(sum)
    return resp
if __name__ == "__main__":
    rospy.init_node("addints_server_p")
    server = rospy.Service("AddInts",AddInts,doReq)
    rospy.spin()
```

**2. 客户端实现：**

`add_two_ints_client.py`
```python
#!/usr/bin/env python

import sys
import rospy
from service_communication.srv import *

if __name__ == "__main__":
    if len(sys.argv) != 3:
        rospy.logerr("错误")
        sys.exit(1)
    rospy.init_node("AddInts_Client_p")
    client = rospy.ServiceProxy("AddInts",AddInts)
    client.wait_for_service()
    req = AddIntsRequest()
    req.num1 = int(sys.argv[1])
    req.num2 = int(sys.argv[2]) 
    resp = client.call(req)
    rospy.loginfo("结果: %d",resp.sum)
```

**CMakeLists.txt 配置：**

在您的 `CMakeLists.txt` 中为 Python 脚本添加以下行：

```cmake
catkin_install_python(PROGRAMS
  scripts/add_two_ints_server.py 
  scripts/add_two_ints_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

#### 编译并运行演示
打开一个终端：
```bash
cd ~/seeed_ws
catkin_make
roscore
```
打开另一个终端：
```bash
cd ~/seeed_ws
source devel/setup.bash
rosrun service_communication add_two_ints_server.py
```

打开第三个终端：
```bash
cd ~/seeed_ws
source devel/setup.bash
rosrun service_communication add_two_ints_client.py 1 5
```
 
 
#### 服务通信命令

要操作 ROS 中的服务，您将使用 `rosservice` 命令。以下是常用 `rosservice` 命令及其功能的列表：

- `rosservice args`：打印服务所需的参数
- `rosservice call`：使用提供的参数调用服务
- `rosservice find`：按类型查找服务
- `rosservice info`：打印有关服务的信息
- `rosservice list`：列出所有活动服务
- `rosservice type`：打印服务的类型
- `rosservice uri`：打印服务的 ROSRPC URI


---
## ROS 参数服务器简介 (Parameter Server)

ROS 参数服务器是一个共享的、多用户的、可通过网络访问的参数存储空间。它提供了一种在运行时存储和检索参数的方法，可用于配置节点或在节点之间共享数据。服务器上的参数可以是各种数据类型，包括整数、布尔值、字符串、浮点数、列表和字典。参数服务器由 **ROS Master** 管理，节点通过设置、检索或删除参数与其进行交互。

### 参数服务器理论模型

参数服务器涉及三个主要角色：
1. **ROS Master**：管理参数服务器，充当参数的中央存储库。
2. **Talker**（设置者）：在服务器上设置参数的节点。
3. **Listener**（读取者）：从服务器检索参数的节点。

与参数服务器交互的过程通常涉及以下步骤：

1. **设置参数 (Talker)**：
   - Talker 节点通过 RPC（远程过程调用）向参数服务器发送一个参数，包括参数的名称和值。ROS Master 将此参数存储在其列表中。

2. **检索参数 (Listener)**：
   - Listener 节点通过发送带有参数名称的查询向参数服务器请求参数。

3. **返回参数 (ROS Master)**：
   - ROS Master 在其存储中搜索请求的参数，并将相应的值返回给 Listener。

    <p align="center">
      <a>
      <img src="./images/Parameter_Server.png" alt="J3010"  width="600" height="auto">
    </a>

**支持的数据类型**：
- 32 位整数
- 布尔值
- 字符串
- 浮点数 (Double)
- ISO8601 日期
- 列表
- Base64 编码的二进制数据
- 字典

### C++ 实现

**设置参数 (C++):**

我们首先使用两种不同的 API 设置参数服务器上的各种类型的参数：`ros::NodeHandle` 和 `ros::param`。

`set_parameters.cpp`
```cpp
#include "ros/ros.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "set_parameters");

    std::vector<std::string> students = {"Alice", "Bob", "Charlie", "David"};
    std::map<std::string, std::string> friends = {{"John", "Doe"}, {"Jane", "Smith"}};

    // 使用 ros::NodeHandle 设置参数
    ros::NodeHandle nh;
    nh.setParam("int_param", 42);
    nh.setParam("double_param", 3.14159);
    nh.setParam("bool_param", true);
    nh.setParam("string_param", "Hello ROS");
    nh.setParam("vector_param", students);
    nh.setParam("map_param", friends);

    // 使用 ros::param 设置参数
    ros::param::set("int_param_param", 84);
    ros::param::set("double_param_param", 6.28318);
    ros::param::set("bool_param_param", false);
    ros::param::set("string_param_param", "Goodbye ROS");
    ros::param::set("vector_param_param", students);
    ros::param::set("map_param_param", friends);

    return 0;
}
```
  在功能包的 `CMakeLists.txt` 末尾添加以下代码：

  ```cmake
  add_executable(set_parameters src/set_parameters.cpp)
  target_link_libraries(set_parameters ${catkin_LIBRARIES})
  ```

在此示例中：
- 我们在参数服务器上设置了各种类型的参数，包括整数、浮点数、布尔值、字符串、向量和映射 (map)。
- 我们同时使用了 `ros::NodeHandle` 和 `ros::param` API 来设置参数。

**检索参数 (C++):**

接下来，我们将检索之前在参数服务器上设置的参数。

`get_parameters.cpp`
```cpp
#include "ros/ros.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "get_parameters");

    // 使用 ros::NodeHandle 检索参数
    ros::NodeHandle nh;
    int int_value;
    double double_value;
    bool bool_value;
    std::string string_value;
    std::vector<std::string> students;
    std::map<std::string, std::string> friends;

    nh.getParam("int_param", int_value);
    nh.getParam("double_param", double_value);
    nh.getParam("bool_param", bool_value);
    nh.getParam("string_param", string_value);
    nh.getParam("vector_param", students);
    nh.getParam("map_param", friends);

    ROS_INFO("检索到的值:");
    ROS_INFO("int_param: %d", int_value);
    ROS_INFO("double_param: %.5f", double_value);
    ROS_INFO("bool_param: %d", bool_value);
    ROS_INFO("string_param: %s", string_value.c_str());

    for (const auto &student : students) {
        ROS_INFO("学生: %s", student.c_str());
    }

    for (const auto &friend_pair : friends) {
        ROS_INFO("朋友: %s = %s", friend_pair.first.c_str(), friend_pair.second.c_str());
    }

    return 0;
}
```

在功能包的 `CMakeLists.txt` 末尾添加以下代码：

```cmake
add_executable(get_parameters src/get_parameters.cpp)
target_link_libraries(get_parameters ${catkin_LIBRARIES})
```

在此示例中：
- 我们使用 `ros::NodeHandle` API 检索服务器上设置的参数。
- 随后将检索到的参数打印到 ROS 日志中进行验证。

**删除参数 (C++):**

最后，让我们看看如何从参数服务器中删除参数。

`delete_parameters.cpp`
```cpp
#include "ros/ros.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "delete_parameters");

    ros::NodeHandle nh;
    bool success;

    // 使用 ros::NodeHandle 删除参数
    success = nh.deleteParam("int_param");
    ROS_INFO("删除 int_param: %s", success ? "成功" : "失败");

    // 使用 ros::param 删除参数
    success = ros::param::del("int_param_param");
    ROS_INFO("删除 int_param_param: %s", success ? "成功" : "失败");

    return 0;
}
```
在功能包的 `CMakeLists.txt` 末尾添加以下代码：

```cmake
add_executable(delete_parameters src/delete_parameters.cpp)
target_link_libraries(delete_parameters ${catkin_LIBRARIES})
```

在此示例中：
- 我们同时使用了 `ros::NodeHandle` 和 `ros::param` API 从服务器中删除参数。
- 删除是否成功的结果会被记录在日志中。

### 2.2 Python 实现

**设置参数 (Python):**

现在让我们使用 Python 设置参数。该过程与 C++ 版本非常相似。

```python
#!/usr/bin/env python

import rospy

if __name__ == "__main__":
    rospy.init_node("set_parameters_py")

    # 设置各种类型的参数
    rospy.set_param("int_param", 42)
    rospy.set_param("double_param", 3.14159)
    rospy.set_param("bool_param", True)
    rospy.set_param("string_param", "Hello ROS")
    rospy.set_param("list_param", ["apple", "banana", "cherry"])
    rospy.set_param("dict_param", {"first_name": "John", "last_name": "Doe"})

    # 修改参数
    rospy.set_param("int_param", 84)
```

在功能包的 `CMakeLists.txt` 末尾添加以下代码：

```cmake
catkin_install_python(PROGRAMS
  scripts/set_parameters_py.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

在此示例中：
- 我们设置了各种类型的参数，包括整数、浮点数、布尔值、字符串、列表和字典。
- 我们还演示了如何修改现有参数。

**检索参数 (Python):**

接下来，我们将检索设置好的参数。

```python
#!/usr/bin/env python

import rospy

if __name__ == "__main__":
    rospy.init_node("get_parameters_py")

    # 检索参数
    int_value = rospy.get_param("int_param", 0)
    double_value = rospy.get_param("double_param", 0.0)
    bool_value = rospy.get_param("bool_param", False)
    string_value = rospy.get_param("string_param", "")
    list_value = rospy.get_param("list_param", [])
    dict_value = rospy.get_param("dict_param", {})

    rospy.loginfo("检索到的值:")
    rospy.loginfo("int_param: %d", int_value)
    rospy.loginfo("double_param: %.5f", double_value)
    rospy.loginfo("bool_param: %s", bool_value)
    rospy.loginfo("string_param: %s", string_value)
    rospy.loginfo("list_param: %s", list_value)
    rospy.loginfo("dict_param: %s", dict_value)
```

在功能包的 `CMakeLists.txt` 末尾添加以下代码：
```cmake
catkin_install_python(PROGRAMS
  scripts/set_parameters_py.py
  scripts/get_parameters_py.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

在此示例中：
- 我们使用 `rospy.get_param` 检索服务器上设置的参数。
- 检索到的值使用 `rospy.loginfo` 记录。

**删除参数 (Python):**

最后，让我们使用 Python 从参数服务器中删除参数。

```python
#!/usr/bin/env python

import rospy

if __name__ == "__main__":
    rospy.init_node("delete_parameters_py")

    try:
        rospy.delete_param("int_param")
        rospy.loginfo("int_param 删除成功。")
    except KeyError:
        rospy.logwarn("int_param 不存在。")

    try:
        rospy.delete_param("non_existent_param")
        rospy.loginfo("non_existent_param 删除成功。")
    except KeyError:
        rospy.logwarn("non_existent_param 不存在。")
```

在功能包的 `CMakeLists.txt` 末尾添加以下代码：
```cmake
catkin_install_python(PROGRAMS
  scripts/set_parameters_py.py
  scripts/get_parameters_py.py
  scripts/delete_parameters_py.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

在此示例中：
- 我们尝试删除一个参数，并使用异常处理 (`KeyError`) 处理参数不存在的情况。

### ROS 参数服务器常用命令
`rosparam` 包含用于在参数服务器上获取和设置 ROS 参数的命令行工具，使用 YAML 编码的文件。

- `rosparam set`：设置参数
- `rosparam get`：获取参数
- `rosparam load`：从外部文件加载参数
- `rosparam dump`：将参数转储到外部文件
- `rosparam delete`：删除参数
- `rosparam list`：列出所有参数

示例：

- `rosparam list`：列出参数服务器上的所有参数。
- `rosparam set <param_name> <value>`：设置具有特定值的参数。
- `rosparam get <param_name>`：获取特定参数的值。
- `rosparam delete <param_name>`：删除特定参数。
- `rosparam load <file_name.yaml>`：从 YAML 文件加载参数。
- `rosparam dump <file_name.yaml>`：将当前参数转储到 YAML 文件。