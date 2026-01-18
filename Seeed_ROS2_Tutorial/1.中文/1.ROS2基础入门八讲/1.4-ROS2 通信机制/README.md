# 2. ROS 2 通信机制

在 ROS 2 中，节点之间的通信依然基于三大支柱：**Topic (话题)**、**Service (服务)** 和 **Parameters (参数)**。但在底层实现上，ROS 2 去除了 Master 节点，转而使用基于 DDS 的中间件进行去中心化的自动发现和通信。

## 2.1 话题通信 (Topic Communication)

### 话题通信简介

话题通信是节点之间通过**发布/订阅**模型交换数据的方式。它适用于连续的数据流（如传感器数据、机器人状态）。

*   **DDS 自动发现**：在 ROS 2 中，发布者和订阅者只要在同一个网络（Domain ID 相同），就能自动发现对方，无需 Master 介入。
*   **通信模型**：
    *   **Publisher (发布者)**: 发送消息。
    *   **Subscriber (订阅者)**: 接收消息。

<p align="center">
  <a>
  <img src="https://docs.ros.org/en/humble/_images/Topic-SinglePublisherandSingleSubscriber.gif" alt="ros2_topic" width="600">
  </a>
</p>

---

### 动手实践：话题通信 (C++)

**目标**：创建 C++ 发布者（发送 "Hello World" + 计数）和订阅者。

#### 1. 创建功能包
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake cpp_topic_pkg --dependencies rclcpp std_msgs
```

#### 2. 编写发布者 (Publisher)
新建 `src/cpp_topic_pkg/src/publisher.cpp`：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// 继承自 rclcpp::Node
class TopicPublisher : public rclcpp::Node
{
public:
  TopicPublisher() : Node("cpp_publisher")
  {
    // 创建发布者：话题名 "topic", 队列长度 10
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    
    // 创建定时器：每 500ms 触发一次回调
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TopicPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello World: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

#### 3. 编写订阅者 (Subscriber)
新建 `src/cpp_topic_pkg/src/subscriber.cpp`：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class TopicSubscriber : public rclcpp::Node
{
public:
  TopicSubscriber() : Node("cpp_subscriber")
  {
    // 创建订阅者
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&TopicSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

#### 4. 配置 CMakeLists.txt
在 `add_executable` 之前添加依赖，在文件末尾添加安装规则：

```cmake
add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME}
)
```

---

### 动手实践：话题通信 (Python)

**目标**：使用 Python 实现相同功能。

#### 1. 创建功能包
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_topic_pkg --dependencies rclpy std_msgs
```

#### 2. 编写发布者
新建 `src/py_topic_pkg/py_topic_pkg/publisher.py`：

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('py_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

#### 3. 编写订阅者
新建 `src/py_topic_pkg/py_topic_pkg/subscriber.py`：

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('py_subscriber')
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
```

#### 4. 配置 setup.py
在 `entry_points` 中注册节点：

```python
    entry_points={
        'console_scripts': [
            'talker = py_topic_pkg.publisher:main',
            'listener = py_topic_pkg.subscriber:main',
        ],
    },
```

---

### 编译与运行
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

*   **终端 1 (C++ 发布):** `ros2 run cpp_topic_pkg talker`
*   **终端 2 (Python 订阅):** `ros2 run py_topic_pkg listener`

### ROS 2 话题常用命令

*   `ros2 topic list`: 列出所有话题。
*   `ros2 topic info /topic`: 查看话题类型和发布/订阅者数量。
*   `ros2 topic echo /topic`: 打印消息内容。
*   `ros2 topic hz /topic`: 查看频率。
*   `ros2 topic pub /topic std_msgs/String "data: 'Hello'"`: 手动发布消息。

---

## 2.2 服务通信 (Service Communication)

服务通信是基于**请求(Request)-响应(Response)**模型的同步或异步通信。
*   **Client (客户端)**: 发起请求。
*   **Server (服务端)**: 处理请求并反馈。

### 1. 创建自定义接口包 (最佳实践)
在 ROS 2 中，建议将自定义消息 (`.msg`) 和服务 (`.srv`) 放在独立的包中，以避免复杂的编译依赖问题。

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake tutorial_interfaces
mkdir -p tutorial_interfaces/srv
```

创建 `tutorial_interfaces/srv/AddTwoInts.srv`：
```text
int64 a
int64 b
---
int64 sum
```

修改 `tutorial_interfaces/CMakeLists.txt`：
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
)
```

修改 `tutorial_interfaces/package.xml`：
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**先编译接口包：**
```bash
cd ~/ros2_ws
colcon build --packages-select tutorial_interfaces
source install/setup.bash
```

---

### 2. C++ 服务端与客户端

**注意：** 在 ROS 2 中，客户端**强烈建议使用异步调用**。如果在回调函数中使用同步阻塞调用，会导致单线程执行器死锁。

创建包：`ros2 pkg create --build-type ament_cmake cpp_service_pkg --dependencies rclcpp tutorial_interfaces`

#### 服务端 (Server)
`src/cpp_service_pkg/src/server.cpp`:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_two_ints.hpp"

void add(const std::shared_ptr<tutorial_interfaces::srv::AddTwoInts::Request> request,
         std::shared_ptr<tutorial_interfaces::srv::AddTwoInts::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld", request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  rclcpp::Service<tutorial_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<tutorial_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");
  rclcpp::spin(node);
  rclcpp::shutdown();
}
```

#### 客户端 (Client - 异步实现)
`src/cpp_service_pkg/src/client.cpp`:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: client X Y");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_client");
  
  // 创建客户端
  rclcpp::Client<tutorial_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<tutorial_interfaces::srv::AddTwoInts>("add_two_ints");

  auto request = std::make_shared<tutorial_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  // 等待服务上线
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // 发送异步请求
  auto result = client->async_send_request(request);
  
  // 等待结果 (spin_until_future_complete)
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
```

*(CMakeLists.txt 配置与话题通信类似，记得添加 `add_executable` 和 `target_link_libraries`)*

---

### 3. Python 服务端与客户端
创建包：`ros2 pkg create --build-type ament_python py_service_pkg --dependencies rclpy tutorial_interfaces`

#### 服务端
`src/py_service_pkg/py_service_pkg/server.py`:
```python
import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main():
    rclpy.init()
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()
```

#### 客户端
`src/py_service_pkg/py_service_pkg/client.py`:
```python
import sys
import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    client = MinimalClientAsync()
    response = client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    client.get_logger().info('Result of add_two_ints: %d' % response.sum)
    client.destroy_node()
    rclpy.shutdown()
```

---

## 2.3 参数 (Parameters)

在 ROS 2 中，**参数归属于特定节点**，不再有全局的 Parameter Server。节点可以通过服务来声明、获取和设置自己的参数。

### 核心操作

1.  **声明参数 (Declare)**: 节点必须先声明参数，才能使用。
2.  **获取/设置 (Get/Set)**: 运行时修改。

### 动手实践 (Python)

```python
import rclpy
from rclpy.node import Node

class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')
        # 1. 声明参数并设置默认值
        self.declare_parameter('my_str', 'Hello')
        self.declare_parameter('my_int', 10)

        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # 2. 获取参数值
        my_str_param = self.get_parameter('my_str').get_parameter_value().string_value
        
        self.get_logger().info('Param value: %s' % my_str_param)

        # 3. 设置新参数 (模拟外部修改)
        # new_param = rclpy.parameter.Parameter('my_str', rclpy.Parameter.Type.STRING, 'World')
        # self.set_parameters([new_param])

def main():
    rclpy.init()
    node = ParamNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### ROS 2 参数命令 (CLI)

*   `ros2 param list`: 列出所有节点的参数。
*   `ros2 param get <node_name> <param_name>`: 获取参数值。
*   `ros2 param set <node_name> <param_name> <value>`: 动态修改参数。
*   `ros2 param dump <node_name>`: 将节点参数导出为 YAML 文件。
*   `ros2 param load <node_name> <file.yaml>`: 加载参数。

**示例：**
```bash
ros2 run py_topic_pkg param_node
# 新开终端修改参数
ros2 param set /param_node my_str "Seeed Studio"
```
你会发现原来的节点日志输出变成了 "Seeed Studio"。