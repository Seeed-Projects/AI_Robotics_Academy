# 1.7 - ROS 2 常用组件与特性

## ROS 2 分布式通信

ROS 2 天生就是为分布式系统设计的。与 ROS 1 不同，ROS 2 取消了中心化的 Master 节点 (`roscore`)，转而使用基于 DDS (Data Distribution Service) 的中间件来实现节点间的自动发现和通信。

这意味着：**只要两台电脑处于同一个局域网内，并且配置了相同的 Domain ID，它们就能自动看到彼此并进行通信。**

### 核心概念：Domain ID

在 ROS 1 中，我们通过指定 `ROS_MASTER_URI` 来决定连接到哪个网络。
在 ROS 2 中，我们使用 **`ROS_DOMAIN_ID`**。

*   **隔离性**：`ROS_DOMAIN_ID` 是一个整数（默认是 0）。只有 ID 相同的节点才能相互通信。
*   **应用场景**：如果实验室里有两组人在同一个 Wi-Fi 下调试不同的机器人，A 组可以设置 `ID=1`，B 组设置 `ID=2`，通过这种方式彻底隔离数据，互不干扰。

### 实现步骤

#### 1. 准备工作

*   **网络连接**：确保所有计算机（主机、树莓派、Jetson 等）都连接到同一个路由器（Wi-Fi 或网线均可）。
*   **网络设置**：ROS 2 极其依赖**UDP 组播 (Multicast)**。
    *   如果是虚拟机，必须设置为 **“桥接模式 (Bridged Adapter)”**。NAT 模式通常无法支持多机通信。
    *   检查防火墙，确保允许 UDP 通信。

#### 2. 配置 Domain ID

在所有需要通信的机器上，设置相同的 `ROS_DOMAIN_ID`。

**在机器 A (例如 PC) 上：**
打开终端或修改 `~/.bashrc`：
```bash
# 建议选择 0-101 之间的整数
export ROS_DOMAIN_ID=5
```

**在机器 B (例如机器人/Jetson) 上：**
同样设置 ID 为 5：
```bash
export ROS_DOMAIN_ID=5
```

> **注意：** 如果不设置该变量，ROS 2 默认使用 ID `0`。但在多机环境中，建议显式设置一个非零 ID 以避免冲突。

#### 3. 检查 Localhost 限制 (常见坑点)

ROS 2 有一个特殊的环境变量 `ROS_LOCALHOST_ONLY`。如果它被设置为 `1`，节点将只在本地通信，无法与外部机器连接。

**在所有机器上检查：**
```bash
echo $ROS_LOCALHOST_ONLY
```
如果输出是 `1`，请将其设为 `0` 或在 `.bashrc` 中取消设置：
```bash
export ROS_LOCALHOST_ONLY=0
```

### 测试设置

ROS 2 提供了一个非常方便的 demo 包 `demo_nodes_cpp` 来测试通信。

#### 1. 测试从机器 A 到机器 B

*   **机器 A (Listener):**
    ```bash
    source ~/.bashrc  # 确保加载了 DOMAIN_ID
    ros2 run demo_nodes_cpp listener
    ```

*   **机器 B (Talker):**
    ```bash
    source ~/.bashrc
    ros2 run demo_nodes_cpp talker
    ```

**预期结果：** 机器 A 的终端应该开始打印 `I heard: [Hello World: ...]`。如果能看到，说明分布式通信已打通。

#### 2. 使用 CLI 工具验证

在任意一台机器上运行：
```bash
ros2 node list
ros2 topic list
```
你应该能看到另一台机器上运行的节点和话题。

### 故障排除 (Troubleshooting)

如果两台机器都在同一个网络且 ID 相同，但仍无法通信，请按以下顺序排查：

1.  **防火墙 (Firewall)**：
    Ubuntu 默认的防火墙可能会拦截 DDS 数据包。
    ```bash
    sudo ufw disable
    ```
    *(或者配置 ufw 允许 UDP 端口 7400-7600)*

2.  **组播支持 (Multicast)**：
    ROS 2 的发现机制依赖组播。有些企业或校园网络会禁用组播功能。
    **测试组播是否通畅：**
    在两台机器上分别运行 `ros2 multicast receive` 和 `ros2 multicast send`。如果收不到消息，说明路由器不支持组播，可能需要使用 *FastDDS Discovery Server*（进阶配置）。

3.  **多网卡问题**：
    如果电脑同时连接了 Wi-Fi 和网线，或者有 Docker 的虚拟网卡，DDS 可能会选错网卡。
    可以通过 XML 配置文件绑定特定网卡（如 `wlan0` 或 `eth0`）。

### ROS 1 vs ROS 2 分布式通信对比

| 特性 | ROS 1 | **ROS 2** |
| :--- | :--- | :--- |
| **核心机制** | Master 节点 (`roscore`) | DDS (无中心节点) |
| **网络标识** | `ROS_MASTER_URI` (IP地址) | `ROS_DOMAIN_ID` (整数) |
| **依赖性** | 必须先启动 Master | 节点随时启动，自动发现 |
| **地址解析** | 必须修改 `/etc/hosts` | 通常不需要，依靠 UDP 发现 |
| **主要痛点** | Master 挂了全网瘫痪 | 路由器不支持组播 |

---

## ROS 2 时间同步 (Time Synchronization)

在分布式系统中，多台机器的时间必须保持一致，否则 TF 坐标变换和传感器数据融合会出错（例如：激光雷达数据的时间戳比当前时间晚了 5 秒，导致数据被丢弃）。

**ROS 2 不会自动同步系统时间**，你需要使用操作系统层面的工具。

### 推荐方案：Chrony

我们在所有机器上安装 `chrony`，选一台作为时间服务器（Master），其他作为客户端（Client）。

1.  **安装 Chrony (所有机器)：**
    ```bash
    sudo apt install chrony
    ```

2.  **配置服务器 (机器人端/主机)：**
    编辑 `/etc/chrony/chrony.conf`，添加：
    ```text
    allow 192.168.1.0/24  # 允许局域网内的机器同步
    local stratum 10      # 即使没联网，也作为本地时间源
    ```
    重启服务：`sudo service chrony restart`

3.  **配置客户端 (其他电脑)：**
    编辑 `/etc/chrony/chrony.conf`，将 `pool ...` 行注释掉，添加：
    ```text
    server <服务器IP地址> minpoll 0 maxpoll 5 iburst
    ```
    重启服务：`sudo service chrony restart`

4.  **验证：**
    在客户端运行 `chronyc sources`，如果看到 `*` 号，说明同步成功。