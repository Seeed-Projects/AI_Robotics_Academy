# ROS 概述与环境搭建

## 简介

本教程简要概述了 ROS（Robot Operating System，机器人操作系统），并指导您在 [reComputer Nvidia Jetson Orin](https://www.seeedstudio.com/reComputer-J3010-w-o-power-adapter-p-5631.html) 上快速安装和体验 ROS。在本教程结束时，您将拥有一个可以运行的 ROS 环境，并能够运行一个简单的 ROS 示例演示。

### 前提条件

为了学习本教程，您需要以下硬件和软件：
- **硬件：** [reComputer Nvidia Jetson Orin](https://www.seeedstudio.com/reComputer-J3010-w-o-power-adapter-p-5631.html) 或其他搭载 Ubuntu 20.04 的设备、显示器、键盘和鼠标。
- **软件：** Jetpack 5.1.x、Ubuntu 20.04、ROS Noetic、Python 和 C++。
<p align="center">
  <a href="https://wiki.seeedstudio.com/reComputer_Intro/">
    <img src="https://files.seeedstudio.com/wiki/reComputer-J4012/5.png" alt="J3010" width="600" height="auto">
  </a>
</p>

## ROS 开发简介

### [什么是 ROS？](https://vimeo.com/639236696)

ROS（Robot Operating System）是一个用于机器人软件开发的开源框架。它在异构计算集群的主操作系统之上提供了一个结构化的通信层。ROS 被设计得尽可能精简，由两部分组成：

- **ROS 系统 (ROS)：** 处理进程间通信的管道系统。它是允许机器人不同部件相互通信的中间件。
- **ROS 软件包：** 编写机器人应用程序所需的库和工具。
<p align="center">
  <a>
    <img src="https://www.ros.org/imgs/ros-equation.png" alt="ros"  width="800" height="auto">
  </a>
</p>

### [为什么选择 ROS？](https://www.ros.org/blog/why-ros/)

ROS 简化了在各种机器人平台上创建复杂且稳健的机器人行为的过程。使用 ROS 的一些关键目标和优势包括：

- **快速开发：** 提供标准平台，加速从研究到生产的开发过程。
- **全球社区：** 拥有庞大且活跃的社区支持，不断贡献和改进软件。
- **成熟的记录：** 广泛应用于学术界和商业机器人领域。
- **上市时间：** 凭借全面的工具和库，帮助缩短产品开发周期。
- **多功能性：** 支持多个领域和平台，包括嵌入式系统。
- **开源：** 免费使用、修改和扩展，促进创新与协作。
- **商业友好：** 采用 Apache 2.0 等宽松的许可协议进行分发。

### ROS 的历史与发展

ROS（机器人操作系统）的历史与机器人技术的整体演进交织在一起：

<p align="center">
  <a>
    <img src="./images/Development-history-of-mobile-robot.png" alt="ros"  width="800" height="auto">
  </a>
</p>

#### 早期机器人技术的发展
- **1959年：** 随着第一台自动机器人的开发，机器人之旅开启。
- **1972年：** 能够与环境互动的机器人出现。
- **1982年：** 机器人开始集成到计算机系统中，用于执行复杂任务。
- **1988年：** 机器人自动化和控制系统取得进展。
- **2002年：** 像 Roomba 这样用于家务的消费级机器人问世。
- **2003年：** 机器人探索扩展至火星（火星车）。
- **2005年：** 在工业自动化中发挥关键作用，例如仓库机器人。

#### ROS 的诞生
- **2007年：** ROS 的开发在斯坦福大学人工智能实验室开始，最初名为“Switchyard”，由 Morgan Quigley、Eric Berger 和 Andrew Ng 共同发起。其目的是解决机器人研究中缺乏共享软件基础设施的问题。
- **2008年：** 在机器人研究实验室 Willow Garage 继续开发。
- **2009年：** ROS 正式成立，并发布了第一个版本。

#### 成长与演进
- **2010年：** ROS 1.0 发布，Willow Garage 在其开发和社区成长中发挥了至关重要作用。
- **2014年：** Pepper 等社交和服务机器人的推出，突显了人机交互的进步。
- **2021年：** ROS 演进为支持各种应用的高级且通用的机器人系统。

### [ROS 版本发布时间线](https://docs.ros.org/en/rolling/Releases.html)

| 版本名称 | 发行版 | 发布日期 | 停止维护日期 (EOL) |
|--------------|--------------|--------------|----------------|
| Boxturtle    | ROS 1        | 2010年3月   | 2011年3月     |
| C Turtle     | ROS 1        | 2010年8月   | 2011年8月    |
| Diamondback  | ROS 1        | 2011年3月   | 2012年11月  |
| Electric Emys| ROS 1        | 2011年8月   | 2013年1月   |
| Fuerte       | ROS 1        | 2012年4月   | 2013年7月      |
| Groovy Galapagos| ROS 1     | 2012年12月| 2014年5月       |
| Hydro Medusa | ROS 1        | 2013年9月| 2015年5月      |
| Indigo Igloo | ROS 1        | 2014年7月    | 2019年4月     |
| Jade Turtle  | ROS 1        | 2015年5月     | 2017年5月       |
| Kinetic Kame | ROS 1        | 2016年5月     | 2021年4月     |
| Lunar Loggerhead | ROS 1    | 2017年5月     | 2019年5月       |
| Melodic Morenia| ROS 1      | 2018年5月     | 2023年5月       |
| Noetic Ninjemys | ROS 1     | 2020年5月     | 2025年5月       |
| Foxy Fitzroy | ROS 2        | 2020年6月    | 2023年6月      |
| Galactic Geochelone | ROS 2 | 2021年5月     | 2022年11月  |
| Humble Hawksbill | ROS 2    | 2022年5月     | 2027年5月       |
| Rolling Ridley | ROS 2      | 持续更新      | 持续更新        |

如今，ROS 由 Open Robotics 维护。这是一个致力于开发核心 ROS 系统（包括针对实时和嵌入式系统进行改进的 ROS 2.0）以及其他工具和库的非营利组织。

## ROS 环境安装与快速体验

### 安装 ROS1
- **步骤 1：** 打开终端并更新系统软件包。
  ```bash
  sudo apt update 
  sudo apt upgrade
  ```
- **步骤 2：** 安装基础工具。
  ```bash
  sudo apt install curl gnupg2 lsb-release
  ```
- **步骤 3：** 添加 ROS 存储库密钥。
  ```bash
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
  ```
- **步骤 4：** 添加 ROS 存储库。
  ```bash
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  ```
- **步骤 5：** 更新软件包列表。
  ```bash
  sudo apt update
  ```
- **步骤 6：** 安装 ros-noetic-desktop-full。
  ```bash
  sudo apt install ros-noetic-desktop-full
  sudo apt-get install python3-rosdep
  ```
- **步骤 7：** 初始化 rosdep。
  ```bash
  sudo rosdep init
  rosdep update
  ```
- **步骤 8：** 设置 ROS 环境变量。
  ```bash
  echo "source /opt/ros/noetic/setup.bash">> ~/.bashrc &&
  source ~/.bashrc
  ```
- **步骤 9：** 安装依赖工具。
  ```bash
  sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
  ```
- **步骤 10：** 测试安装。
  ```bash
  roscore
  ```

<p align="center">
  <a>
    <img src="https://files.seeedstudio.com/wiki/robotics/software/install_ros1/fig2.png" alt="ros"  width="600" height="auto">
  </a>
</p>

### ROS 快速入门

为了快速体验 ROS，让我们创建一个 ROS 工作空间并运行一个简单的演示。

1. **创建 ROS 工作空间**
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make
   ```

2. **加载环境设置文件**
   ```bash
   source devel/setup.bash
   ```

3. **运行演示**
   ```bash
   roscore
   ```
   打开另一个终端并运行：
   ```bash
   rosrun turtlesim turtlesim_node
   ```
   再打开一个终端并运行：
   ```bash
   rosrun turtlesim turtle_teleop_key
   ```
  <div align="center">
      <img width={600} 
      src="./images/turtle.png" />
  </div>

这个快速演示展示了一个可以使用键盘控制的图形化乌龟机器人。

### ROS 常用开发软件安装

#### 安装 VSCode 及 ROS 开发扩展
1. **关于 VSCode 的安装，请参考之前的教程：[3.1-Python 与编程基础](https://github.com/Seeed-Projects/reComputer-Jetson-for-Beginners/blob/main/3-Basic-Tools-and-Getting-Started/3.1-Python-and-Programming-Fundamentals/README.md)**

2. **从 VSCode 扩展市场安装 `Python`、`ROS`、`C++` 和 `CMake Tools` 等工具。**

<p align="center">
  <a">
    <img src="./images/vscode_plugs.png" alt="ros"  width="600" height="auto">
  </a>
</p>

#### 安装 Terminator 多功能终端
1. **安装**
    ```bash
    sudo apt-get update
   sudo apt install terminator
   ```

<p align="center">
  <a">
    <img src="./images/terminator.png" alt="ros"  width="600" height="auto">
  </a>
</p>


2. **显示应用程序 ---> 搜索 "Terminator" ---> 右键点击并选择 "添加到收藏夹"**

3. **Terminator 常用快捷键**
    - **Alt + Up**: 移动到上方的终端
    - **Alt + Down**: 移动到下方的终端
    - **Alt + Left**: 移动到左侧的终端
    - **Alt + Right**: 移动到右侧的终端
    - **Ctrl + Shift + O**: 水平拆分终端
    - **Ctrl + Shift + E**: 垂直拆分终端
    - **Ctrl + Shift + Right**: 在垂直拆分的终端中向右移动分割线
    - **Ctrl + Shift + Left**: 在垂直拆分的终端中向左移动分割线
    - **Ctrl + Shift + Up**: 在水平拆分的终端中向上移动分割线
    - **Ctrl + Shift + Down**: 在水平拆分的终端中向下移动分割线
    - **Ctrl + Shift + S**: 隐藏/显示滚动条
    - **Ctrl + Shift + F**: 查找
    - **Ctrl + Shift + C**: 复制选中内容到剪贴板
    - **Ctrl + Shift + V**: 粘贴剪贴板内容
    - **Ctrl + Shift + W**: 关闭当前终端
    - **Ctrl + Shift + Q**: 退出当前窗口，关闭其中所有终端
    - **Ctrl + Shift + X**: 最大化当前终端
    - **Ctrl + Shift + Z**: 最大化当前终端并放大字体
    - **Ctrl + Shift + N 或 Ctrl + Tab**: 切换到下一个终端
    - **Ctrl + Shift + P 或 Ctrl + Shift + Tab**: 切换到上一个终端
    - **F11**: 切换全屏
    - **Ctrl + Shift + T**: 打开新标签页
    - **Ctrl + PageDown**: 切换到下一个标签页
    - **Ctrl + PageUp**: 切换到上一个标签页
    - **Ctrl + Shift + PageDown**: 将当前标签页与下一个交换位置
    - **Ctrl + Shift + PageUp**: 将当前标签页与上一个交换位置
    - **Ctrl + Plus (+)**: 增大字体
    - **Ctrl + Minus (-)**: 减小字体
    - **Ctrl + Zero (0)**: 重置字体大小为默认
    - **Ctrl + Shift + R**: 重置终端状态
    - **Ctrl + Shift + G**: 重置终端状态并清屏
    - **Super + g**: 广播所有终端（在所有终端同步输入内容）
    - **Super + Shift + G**: 取消广播所有终端
    - **Super + t**: 广播当前标签页的所有终端
    - **Super + Shift + T**: 取消广播当前标签页的终端
    - **Ctrl + Shift + I**: 打开新窗口（与原窗口共享进程）
    - **Super + i**: 打开新窗口（独立进程）