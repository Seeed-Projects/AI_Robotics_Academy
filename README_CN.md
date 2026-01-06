## 🌏 中文

欢迎来到 Seeed Studio 机器人技术仓库。这里汇集了我们的机器人硬件套件、执行器、传感器的详细文档，以及全新推出的机器人学院学习课程。

### 📚 目录
- [📦 硬件 Wiki](#-硬件-wiki)
  - [机器人套件](#机器人套件)
  - [执行器与电机](#执行器与电机)
  - [传感器](#传感器)
  - [软件生态](#软件生态)
- [🎓 机器人学院](#-机器人学院)
  - [核心软件与仿真](#核心软件与仿真)
  - [机器人应用](#机器人应用)

---

### 📦 硬件 Wiki

提供机器人硬件的详细指南、数据手册和设置教程。

#### <a id="机器人套件"></a>🤖 机器人套件 (Robot Kits)
| 分类 | 产品 / 指南 | 状态 |
| :--- | :--- | :--- |
| **🤗 SO-Arm** | [SO100/101 机械臂](/lerobot_so100m_new/) | |
| | [SO10x 机械臂 (LeIsaac)](/simulate_soarm101_by_leisaac/) | |
| | [Phospho Lerobot 控制](/control_robotic_arm_via_phospho/) | |
| | [So Arm 强化学习训练](/training_soarm101_policy_with_isaacLab/) | 🔥 新品 |
| | [SO101 适配 NVIDIA GR00T](/fine_tune_gr00t_n1.5_for_lerobot_so_arm_and_deploy_on_jetson_thor/) | 🔥 新品 |
| **🚗 Lekiwi** | [Lekiwi 移动底盘](/lerobot_lekiwi/) | |
| | [Lekiwi 声源跟随](/sound_follow_robot/) | |
| **🦾 StarAI Arm** | [StarAI 机械臂](/lerobot_starai_arm/) | 🔥 新品 |
| | [StarAI Arm MoveIt 2 配置](/starai_arm_ros_moveit/) | 🔥 新品 |
| | [StarAI 适配 NVIDIA GR00T](/control_robotic_arm_via_gr00t) | |
| **🦿 轮足机器人** | [迷你轮足机器人](/StackForce_Mini_Wheeled_Legged_Robot) | 🔥 新品 |
| **🖐️ 灵巧手** | [AmazingHand](/hand_amazinghand/) | 🔥 新品 |
| **🦀 末端执行器** | [DM 夹爪](/dm_gripper/) | 🔥 新品 |

#### <a id="执行器与电机"></a>⚙️ 执行器与电机 (Actuators)
*   **系列:** [MyActuator X 系列](/myactuator_series/) | [大妙 DM43 系列](/damiao_series/) | [HighTorque 系列](/hightorque_control) | [Fashionstar 系列](/fashionstar_servo/) | [Stackforce 系列](/stackforce_series/)
*   **特定型号:** [飞特 STS3215 舵机](/feetech_servo/)
*   **控制算法:** [RobStride 控制](/robstride_control/) (🔥 新品)

#### <a id="传感器"></a>👁️ 传感器 (Sensors)
*   **📡 激光雷达 (LiDAR):** [速腾聚创 (RoboSense)](/robosense_lidar/) | [MID360](/mid360/) | [思岚 (Slamtec)](/slamtec/) | [A-LOAM 算法](/a_loam/)
*   **📷 摄像头 (Camera):** 
    *   [奥比中光 Gemini 2](/orbbec_gemini2/) | [Gemini 335Lg Depth](/orbbec_gemini_335lg) (🔥 新品) | [Gemini 336 Depth](/orbbec_gemini336) (🔥 新品)
    *   [RoboSense AC1](/ac1) (🔥 新品) | [森云 GMSL2](/sensing_gmsl_cameras)
    *   **集成应用:** [Orbbec ROS 适配](/orbbec_depth_camera_on_ros/) | [ORB-SLAM3](/orb_slam3_orbbec_gemini2/) | [PyCuVSLAM](/pycuvslam_recomputer_robotics/)
*   **🎤 语音 (Voice):** [ReSpeaker Core v2.0](/ReSpeaker_Core_v2.0/) | [ReSpeaker Mic Array v2.0](/ReSpeaker_Mic_Array_v2.0/)
*   **🧭 IMU:** [HEXFELLOW Y200](/hexfellow_y200/) | [WHEELTEC IMU](/wheeltec_imu/)

#### <a id="软件生态"></a>💻 软件生态 (Software)
*   **ROS:** [ROS 1 安装](/installing_ros1/) | [ROS 2 Humble 安装](/install_ros2_humble/)
*   **NVIDIA Isaac:** [Isaac ROS 安装](/install_isaacros/) | [Isaac Lab 安装](/install_isaaclab/) | [Isaac Sim 仿真](/simulate_soarm101_by_leisaac/)
*   **算法:** [Isaac ROS AprilTag](/isaac_ros_apriltag/) | [Isaac ROS V-SLAM](/isaac_ros_visual_slam/)
*   **PX4:** [Jetson 适配 PX4](/control_px4_with_recomputer_jetson/) | [目标追踪 (PX4)](/object_tracking_with_reComputer_jetson_and_pX4/)

---

### 🎓 机器人学院 (Robotics Academy)

涵盖 ROS、仿真、强化学习和高级机器人控制的综合课程体系。

#### 核心软件与仿真
| 课程 | 难度 | 时长 | 简介 |
| :--- | :--- | :--- | :--- |
| **ROS 1 基础** | 🟢 初级 | 8h | 学习节点、话题、服务，并 DIY 移动机器人。 |
| **ROS 2 Humble** | 🟡 中级 | 8h | 使用 Python 掌握节点、话题、服务和动作 (Action)。 |
| **MoveIt 1/2** | 🟢 初级 | 6h | 导入自定义机械臂并实现正逆运动学。 |
| **Pinocchio** | 🟡 中级 | 6h | 学习 Pinocchio 框架，开发机械臂动力学任务。 |
| **NVIDIA Isaac Sim**| 🔴 高级 | 12h | Sim2Real 工作流、USD 管道和环境创建。 |
| **MuJoCo** | 🔴 高级 | 10h | 敏捷机器人控制的动力学建模与仿真。 |
| **强化学习 (RL)** | 🟣 困难 | 20h | PPO、DRL 训练管道以及 Jetson 部署。 |

#### 机器人应用
*   **人形机器人:** 小型舵机人形机器人课程（硬件驱动、动作与舞蹈编排）。
*   **移动机器人:** 移动机器人 (Lekiwi) - SLAM、Navigation 2 和自主巡逻。
*   **机械臂:** StarAI 机械臂系统 - 运动学、动力学、模仿学习及基于 VLA 的强化学习。
*   **轮足机器人:** Stackforce 课程 - 混合机器人的平衡控制与 LQR 运动规划。
*   **桌面机器人:** ReachyMini 课程 - 基础操作、接口教程及二次开发。

---

<p align="center">
  Generated from Seeed Studio Documentation
</p>
