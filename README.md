<div align="center">
  <img src="https://files.seeedstudio.com/wiki/wiki-platform/S-tempor.png" width="100%" alt="Seeed Studio Robotics">
  
  # 🤖 Seeed Studio Robotics
  
  **Hardware Documentation & Robotics Academy**
  
  > *"The science of today is the technology of tomorrow." - Edward Teller*

  [English](#-english) | [中文](#-中文)
</div>

---

## 🌏 English

Welcome to the Seeed Studio Robotics repository. Here you will find comprehensive documentation for our robotic hardware kits, actuators, sensors, and our newly launched Robotics Academy learning courses.

### 📚 Table of Contents
- [📦 Hardware Wiki](#-hardware-wiki)
  - [Robot Kits](#robot-kits)
  - [Actuators](#actuators)
  - [Sensors](#sensors)
  - [Software Ecosystem](#software-ecosystem)
- [🎓 Robotics Academy](#-robotics-academy)
  - [Core Software & Simulation](#core-software--simulation)
  - [Robot Applications](#robot-applications)

---

### 📦 Hardware Wiki

Detailed guides, datasheets, and setup instructions for robotic hardware.

#### <a id="robot-kits"></a>🤖 Robot Kits
| Category | Product / Guide | Status |
| :--- | :--- | :--- |
| **🤗 SO-Arm** | [SO100/101 Arm](/lerobot_so100m_new/) | |
| | [SO10x Arm with LeIsaac](/simulate_soarm101_by_leisaac/) | |
| | [Phospho Lerobot](/control_robotic_arm_via_phospho/) | |
| | [So Arm RL Training](/training_soarm101_policy_with_isaacLab/) | 🔥 New |
| | [SO101 with NVIDIA GR00T](/fine_tune_gr00t_n1.5_for_lerobot_so_arm_and_deploy_on_jetson_thor/) | 🔥 New |
| **🚗 Lekiwi** | [Lekiwi Mobile Base](/lerobot_lekiwi/) | |
| | [Lekiwi Sound Follow](/sound_follow_robot/) | |
| **🦾 StarAI Arm** | [StarAI Robotic Arm](/lerobot_starai_arm/) | 🔥 New |
| | [StarAI Arm MoveIt 2](/starai_arm_ros_moveit/) | 🔥 New |
| | [StarAI with NVIDIA GR00T](/control_robotic_arm_via_gr00t) | |
| **🦿 Legged** | [Mini Wheeled-Legged Robot](/StackForce_Mini_Wheeled_Legged_Robot) | 🔥 New |
| **🖐️ Hand** | [AmazingHand](/hand_amazinghand/) | 🔥 New |
| **🦀 Effector** | [DM Gripper](/dm_gripper/) | 🔥 New |

#### <a id="actuators"></a>⚙️ Actuators
*   **Series:** [MyActuator X Series](/myactuator_series/) | [Damiao DM43 Series](/damiao_series/) | [HighTorque Series](/hightorque_control) | [Fashionstar Series](/fashionstar_servo/) | [Stackforce Series](/stackforce_series/)
*   **Specific:** [Feetech STS3215 Servo](/feetech_servo/)
*   **Control:** [RobStride Control](/robstride_control/) (🔥 New)

#### <a id="sensors"></a>👁️ Sensors
*   **📡 LiDAR:** [RoboSense](/robosense_lidar/) | [MID360](/mid360/) | [Slamtec](/slamtec/) | [A-LOAM Algorithm](/a_loam/)
*   **📷 Camera:** 
    *   [Orbbec Gemini 2](/orbbec_gemini2/) | [Gemini 335Lg Depth](/orbbec_gemini_335lg) (🔥 New) | [Gemini 336 Depth](/orbbec_gemini336) (🔥 New)
    *   [RoboSense AC1](/ac1) (🔥 New) | [SENSING GMSL2](/sensing_gmsl_cameras)
    *   **Integrations:** [Orbbec with ROS](/orbbec_depth_camera_on_ros/) | [ORB-SLAM3](/orb_slam3_orbbec_gemini2/) | [PyCuVSLAM](/pycuvslam_recomputer_robotics/)
*   **🎤 Voice:** [ReSpeaker Core v2.0](/ReSpeaker_Core_v2.0/) | [ReSpeaker Mic Array v2.0](/ReSpeaker_Mic_Array_v2.0/)
*   **🧭 IMU:** [HEXFELLOW Y200](/hexfellow_y200/) | [WHEELTEC IMU](/wheeltec_imu/)

#### <a id="software-ecosystem"></a>💻 Software Ecosystem
*   **ROS:** [ROS 1 Install](/installing_ros1/) | [ROS 2 Install](/install_ros2_humble/)
*   **NVIDIA Isaac:** [Isaac ROS Install](/install_isaacros/) | [Isaac Lab Install](/install_isaaclab/) | [Isaac Sim (SO Arm)](/simulate_soarm101_by_leisaac/)
*   **Algorithms:** [Isaac ROS AprilTag](/isaac_ros_apriltag/) | [Isaac ROS V-SLAM](/isaac_ros_visual_slam/)
*   **PX4:** [PX4 with Jetson](/control_px4_with_recomputer_jetson/) | [Object Tracking](/object_tracking_with_reComputer_jetson_and_pX4/)

---

### 🎓 Robotics Academy

Comprehensive curriculum covering ROS, Simulation, Reinforcement Learning, and advanced robot control.

#### Core Software & Simulation
| Course | Level | Time | Description |
| :--- | :--- | :--- | :--- |
| **ROS 1 Basics** | 🟢 Beginner | 8h | Learn Nodes, Topics, Services, and DIY Mobile Robot. |
| **ROS 2 Humble** | 🟡 Intermediate | 8h | Master Nodes, Topics, Services, and Actions with Python. |
| **MoveIt 1/2** | 🟢 Beginner | 6h | Custom Manipulator import & Kinematics implementation. |
| **Pinocchio** | 🟡 Intermediate | 6h | Kinematics and Dynamics tasks for your own manipulator. |
| **NVIDIA Isaac Sim**| 🔴 Advanced | 12h | Sim2Real workflows, USD pipelines, and environments. |
| **MuJoCo** | 🔴 Advanced | 10h | Model dynamics and simulation for agile robot control. |
| **Reinforcement Learning** | 🟣 Hard | 20h | PPO, DRL training pipelines, and Jetson deployment. |

#### Robot Applications
*   **Humanoid:** Small Servo Humanoid Robots Course (Hardware, Motion & Dance).
*   **Mobile Robot:** Mobile Robotics (Lekiwi) - SLAM, Nav2, autonomous patrolling.
*   **Robot Arm:** StarAI Arm System - Kinematics, Dynamics, Imitation Learning & VLA-based RL.
*   **Wheel-Legged:** Stackforce Course - Balance control and LQR motion planning.
*   **Desktop Robot:** ReachyMini Course - Basic Operation & Secondary Development.

<br>

---
