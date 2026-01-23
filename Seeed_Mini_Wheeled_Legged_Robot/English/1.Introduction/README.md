## Chapter 1. Introduction

### Course Explanation

This tutorial is a **systematic, engineering-oriented course on wheeled-legged robot motion control**, built around a real, fully functional wheeled-legged robot platform. It progressively presents a complete technical roadmap, from **fundamental self-balancing control** to **advanced motion control algorithms**.

The course focuses on the core challenges of wheeled-legged robots, including:

* Pitch self-balancing and closed-loop velocity control

* Attitude control and terrain-adaptive algorithms

* High-mobility motion enabled by parallel bionic leg control

* Advanced motion control methods such as LQR and VMC

* Training wheeled-legged robots using reinforcement learning

* Other applications and algorithm development for wheeled-legged robots

The tutorial combines mathematical derivations with strong engineering insight, emphasizing the workflow of **“physical meaning → control logic → deployable code.”&#x20;**&#x54;his approach helps learners truly understand *why* each control algorithm is designed, *what problem it solves*, and *how it is implemented on a real robot*.

The course assumes that readers have basic experience in embedded systems or robotics, including but not limited to:

* Practical use of IMUs, encoders, motors, and servos

* A basic understanding of control concepts such as PID

Building on this foundation, the course guides learners to develo&#x70;**&#x20;a complete understanding of wheeled-legged robot motion control**, enabling them to independently tune systems, improve algorithms, and even extend their work to **reinforcement learning** or **more advanced intelligent control methods**.

After completing this tutorial, you will be able to:

* Build a stable and controllable wheeled-legged robot self-balancing system from scratch

* Master motion control methods that are proven and usable in real engineering practice

* Establish a solid foundation for advanced research in robot control, reinforcement learning, or product-level development

### Introduction to Educational Robot Kits

#### StackForce Mini Wheeled-Legged Robot&#xA;

![](<images/Wheeled-Leg Robot-image.png>)

The **StackForce Mini Wheeled-Legged Robot** is a bipedal wheeled robot featuring an advanced whole-body kinematic model and adaptive self-balancing algorithms. It supports multi-posture control, stable operation on complex terrain, and stair descent.

It offers multiple control options, including remote control, Bluetooth, serial, and wireless communication, with full installation guides and video tutorials, making it a cost-effective desktop bipedal wheeled robot.

[International site purchasing channels🛒](https://www.seeedstudio.com/Fashionstar-Star-Arm-Viola-Violin-p-6497.html)    <span style="color: inherit; background-color: rgb(247,105,100)">（等待修改）</span>

| Category            | Specification                                                                 |
|---------------------|--------------------------------------------------------------------------------|
| Main Controller     | StackForce Main Control Board                                                   |
| Motor Driver Board  | 5A Dual-Channel Brushless Motor Driver (Low Power)                              |
| Motor               | 2208 Gimbal Brushless Motor                                                      |
| Servo Driver Board  | Multi-Channel Servo Driver with Integrated IMU                                  |
| Power Supply        | 12.6V Lithium Polymer Battery                                                    |
| Encoder             | MT6701 14-bit High-Precision Magnetic Encoder                                   |
| Wireless Control    | WiFi Remote Control + PS4 Bluetooth Game Controller                              |
| Total Weight        | 540 g                                                                           |
| Dimensions          | 10.5 × 21.0 cm (L × W), Height 12.0–21.0 cm                                      |

Other references:

wiki：[Stackforce\_Mini\_Wheeled\_Legged\_Robot](https://wiki.seeedstudio.com/stackforce_mini_wheeled_legged_robot/)

#### StackForce High Torque Wheel-Legged Robot

![](<images/Wheeled-Leg Robot-image-1.png>)

The **StackForce Large Wheeled-Leg Robot** uses precise kinematic models and adaptive algorithms to continuously monitor and adjust posture and speed, preventing tipping and rollover. This ensures stable operation on uneven terrain, during sharp turns, or on steep slopes, both indoors and outdoors.

Equipped with brushless motors, it delivers smooth and reliable motion. The robot also includes comprehensive installation guides and video tutorials to help developers quickly get started and build innovative applications, such as implementing LQR control, reinforcement learning, or integrating vision-based perception and navigation.

[International site purchasing channels🛒](https://www.seeedstudio.com/Fashionstar-Star-Arm-Viola-Violin-p-6497.html)    <span style="color: inherit; background-color: rgb(247,105,100)">（等待修改）</span>

| Specification Item | Detailed Description                                      |
|--------------------|-----------------------------------------------------------|
| Main Control Board | StackForce Control Board                                  |
| Motor Driver Board | 25A High-Power Brushless Motor Driver                     |
| Power Supply       | 22.2V 6S Lithium Polymer Battery (RC Grade)                |
| Motors             | Joint Motors + Wheel Motors                               |
| Total Weight       | 10 kg                                                     |
| Dimensions         | 50 cm × 56 cm (L × W), Height: 16–50 cm                    |


Other references:

wiki：[StackForce High Torque Wheel-Legged Robot](https://wiki.seeedstudio.com/lerobot_starai_arm/)

### Github

<span style="color: inherit; background-color: rgb(247,105,100)">（等待修改）</span>
