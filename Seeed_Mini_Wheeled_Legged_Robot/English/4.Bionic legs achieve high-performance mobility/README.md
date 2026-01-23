## Chapter 4. **Bionic legs achieve high-performance mobility**

This chapter focuses on explaining how to utilize **the X-axis control capability of a biomimetic parallel leg** to achieve **instantaneous braking** and **high maneuverability** beyond that of a conventional self-balancing vehicle.

### 一、 Core Concept: From Y-Axis to X-Axis Control

In previous lessons, we mainly used inverse kinematics to control the **Y coordinate** of the foot end (achieving leg extension/retraction, height control, and terrain adaptation). Now, we introduce **X coordinate** control:

* **X-coordinate control:&#x20;**&#x43;ontrols the forward and backward displacement of the foot end relative to the robot body’s center of mass. &#x20;

* **Physical effect:&#x20;**&#x43;hanging the X coordinate causes the projection of the center of mass to shift away from the foot end, generating **a lateral force component**.

### 二、 Physical Principle: Lateral Force and Acceleration/Deceleration

By adjusting the foot-end X coordinate, we can deliberately alter the force state acting on the robot, allowing the robot to perform instant braking and rapid acceleration:

![](<images/Wheeled-Leg Robot-image-28.png>)

> **Why Do Wheeled Robots Have Stronger Mobility Than Conventional Self-Balancing Vehicles?**
>
> The key difference between a wheeled robot and a conventional self-balancing vehicle is the use of flexible biomimetic parallel legs, which enable a shift from single wheel-torque control to active, multi-dimensional center-of-mass control. Traditional self-balancing vehicles rely on rigid structures and can only maintain balance passively. In contrast, wheeled robots actively regulate foot-end displacement along both the X (forward–backward) and Y (up–down) axes, allowing dynamic body leveling on complex terrain and generating lateral force through center-of-mass shifting during braking. This results in superior terrain adaptability, braking performance, and motion stability.

![](<images/Wheeled-Leg Robot-image-29.png>)



### 三、 Algorithm Implementation: Lower-Limb Assistive Motion Control

To enable the robot to dynamically assist with braking or acceleration, an algorithm is required to convert **velocity errors** into **foot-end displacement**.

![](<images/Wheeled-Leg Robot-image-30.png>)



**Control Formula**

$$X = -Kp\_x \times (targetSpeed - speedAvg)$$

* **$targetSpeed$**: The target value set by the remote controller or the control program.

* **$speedAvg$**:**&#x20;**&#x54;he current actual speed obtained from feedback of the wheel speed sensors on both sides.

* **$Kp\_x$**: Determines the sensitivity of converting the velocity error into leg motion.

**Example Analysis: Instantaneous Braking**

Assume the robot is moving forward with a positive **wheel speed** of $10$ rad/s . The system suddenly commands a full stop ($targetSpeed = 0$):

1. **Error Calculation:&#x20;**$(0 - 10) = -10$。

2. **Displacement Calculation:&#x20;**&#x49;f $Kp\_x = 2$, then $X = -2 \times (-10) = 20$ 。

3. **Physical Response:&#x20;**&#x54;he legs rapidly extend forward by $20$mm, shifting the robot’s center of mass backward and generating a strong backward force component to assist deceleration.

4. **Dynamic Retraction:&#x20;**&#x41;s the vehicle speed decreases, $speedAvg$ is reduced accordingly, and the value of $X$ automatically decreases. When the robot comes to a complete stop, $X$ returns to $0$, and the legs retract.

![](<images/Wheeled-Leg Robot-image-31.png>)

> &#x20;lesson9\_enhance / src / main.cpp&#x20;
>
> The code is located before inverse kinematics and is grouped with the foot end Y calculation. The desired end position X and Y are calculated together through inverse kinematics to obtain the desired servo joint positions.

![](<images/Wheeled-Leg Robot-image-32.png>)


