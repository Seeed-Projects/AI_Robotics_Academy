## Chapter 2. Pitch angle self-balancing movement

### Principle of Pitch Stability

The self-balancing effect of a wheeled-leg robot is to maintain the body in an upright state perpendicular to the ground. Essentially, **balance is a game of torques**.

![](<images/Wheeled-Leg Robot-image-2.png>)



#### 1. Tipping Torque

When the robot’s center of gravity (CoG) is not directly above the wheel axle, gravity generates a torque that causes the robot to fall.

* **Physical analysis:** When the robot has a forward tilt angle $\theta$, the gravitational force $G$ can be decomposed into a component $F_1$ that is perpendicular to the leg.

* **Moment arm $L$:&#x20;**&#x54;he distance from the center of gravity to the center of the wheel axle.

* **Tipping torque: $T_1 = F_1 \cdot L$**。

* **Conclusion:&#x20;**&#x54;he larger the tilt angle $\theta$ or the longer the leg length $L$, the greater the tipping torque, and the faster the robot will fall.



#### 2. Restoring Torque&#x20;

To counteract $T_1$, a restoring torque in the opposite direction must be generated through the acceleration of the wheels.

* **Control logic**:

  * **When tilting forward:&#x20;**&#x74;he wheels must rapidly accelerate forward so that the chassis moves underneath the center of gravity.

  * **When tilting backward:&#x20;**&#x74;he wheels must rapidly accelerate backward.

* **Objective:&#x20;**&#x54;o keep the pitch angle always approaching 0°.

| ![](<images/Wheeled-Leg Robot-image-3.png>) | ![](<images/Wheeled-Leg Robot-image-4.png>) |
| ------------------------------------------- | ------------------------------------------- |



#### 3. **P-Controller**

In this self-balancing system, the **P-Controller** (Proportional Control) is the most basic and core component. Its task is to compute the required **“uprighting effort” (torque)** in real time based on the current **“degree of falling” (tilt angle)**.



**1. Core Formula**

We establish a linear proportional relationship between the tilt angle and the torque:

$$\text{Torque} = k_p \times \text{Pitch}$$

* **Pitch**：The current attitude deviation (input).

* $k_p$**&#x20;(proportional gain)**：The proportional coefficient that determines the robot’s “sensitivity” to tilting.

* **Torque (restoring torque)**：The reaction torque output by the wheels acting on the ground (output).

![](<images/Wheeled-Leg Robot-image-5.png>)



**2. Closed-Loop Feedback Process**

As shown in the figure above, the robot’s self-balancing behavior is a continuous closed-loop process:

1. **Sensing:&#x20;**&#x54;he gyroscope module on the control board measures the pitch-axis tilt angle (Pitch) in real time.

![](<images/Wheeled-Leg Robot-image-6.png>)

* **Computation:** The measured angle is converted into a target restoring torque (Torque) through the proportional gain $k_p$.

* **Actuation:&#x20;**&#x54;he motors/wheels respond by outputting torque to correct the robot’s posture.

* **Looping:** The above steps are continuously repeated to achieve dynamic equilibrium.

> &#x20;lesson3\_poseRead / src / main.cpp
>
> The code demonstrates the data reading of the gyroscope and saves it to the corresponding variable.

![](<images/Wheeled-Leg Robot-image-7.png>)

**3. Tuning Key: Finding the Optimal$k_p$**

The process of tuning $k_p$ is essentially a trade-off between **response speed** and **system stability**.

Moreover, the $k_p$ values corresponding to **different height attitude&#x20;**&#x77;ill also vary.

![](<images/Wheeled-Leg Robot-image-8.png>)

> &#x20;lesson3\_Stable / src / main.cpp&#x20;
>
> The code provides three different height postures, corresponding to $k_p1$ $k_p2$ $k_p3$, three different proportional coefficients.

![](<images/Wheeled-Leg Robot-image-9.png>)

### Self-Balancing Optimization + Forward/Backward Motion

#### Limitations of Pitch-Only Self-Balancing

To further improve the stability of a wheeled-leg robot, **relying solely on tilt correction is insufficient and can easily result in a fall.**

When the robot tilts, the horizontal component of gravity immediately induces forward or backward acceleration. However, sensors can only detect the tilt after it occurs, by which time a certain amount of horizontal velocity has already accumulated. Although the pitch self-balancing controller can generate a restoring torque to straighten the body, **it cannot eliminate the existing horizontal velocity.** Due to inertia, the robot continues to accelerate until the motor reaches its physical limits and can no longer provide sufficient torque, ultimately causing a fall.

![](<images/Wheeled-Leg Robot-image-10.png>)



#### Wheel Speed Control

To fundamentally solve the stability problem, **wheel speed control** must be introduced. By actively adjusting the robot’s target posture, the accumulated horizontal velocity can be effectively compensated.

1. **Deceleration by Backward Tilt**

* When the robot develops forward velocity, the controller no longer sets the target pitch angle to 0.

* Instead, the controller computes an appropriate **backward tilt angle (deceleration pitch)**.

* By actively tilting backward, the robot uses the opposing component of gravity as a “brake” to counteract forward inertia, allowing the velocity to gradually return to zero.

![](<images/Wheeled-Leg Robot-image-11.png>)

* **Algorithm Logic Implementation**

The control program runs in real-time through the following two core loops

A. Speed loop: calculate **target angle (deceleration tilt)**

$$targetAngle = Kp \times (0 - speedAvg)$$

* **speedAgg: Current** average speed of the wheel read through the encoder.

* **Kp:&#x20;**&#x50;roportional conversion factor that converts the speed difference into an appropriate tilt angle.

![](<images/Wheeled-Leg Robot-image-12.png>)

B. Angle loop: Calculating Restoring Torque

$$Torque = Kp \times (targetAngle - pitch)$$

* The program calculates the compensating torque that the motor should output based on the **difference** between the **target tilt angle** and the **sensor-measured tilt angle**.

![](<images/Wheeled-Leg Robot-image-13.png>)



#### Forward/Backward Motion

In the previous 'Self-Balancing' logic, we hardcoded the target speed to `0`. If we want the robot to move, we just need to replace the `0` in the formula with a dynamic variable, **targetSpeed**.

1. Dynamic Variable Replacement

We have upgraded the speed loop formula to:

$$targetAngle = Kp \times (targetSpeed - speedAvg)$$

* **targetSpeed:&#x20;**&#x54;his is the target speed we want the robot to reach.&#x20;

* **Meaning:&#x20;**&#x54;he formula currently calculates the **difference&#x20;**&#x62;etween the **target speed** and the **actual speed**. The robot will automatically adjust its body tilt to eliminate this difference.

Angle loop formula unchanged:：

$$Torque = Kp \times (targetAngle - pitch)$$

![](<images/Wheeled-Leg Robot-image-14.png>)

* Detailed Motion Process

- **Start-up Phase:&#x20;**&#x57;hen you set $targetSpeed > 0$ (for example, commanding a forward speed of 0.5 m/s), $speedAvg$ is initially 0. The control equation therefore computes a positive forward target pitch angle ($targetAngle$). The robot tilts forward accordingly and **begins to accelerate by using the forward component of gravity**.

- **Constant-Speed Phase:&#x20;**&#x41;s the wheel speed increases, $speedAvg$ gradually approaches $targetSpeed$, and the speed error decreases. **The robot’s target pitch angle automatically returns toward upright.** When the actual speed equals the target speed, the robot **maintains a small tilt angle to sustain stable motion**.

- **Stopping Phase:&#x20;**&#x57;hen $targetSpeed$ is reset to 0, the controller detects that the current $speedAvg$ is much larger than the target value. It then computes a negative backward pitch angle (**deceleration tilt**), using gravity to rapidly “brake” the robot until it comes to a stop.

> &#x20;lesson4\_VelCtrl / src / main.cpp&#x20;
>
> The code obtains the wheel speed through motors.getBLDCData() and calculates the overall robot speed speedAvg. Then, the mpu6050.update() function updates the gyroscope data, the setRobotHeight function sets the servo angle to fix the leg posture. Finally, the torque is calculated according to the wheel speed control formula, and the motors.setTargets function is used to send wheel control commands.

![](<images/Wheeled-Leg Robot-image-15.png>)

### **Turn control**

The robot achieves steering functionality in the same way as a tank, by turning through th&#x65;**&#x20;speed difference between the left and right wheels**.

Based on the previously calculated restoring torque ($Torque$), superimposed a steering torque ($turnTorque$):

* **Left wheel output&#x20;**= $Torque + turnTorque$

* **right wheel output&#x20;**= $Torque - turnTorque$

When $turnTorque$ is positive, the thrust of the left wheel increases while the thrust of the right wheel decreases, causing the robot to turn right; conversely, when $turnTorque$ is negative, the robot turns left.

![](<images/Wheeled-Leg Robot-image-16.png>)

> &#x20;lesson5\_TurnCtrl / src / main.cpp&#x20;
>
> It is generally the same as lesson4\_VelCtrl / src / main.cpp, and only needs to introduce the steering torque to calculate the desired torque.

![](<images/Wheeled-Leg Robot-image-17.png>)

