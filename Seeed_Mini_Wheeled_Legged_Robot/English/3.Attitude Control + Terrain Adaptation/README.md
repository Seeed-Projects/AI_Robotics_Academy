## Chapter 3. Attitude Control + Terrain Adaptation

### Inverse Kinematics

Inverse kinematics can calculate the required servo angles based on the desired end position of a wheeled-legged robot's foot, thereby adjusting the leg's state and achieving different attitude controls.

* **Inverse kinematics:&#x20;**&#x47;iven the desired foot-end coordinates $B(x, y)$, find the joint angles $\alpha, \beta$。

  * Significance: Achieve precise conversion from where the foot is intended to go to how many degrees the motor should rotate.

![](<images/Wheeled-Leg Robot-image-18.png>)

**Derivation of Joint Angle&#x20;**$\alpha$

**Objective**:
To compute the left motor angle $\alpha$ given the known position of point $B(x, y)$.

**1. Establishing the Basic Equations**

According to the linkage geometry, the coordinates of the foot point $B$ can be expressed as:

* $x = L_1 \cos\alpha + L_2 \cos\theta$

* $y = L_1 \sin\alpha + L_2 \sin\theta$

**2. Eliminating the Intermediate Variable $\theta$**

To eliminate the unknown variable $\theta$, the equations are rearranged as:

**$$(x - L_1 \cos\alpha)^2 + (y - L_1 \sin\alpha)^2 = (L_2 \cos\theta)^2 + (L_2 \sin\theta)^2$$**

Expanding the equation and applying the identity $\sin^2\theta + \cos^2\theta = 1$ yields:

**$$x^2 + y^2 - 2xL_1 \cos\alpha - 2yL_1 \sin\alpha + L_1^2 = L_2^2$$**

**3. Constructing the Standard Trigonometric Equation**

The above expression can be rearranged into the standard form:

**$$a \cos\alpha + b \sin\alpha = c$$**

where:

* $a = 2xL_1$

* $b = 2yL_1$

* $c = x^2 + y^2 + L_1^2 - L_2^2$

**4. Half-Angle Substitution and Solution**

By introducing the substitution $\tan\left(\frac{\alpha}{2}\right)$, the equation is transformed into a quadratic equation in

$$X = \tan\left(\frac{\alpha}{2}\right)$$

resulting in:

$$(-a - c)X^2 + 2bX + (a - c) = 0$$

Solving for $X$ using the quadratic formula, the joint angle \$\alpha\$ is finally obtained as:

$$\alpha = 2 \arctan\left(\frac{b \pm \sqrt{a^2 + b^2 - c^2}}{a + c}\right)$$

**Derivation of Joint Angle $\beta$**

**Objective**:
To compute the right motor angle \$\beta\$ given the known position of point \$B(x, y)\$.

**1. Establishing the Relationship via Vector Addition**

The coordinates of point $C$ are determined by the right-side linkage $\beta$) and the horizontal offset $L_5$:

$$\vec{OC} = \vec{OD} + \vec{DC} = (L_5 + L_4 \cos\beta,; L_4 \sin\beta)$$

**2. Constructing the Standard Equation for $\beta$**

Similarly, using the coordinates of point $B$ and the link length $L_3$, the intermediate variables can be eliminated to obtain:

$$d \cos\beta + e \sin\beta = f$$

where:

* $d = 2(x - L_5)L_4$

* $e = 2yL_4$

* $f = (x - L_5)^2 + L_4^2 + y^2 - L_3^2$

**3. Final Solution**

Following the same procedure used for $\beta$ is given by:

$$\beta = 2 \arctan\left(\frac{e \pm \sqrt{d^2 + e^2 - f^2}}{d + f}\right)$$
> &#x20;lesson6\_HeightCtrl / src / main.cpp&#x20;
>
> Implementing mathematical calculations through code is a complex process. The demonstration code may have some differences and omissions, but the variables obtained in the end are correct, which requires learners to understand patiently.

| Variable Name        | Meaning                                                     | Variable Name         | Meaning                                                      |
|----------------------|-------------------------------------------------------------|-----------------------|--------------------------------------------------------------|
| IKParam.XLeft        | Left leg X-coordinate                                       | IKParam.XRight        | Right leg X-coordinate                                       |
| IKParam.YLeft        | Left leg Y-coordinate                                       | IKParam.YRight        | Right leg Y-coordinate                                       |
| aLeft                | Left leg intermediate variable a                            | aRight                | Right leg intermediate variable a                            |
| bLeft                | Left leg intermediate variable b                            | bRight                | Right leg intermediate variable b                            |
| cLeft                | Left leg intermediate variable c                            | cRight                | Right leg intermediate variable c                            |
| dLeft                | Left leg intermediate variable d                            | dRight                | Right leg intermediate variable d                            |
| eLeft                | Left leg intermediate variable e                            | eRight                | Right leg intermediate variable e                            |
| fLeft                | Left leg intermediate variable f                            | fRight                | Right leg intermediate variable f                            |
| alpha1               | Two solutions of the quadratic equation for the rear leg    | beta1                 | Two solutions of the quadratic equation for the front leg    |
| alpha2               | Two solutions of the quadratic equation for the rear leg    | beta2                 | Two solutions of the quadratic equation for the front leg    |
| IKParam.alphaLeft    | Final α angle of the left rear leg                          | IKParam.alphaRight    | Final α angle of the right rear leg                          |
| IKParam.betaLeft     | Final β angle of the left front leg                         | IKParam.betaRight     | Final β angle of the right front leg                         |
| alphaLeftToAngle     | Left rear leg α angle converted from radians to degrees     | alphaRightToAngle     | Right rear leg α angle converted from radians to degrees     |
| betaLeftToAngle      | Left front leg β angle converted from radians to degrees    | betaRightToAngle      | Right front leg β angle converted from radians to degrees    |
| servoLeftFront       | Target angle of the left front leg servo                    | servoRightFront       | Target angle of the right front leg servo                    |
| servoLeftRear        | Target angle of the left rear leg servo                     | servoRightRear        | Target angle of the right rear leg servo                     |


![](<images/Wheeled-Leg Robot-image-19.png>)

### Compliant Control

To avoid abrupt motor motions, the foot-end coordinates must be generated in a continuous and dense manner.

**Parameter Setting: $X=0$**

In the self-balancing state,  $X=0$ is typically set to ensure that the leg moves only in the vertical direction (the $Y$axis), which simplifies the control model.

**Dense Point Generation Formula ($Kp_Y$)**

Instead of jumping directly to the target height, the controller iteratively updates the foot position using the following equation:

$$Y = Y + Kp\_Y \times (Y_{demand} - Y)$$

* **$Y_{demand}$**：The desired target leg height.

* **Effect of $Kp\_Y$**：

  * **Smaller values:&#x20;**&#x67;enerate denser $B(x, y)$ coordinate points, resulting in smoother servo motion.

  * **Larger values:&#x20;**&#x70;roduce faster movements but may introduce mechanical shock.

![](<images/Wheeled-Leg Robot-image-20.png>)

**Implementation Process**

1. **Input**：Set the target leg height $Y_{demand}$。

2. **Planning**：Use $Kp\_Y$ to iteratively generate dense intermediate coordinate points.

3. **Inverse Kinematics**：Substitute each point into the inverse kinematics equations to compute the corresponding joint angles $\alpha$ and $\beta$ in real time.

4. **Execution**： Continuously send the computed joint angles to the servos, enabling smooth squatting or balanced motion.

### Attitude Control

#### 4. Height Control &#x20;

Referring to **compliant control**, it is only necessary to modify the coordinate point $y$ while keeping the coordinate point $x = 0$ unchanged. The corresponding $\alpha$ and $\beta$ are then calculated using the inverse kinematics equations and output to the servos.

#### 5. Roll Attitude Control &#x20;

Roll control essentially causes the robot to tilt left or right. For a wheel-legged robot, this does not require additional actuators; instead, it is achieved by adjusting the **desired heights** of the left and right legs ($L\_Height\_Demand$, $R\_Height\_Demand$).

**Conversion Between Roll Angle and Height Difference:&#x20;**&#x54;aking the center of the robot body as the rotation axis, when it rotates counterclockwise by an angle $\phi$, the left side moves downward while the right side moves upward.

**The height difference is calculated as:&#x20;**$E_H = \frac{I}{2} \cdot \sin(\phi)$

* $I$：Width of the robot body.

* $\phi$：Desired roll angle given by the remote control.

**Height distribution of left and right legs**：

* **Desired height of the left leg**：$L\_Height\_Demand = Remoter\_Input + E_H$

* **Desired height of the right leg**：$R\_Height\_Demand = Remoter\_Input - E_H$



![](<images/Wheeled-Leg Robot-image-21.png>)

> &#x20;lesson7\_RollCtrl / src / main.cpp&#x20;
>
> As shown in the code, the desired height and roll angle are obtained from the remote control data. First, the current control height is calculated using compliant control, and then the height difference is solved to calculate the desired heights of the left and right legs.

![](<images/Wheeled-Leg Robot-image-22.png>)

### Complex terrain adaptation

Complex terrain adaptation refers to situations where the robot experiences lateral body tilt when placed on a slope or uneven surface. To maintain a level and stable posture, the robot must compensate for this tilt by **adjusting the effective length of its legs**.

![](<images/Wheeled-Leg Robot-image-23.png>)



#### Incremental Compensation Algorithm

$$stab\_roll = stab\_roll + Kp\_roll \times (0 - mpu\_roll)$$

* **stab\_roll:&#x20;**&#x52;oll attitude compensation angle. &#x20;

* **Kp\_roll:&#x20;**&#x43;onversion coefficient (proportional gain), used to control the robot’s **recovery speed** in response to terrain changes. &#x20;

* **0 - mpu\_roll:&#x20;**&#x54;he error between the target angle (0 degrees) and the actual angle measured by the gyroscope.

![](<images/Wheeled-Leg Robot-image-24.png>)



#### Execution Process Analysis

During operation, the robot control program runs continuously in a loop. **At a very high update frequency**, the algorithm incrementally increases the compensation until balance is restored.

**Stage 1: Imbalance Detection**

* **Initial state:&#x20;**&#x57;hen the robot enters a slope, the roll angle $mpu\_roll$ measured by the gyroscope is no longer zero. &#x20;

* **Error calculation:&#x20;**&#x54;he term $(0 - mpu\_roll)$ in the formula represents the error between **the target level attitude** and **the actual tilted attitude**.

**Stage 2: Generation of Incremental Compensation**

* **Dynamic response:&#x20;**&#x54;he more uneven the terrain is (the larger $mpu\_roll$ becomes), the larger the computed error. &#x20;

* **Gain scaling:&#x20;**&#x54;his error is multiplied by $Kp\_roll$ (the conversion coefficient) to generate **an incremental compensation term**. &#x20;

* **Accumulation effect:**：Because the program runs in a high-frequency loop, this incremental term is continuously added to the existing $stab\_roll$. As a result, the variable $stab\_roll$ keeps increasing, producing **a progressively larger compensation angle**.

![](<images/Wheeled-Leg Robot-image-25.png>)

**Stage 3: Leg Motion Feedback**

* **Command execution:&#x20;**&#x54;he generated $stab\_roll$ is fed in real time into **the roll attitude control module**. &#x20;

* **Physical compensation:&#x20;**&#x42;ased on this value, the controller computes and **adjusts the extension and retraction of the left and right legs**. &#x20;

* **Attitude correction:&#x20;**&#x41;s the legs move, the robot body gradually returns toward a horizontal orientation.

**Stage 4: Achievement of Dynamic Balance**

* **Convergence to zero:&#x20;**&#x41;s the body approaches a level state (with $mpu\_roll$ approaching 0), the incremental term $Kp\_roll \times (0 - mpu\_roll)$ also approaches zero. &#x20;

* **Termination of accumulation:&#x20;**&#x4F;nce $mpu\_roll$ equals **0**, the **increment** in the equation **disappears**, and $stab\_roll$ **stops increasing**, remaining at **a fixed value** that maintains robot balance. &#x20;

* **Self-stabilization completed:&#x20;**&#x41;t this point, the robot successfully maintains a horizontal attitude on the slope, completing roll attitude self-stabilization.

![](<images/Wheeled-Leg Robot-image-26.png>)

> &#x20;lesson8\_adaptive / src / main.cpp&#x20;
>
> As the code shows, the essence is to replace the height difference (E\_H) of roll control with the roll attitude compensation angle (stab\_roll).

![](<images/Wheeled-Leg Robot-image-27.png>)

