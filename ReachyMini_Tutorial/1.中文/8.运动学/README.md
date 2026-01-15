# 运动学 - Reachy Mini SDK

## 概述

运动学系统处理关节角度和头部姿态之间的数学转换。Reachy Mini 使用 Stewart 平台机构进行头部控制，需要专门的运动学。

## 运动学引擎

Reachy Mini 支持多个运动学引擎：

| 引擎 | 描述 | 用例 |
|--------|-------------|----------|
| `AnalyticalKinematics` | 解析解 | 默认，快速，准确 |
| `NNKinematics` | 神经网络 | 可选，实验性 |
| `PlacoKinematics` | Placo 求解器 | 可选，高级 |

解析运动学引擎默认使用，为大多数用例提供准确的解决方案。

## 正向运动学（FK）

将关节角度转换为头部姿态。

### 基本正向运动学

```python
# 关节角度: [body_yaw, stewart_1, ..., stewart_6]
joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# 计算正向运动学
pose = reachy_mini.kinematics.fk(joint_angles)

print(f"头部姿态:\n{pose}")
# 输出: 4x4 齐次变换矩阵
```

### 从姿态提取位置

```python
import numpy as np

pose = reachy_mini.kinematics.fk(joint_angles)

# 提取位置 (x, y, z)
position = pose[:3, 3]
print(f"位置: x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")
```

### 从姿态提取旋转

```python
import numpy as np

pose = reachy_mini.kinematics.fk(joint_angles)

# 提取旋转矩阵
rotation = pose[:3, :3]
print(f"旋转矩阵:\n{rotation}")
```

## 逆向运动学（IK）

将所需的头部姿态转换为关节角度。

### 基本逆向运动学

```python
import numpy as np

# 目标姿态（4x4 矩阵）
target_pose = np.array([
    [1, 0, 0, 0.1],
    [0, 1, 0, 0.0],
    [0, 0, 1, 0.2],
    [0, 0, 0, 1]
])

# 计算逆向运动学
joint_angles = reachy_mini.kinematics.ik(pose=target_pose)

print(f"关节角度: {joint_angles}")
# 输出: [body_yaw, stewart_1, ..., stewart_6]
```

### 带身体偏航的逆向运动学

```python
# 指定身体偏航角
joint_angles = reachy_mini.kinematics.ik(
    pose=target_pose,
    body_yaw=0.5  # 弧度
)
```

### 禁用碰撞检查

```python
# 禁用碰撞检查以加快计算
joint_angles = reachy_mini.kinematics.ik(
    pose=target_pose,
    check_collision=False
)
```

### 设置迭代次数

```python
# 设置求解器的迭代次数
joint_angles = reachy_mini.kinematics.ik(
    pose=target_pose,
    no_iterations=100
)
```

## 将运动学与 look_at 一起使用

`look_at` 函数内部使用逆向运动学来计算所需的关节角度。

### 注视图像点

```python
# 这在内部使用 IK
joint_positions = reachy_mini.look_at_image(
    u=320, v=240,
    duration=0.5,
    perform_movement=True
)
```

### 注视世界点

```python
# 这也在内部使用 IK
joint_positions = reachy_mini.look_at_world(
    x=0.5, y=0.0, z=0.3,
    duration=1.0,
    perform_movement=True
)
```

## 自动身体偏航

运动学系统可以自动计算到达目标姿态的最佳身体偏航角。

### 启用自动身体偏航

```python
# 在运动学中启用
reachy_mini.kinematics.set_automatic_body_yaw(True)

# 现在 IK 将自动计算身体偏航
joint_angles = reachy_mini.kinematics.ik(pose=target_pose)
```

### 禁用自动身体偏航

```python
# 在运动学中禁用
reachy_mini.kinematics.set_automatic_body_yaw(False)

# 必须手动指定身体偏航
joint_angles = reachy_mini.kinematics.ik(
    pose=target_pose,
    body_yaw=0.0
)
```

## 示例：计算可达位置

```python
import numpy as np
import math

def is_reachable(reachy_mini, x, y, z):
    """检查位置是否可达"""
    from reachy_mini.utils import create_head_pose

    # 创建目标姿态
    target_pose = create_head_pose(
        x=x, y=y, z=z,
        roll=0, pitch=0, yaw=0
    )

    try:
        # 尝试计算 IK
        joint_angles = reachy_mini.kinematics.ik(target_pose)
        return joint_angles is not None
    except:
        return False

# 测试各种位置
print("正在测试可达位置...")

for x in np.linspace(-0.2, 0.2, 5):
    for y in np.linspace(-0.2, 0.2, 5):
        for z in np.linspace(0.1, 0.3, 3):
            reachable = is_reachable(reachy_mini, x, y, z)
            status = "✓" if reachable else "✗"
            print(f"{status} ({x:.2f}, {y:.2f}, {z:.2f})")
```

## 示例：工作空间可视化

```python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def sample_workspace(reachy_mini, resolution=10):
    """采样工作空间并返回可达点"""
    reachable_points = []

    # 定义工作空间边界
    x_range = np.linspace(-0.2, 0.2, resolution)
    y_range = np.linspace(-0.2, 0.2, resolution)
    z_range = np.linspace(0.1, 0.3, resolution)

    for x in x_range:
        for y in y_range:
            for z in z_range:
                if is_reachable(reachy_mini, x, y, z):
                    reachable_points.append([x, y, z])

    return np.array(reachable_points)

# 采样工作空间
points = sample_workspace(reachy_mini, resolution=8)

# 可视化
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

if len(points) > 0:
    ax.scatter(points[:, 0], points[:, 1], points[:, 2],
               c=points[:, 2], cmap='viridis', s=10)

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Reachy Mini 工作空间')

plt.show()
```

## 示例：轨迹规划

```python
import numpy as np
from reachy_mini.utils import create_head_pose

def plan_trajectory(reachy_mini, start_pose, end_pose, steps=10):
    """规划两个姿态之间的轨迹"""
    trajectory = []

    for i in range(steps):
        t = i / (steps - 1)

        # 线性插值
        interpolated_pose = start_pose + t * (end_pose - start_pose)

        # 为每个点计算 IK
        joint_angles = reachy_mini.kinematics.ik(interpolated_pose)

        if joint_angles is not None:
            trajectory.append(joint_angles)

    return trajectory

# 定义起点和终点姿态
start = create_head_pose(0, 0, 0.15, 0, 0, 0)
end = create_head_pose(0.1, 0.1, 0.2, 0.1, 0, 0)

# 规划轨迹
trajectory = plan_trajectory(reachy_mini, start, end, steps=20)

# 执行轨迹
for joint_angles in trajectory:
    reachy_mini.set_target_antenna_joint_positions(
        antennas=joint_angles[-2:]  # 最后两个是天线
    )
    # 此处进行额外控制...
```

## 运动学辅助函数

### 创建头部姿态

```python
from reachy_mini.utils import create_head_pose

pose = create_head_pose(
    x=0.1, y=0.0, z=0.2,    # 位置
    roll=0.0, pitch=0.1, yaw=0.0,  # 方向
    mm=False,     # 使用米
    degrees=False # 使用弧度
)
```

### 线性姿态插值

```python
from reachy_mini.utils import linear_pose_interpolation

import numpy as np

start_pose = np.eye(4)
end_pose = create_head_pose(0.1, 0, 0.1, 0, 0, 0)

# 在 t=0.5（中间）插值
interpolated = linear_pose_interpolation(start_pose, end_pose, t=0.5)
```

### 计算旋转之间的角度

```python
from reachy_mini.utils import delta_angle_between_mat_rot

import numpy as np

R1 = np.eye(3)
R2 = create_head_pose(0, 0, 0, 0.1, 0, 0)[:3, :3]

# 计算旋转之间的角度
angle = delta_angle_between_mat_rot(R1, R2)
print(f"角度: {angle:.4f} 弧度")
```
