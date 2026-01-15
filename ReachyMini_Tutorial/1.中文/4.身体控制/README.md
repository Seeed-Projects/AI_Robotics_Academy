# 身体控制 - Reachy Mini SDK

## 概述

身体控制系统管理 Reachy Mini 的身体绕垂直轴（偏航）的旋转。这是 7-DOF 头部系统中的第一个关节，为所有头部运动提供基础旋转。

## 身体偏航控制

### 直接设置身体偏航

```python
# 以弧度为单位设置身体偏航角
reachy_mini.set_target_body_yaw(body_yaw=0.5)  # 向左旋转 0.5 弧度

# 负值向右旋转
reachy_mini.set_target_body_yaw(body_yaw=-0.3)  # 向右旋转 0.3 弧度
```

### 平滑运动设置身体偏航

```python
# 平滑旋转到目标
reachy_mini.goto_target(
    head=None,
    antennas=None,
    body_yaw=0.5,
    duration=1.0,
    method="MIN_JERK"
)
```

### 快速设置方法

```python
# 便捷方法
reachy_mini.set_target(
    head=None,
    antennas=None,
    body_yaw=0.5
)
```

## 自动身体偏航

自动身体偏航功能允许机器人自动调整其身体旋转以更好地到达目标头部位置。

### 启用自动身体偏航

```python
# 启用（默认）
reachy_mini.set_automatic_body_yaw(automatic_body_yaw=True)
```

### 禁用自动身体偏航

```python
# 禁用 - 身体偏航保持固定
reachy_mini.set_automatic_body_yaw(automatic_body_yaw=False)
```

### 使用自动身体偏航设置初始化

```python
from reachy_mini import ReachyMini

# 创建时启用自动身体偏航（默认）
reachy_mini = ReachyMini(automatic_body_yaw=True)

# 或禁用它
reachy_mini = ReachyMini(automatic_body_yaw=False)
```

## 自动身体偏航的工作原理

启用后，运动学系统会自动计算到达给定头部姿态的最佳身体偏航角。这提供了：

- **扩展工作空间**：可以到达固定身体偏航无法到达的位置
- **自然运动**：模仿人类自然旋转身体的方式
- **最佳姿态**：为每个头部姿态找到最舒适的身体位置

### 示例：自动与手动

```python
# 使用自动身体偏航（启用）
reachy_mini.set_automatic_body_yaw(True)
reachy_mini.goto_target(
    head=(0.2, 0.1, 0.1, 0.0, 0.0, 0.0),
    duration=1.0
)
# 身体偏航自动计算

# 使用手动身体偏航（禁用）
reachy_mini.set_automatic_body_yaw(False)
reachy_mini.goto_target(
    head=(0.2, 0.1, 0.1, 0.0, 0.0, 0.0),
    body_yaw=0.5,  # 必须手动指定
    duration=1.0
)
```

## 示例：身体旋转扫描

```python
import time

# 使用身体偏航左右扫描
angles = [-0.5, -0.25, 0.0, 0.25, 0.5, 0.0]

for angle in angles:
    reachy_mini.goto_target(
        body_yaw=angle,
        duration=0.5
    )
    time.sleep(0.5)
```

## 示例：跟踪物体

```python
# 模拟跟踪从左到右移动的物体
import time
import math

for i in range(20):
    t = i / 20.0

    # 物体沿弧线移动
    x = 0.3 * math.cos(math.pi * t)
    y = 0.3 * math.sin(math.pi * t)

    # 使用自动身体偏航注视物体
    reachy_mini.look_at_world(
        x=x, y=y, z=0.2,
        duration=0.1,
        perform_movement=True
    )

    time.sleep(0.1)
```

## 示例：问候动作

```python
import time

# 鞠躬并转身问候
def greet(direction):
    """
    direction: 1 表示向右，-1 表示向左
    """
    # 转动身体
    reachy_mini.goto_target(
        body_yaw=direction * 0.4,
        duration=0.5
    )

    # 点头
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.05, 0.0, 0.2, 0.0),
        duration=0.3
    )

    time.sleep(0.5)

    # 回到中性位置
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        body_yaw=0.0,
        duration=0.5
    )

# 向右问候
greet(direction=1)

time.sleep(1.0)

# 向左问候
greet(direction=-1)
```

## 身体偏航限制

| 参数 | 值 | 描述 |
|-----------|-------|-------------|
| 最小角度 | ~-1.0 rad | 最大右旋转 |
| 最大角度 | ~1.0 rad | 最大左旋转 |
| 中间位置 | 0.0 rad | 面向前方 |

注意：实际限制可能基于校准和机械约束而有所不同。

## 关节位置参考

获取关节位置时，身体偏航是第一个元素：

```python
head_joints, antenna_joints = reachy_mini.get_current_joint_positions()

body_yaw = head_joints[0]  # 第一个关节
stewart_joints = head_joints[1:]  # 其余 6 个关节

print(f"身体偏航: {body_yaw} 弧度")
print(f"Stewart 平台: {stewart_joints}")
```
