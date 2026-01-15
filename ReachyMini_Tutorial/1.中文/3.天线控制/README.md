# 天线控制 - Reachy Mini SDK

## 概述

Reachy Mini 有两个天线（左和右），可以独立控制。每个天线有 1 个自由度用于角度调整。

## 天线关节结构

```
天线: 2 DOF
- [0] right_angle: 右天线角度
- [1] left_angle: 左天线角度
```

## 获取天线位置

### 获取所有天线位置

```python
# 获取头部和天线位置
head_joints, antenna_joints = reachy_mini.get_current_joint_positions()

print(f"右天线: {antenna_joints[0]}")
print(f"左天线: {antenna_joints[1]}")
```

### 仅获取天线位置

```python
# 仅获取天线位置
antenna_positions = reachy_mini.get_present_antenna_joint_positions()

print(f"天线位置: {antenna_positions}")
```

## 设置天线位置

### 设置目标天线位置

```python
# 直接设置天线角度
reachy_mini.set_target_antenna_joint_positions(
    antennas=[0.5, 0.5]  # [右, 左]，单位为弧度
)

# 或通过 goto_target 进行平滑运动
reachy_mini.goto_target(
    head=None,
    antennas=[0.5, 0.5],  # [右, 左]
    duration=0.5
)
```

### 头部和天线联合控制

```python
# 同时控制头部和天线
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),  # 头部姿态
    antennas=[0.5, 0.5],                     # 天线角度
    duration=1.0,
    method="MIN_JERK"
)
```

## 快速设置方法

```python
# 用于快速定位的便捷方法
reachy_mini.set_target(
    head=None,           # 不改变头部
    antennas=[0.5, 0.5], # 设置天线角度
    body_yaw=None        # 不改变身体偏航
)
```

## 示例：天线摆动

```python
import time

# 交替摆动天线
for _ in range(3):
    # 右天线抬起
    reachy_mini.goto_target(
        antennas=[0.8, 0.0],
        duration=0.3
    )
    time.sleep(0.3)

    # 左天线抬起
    reachy_mini.goto_target(
        antennas=[0.0, 0.8],
        duration=0.3
    )
    time.sleep(0.3)

# 回到中间
reachy_mini.goto_target(
    antennas=[0.5, 0.5],
    duration=0.5
)
```

## 示例：天线舞动

```python
import time
import math

# 按模式动画天线
duration = 2.0
steps = 20

for i in range(steps):
    t = i / steps

    # 创建振荡运动
    right_angle = 0.5 + 0.3 * math.sin(2 * math.pi * t)
    left_angle = 0.5 + 0.3 * math.sin(2 * math.pi * t + math.pi)

    reachy_mini.set_target_antenna_joint_positions(
        antennas=[right_angle, left_angle]
    )

    time.sleep(duration / steps)

# 重置到中间
reachy_mini.set_target_antenna_joint_positions(
    antennas=[0.5, 0.5]
)
```

## 示例：表情动作

```python
# 不同的天线表情

# 开心/兴奋（两个天线抬起）
reachy_mini.goto_target(
    antennas=[0.8, 0.8],
    duration=0.5
)

# 好奇（一个天线抬起）
reachy_mini.goto_target(
    antennas=[0.7, 0.3],
    duration=0.5
)

# 警觉（两个天线向前）
reachy_mini.goto_target(
    antennas=[0.2, 0.2],
    duration=0.3
)

# 放松（中间位置）
reachy_mini.goto_target(
    antennas=[0.5, 0.5],
    duration=0.5
)
```

## 联合运动示例

```python
# 协调的头部和天线运动
import time

# 感兴趣地看着某物
reachy_mini.goto_target(
    head=(0.0, 0.1, 0.15, 0.0, -0.1, 0.3),  # 向右上看
    antennas=[0.7, 0.3],                       # 右天线抬起
    duration=1.0,
    method="MIN_JERK"
)

time.sleep(1.0)

# 转移注意力
reachy_mini.goto_target(
    head=(0.0, -0.1, 0.15, 0.0, -0.1, -0.3), # 向左上看
    antennas=[0.3, 0.7],                       # 左天线抬起
    duration=1.0,
    method="MIN_JERK"
)
```

## 天线角度范围

| 天线 | 最小角度 | 最大角度 | 中间位置 |
|---------|-----------|-----------|--------|
| 右 | ~0.0 rad | ~1.0 rad | 0.5 rad |
| 左 | ~0.0 rad | ~1.0 rad | 0.5 rad |

注意：实际范围可能因校准和机械限制而略有不同。
