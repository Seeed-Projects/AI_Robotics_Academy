# 插值与运动 - Reachy Mini SDK

## 概述

Reachy Mini 提供多种插值技术，用于创建平滑、自然的运动。插值系统控制机器人在姿态之间如何转换。

## 插值方法

| 方法 | 描述 | 最适用于 |
|--------|-------------|----------|
| `LINEAR` | 线性插值 | 简单、可预测的运动 |
| `MIN_JERK` | 最小加加速度轨迹 | 最平滑、最自然的运动 |
| `EASE_IN_OUT` | 缓入缓出 | 温和的启动和停止 |
| `CARTOON` | 卡通弹跳效果 | 有趣、表现力强的运动 |

## 将插值与 goto_target 一起使用

### 基本用法

```python
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    duration=1.0,
    method="MIN_JERK"  # 插值方法
)
```

### 线性插值

```python
# 整个过程恒定速度
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    duration=1.0,
    method="LINEAR"
)
```

### 最小加加速度（推荐）

```python
# 最平滑的运动，最小化加速度变化
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    duration=1.0,
    method="MIN_JERK"
)
```

### 缓入缓出

```python
# 温和启动，加速，然后减速
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    duration=1.0,
    method="EASE_IN_OUT"
)
```

### 卡通效果

```python
# 有弹性、卡通式的运动
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    duration=1.0,
    method="CARTOON"
)
```

## 运动持续时间

持续时间参数控制运动需要多长时间。

### 快速运动

```python
# 快速运动（0.3 秒）
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    duration=0.3,
    method="MIN_JERK"
)
```

### 慢速运动

```python
# 慢速、深思熟虑的运动（2 秒）
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    duration=2.0,
    method="MIN_JERK"
)
```

## 示例：比较插值方法

```python
import time

# 目标位置
target = (0.0, 0.0, 0.1, 0.0, 0.0, 0.0)

methods = ["LINEAR", "MIN_JERK", "EASE_IN_OUT", "CARTOON"]

for method in methods:
    print(f"使用 {method}...")

    # 移动到目标
    reachy_mini.goto_target(
        head=target,
        duration=1.0,
        method=method
    )

    time.sleep(1.5)  # 等待运动完成

    # 回到中间
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        duration=1.0,
        method=method
    )

    time.sleep(1.5)
```

## 示例：平滑序列

```python
import time

# 定义姿态序列
poses = [
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),   # 中间
    (0.0, -0.1, 0.1, 0.0, 0.0, -0.3), # 左
    (0.0, 0.0, 0.15, 0.0, -0.1, 0.0), # 上
    (0.0, 0.1, 0.1, 0.0, 0.0, 0.3),   # 右
    (0.0, 0.0, 0.05, 0.0, 0.2, 0.0),  # 下
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),   # 中间
]

# 使用平滑插值执行序列
for pose in poses:
    reachy_mini.goto_target(
        head=pose,
        duration=1.0,
        method="MIN_JERK"  # 最平滑
    )
    time.sleep(1.1)  # 略长于持续时间
```

## 示例：动态持续时间

```python
import math

def calculate_duration(distance, max_speed=0.2):
    """基于距离计算持续时间"""
    return distance / max_speed

# 当前位置
current_head = reachy_mini.get_current_head_pose()
current_pos = current_head[:3, 3]

# 目标位置
target = create_head_pose(0.1, 0.0, 0.15, 0.0, 0.0, 0.0)
target_pos = target[:3, 3]

# 计算距离
distance = np.linalg.norm(target_pos - current_pos)

# 计算持续时间
duration = calculate_duration(distance)

# 使用计算的持续时间移动
reachy_mini.goto_target(
    head=(0.1, 0.0, 0.15, 0.0, 0.0, 0.0),
    duration=duration,
    method="MIN_JERK"
)
```

## 辅助函数

### 最小加加速度轨迹

```python
from reachy_mini.utils import minimum_jerk
import numpy as np

# 定义起点和目标位置
start_pos = np.array([0.0, 0.0, 0.0])
goal_pos = np.array([0.1, 0.0, 0.1])

# 创建最小加加速度轨迹
trajectory_func = minimum_jerk(
    starting_position=start_pos,
    goal_position=goal_pos,
    duration=1.0,
    sample_time=0.01
)

# 在不同时间评估
for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    position = trajectory_func(t)
    print(f"t={t:.2f}: {position}")
```

### 时间轨迹

```python
from reachy_mini.utils import time_trajectory

# 获取不同方法的时间轨迹值
methods = ["LINEAR", "MIN_JERK", "EASE_IN_OUT", "CARTOON"]

for method in methods:
    print(f"\n{method}:")
    for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
        value = time_trajectory(t, method)
        print(f"  t={t:.2f}: {value:.3f}")
```

## 示例：自定义动画

```python
import numpy as np
import time

def create_animation_keyframes(duration=2.0, fps=30):
    """创建动画关键帧"""
    keyframes = []
    num_frames = int(duration * fps)

    for i in range(num_frames):
        t = i / num_frames

        # 创建圆形运动
        x = 0.1 * np.cos(2 * np.pi * t)
        y = 0.1 * np.sin(2 * np.pi * t)
        z = 0.15 + 0.05 * np.sin(4 * np.pi * t)

        keyframes.append((x, y, z, 0.0, 0.0, 0.0))

    return keyframes

# 生成关键帧
keyframes = create_animation_keyframes(duration=3.0)

# 播放动画
frame_duration = 1.0 / 30  # 30 FPS

for pose in keyframes:
    reachy_mini.goto_target(
        head=pose,
        duration=frame_duration,
        method="MIN_JERK"
    )
    time.sleep(frame_duration)
```

## 示例：贝塞尔曲线运动

```python
import numpy as np

def bezier_point(t, control_points):
    """计算时间 t 时贝塞尔曲线上的点"""
    n = len(control_points) - 1
    point = np.zeros(3)

    for i, cp in enumerate(control_points):
        # 伯恩斯坦多项式
        coeff = np.math.comb(n, i) * (1-t)**(n-i) * t**i
        point += coeff * np.array(cp[:3])

    return point

# 定义控制点
control_points = [
    (0.0, 0.0, 0.0),
    (0.05, -0.05, 0.1),
    (0.1, 0.05, 0.15),
    (0.1, 0.0, 0.1)
]

# 沿贝塞尔曲线移动
steps = 50

for i in range(steps):
    t = i / (steps - 1)
    point = bezier_point(t, control_points)

    reachy_mini.goto_target(
        head=(point[0], point[1], point[2], 0.0, 0.0, 0.0),
        duration=0.05,
        method="MIN_JERK"
    )
```

## 最佳实践

### 1. 使用 MIN_JERK 进行自然运动

```python
# 默认推荐
reachy_mini.goto_target(
    head=target,
    duration=1.0,
    method="MIN_JERK"  # 最自然
)
```

### 2. 根据距离调整持续时间

```python
# 较长距离 = 较长持续时间
duration = max(0.5, distance * 5)  # 最小 0.5 秒
```

### 3. 与身体运动结合

```python
# 协调头部、天线和身体
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
    antennas=(0.5, 0.5),
    body_yaw=0.0,
    duration=1.0,
    method="MIN_JERK"
)
```
