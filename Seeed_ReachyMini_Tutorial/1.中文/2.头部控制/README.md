# 头部控制 - Reachy Mini SDK

## 概述

Reachy Mini 的头部具有 7 个自由度（DOF），由 Stewart 平台机构控制。这实现了精确定位和平滑运动。

## 头部姿态控制

### 获取当前头部姿态

```python
# 获取当前 4x4 头部姿态矩阵
pose = reachy_mini.get_current_head_pose()
print(pose)
# 输出: 4x4 齐次变换矩阵
```

### 设置目标头部姿态

```python
import numpy as np

# 创建目标姿态 (x, y, z, roll, pitch, yaw)
from reachy_mini.utils import create_head_pose

target_pose = create_head_pose(
    x=0.0, y=0.0, z=0.1,  # 向前 10cm
    roll=0.0, pitch=0.1, yaw=0.0,  # 稍微向下看
    mm=False, degrees=True
)

# 立即设置
reachy_mini.set_target_head_pose(target_pose)

# 或使用平滑过渡
reachy_mini.goto_target(
    head=target_pose,
    duration=1.0,  # 1 秒
    method="MIN_JERK"
)
```

## 注视功能

### 注视图像坐标

```python
# 注视相机图像中的特定像素
joint_positions = reachy_mini.look_at_image(
    u=320,  # 像素 x 坐标
    v=240,  # 像素 y 坐标
    duration=0.5,
    perform_movement=True
)
```

### 注视世界坐标

```python
# 注视世界空间中的 3D 点
joint_positions = reachy_mini.look_at_world(
    x=0.5,  # 米
    y=0.0,
    z=0.3,
    duration=1.0,
    perform_movement=True
)
```

## 关节位置控制

### 获取关节位置

```python
# 获取所有关节位置
head_joints, antenna_joints = reachy_mini.get_current_joint_positions()

print(f"头部关节 (7 DOF): {head_joints}")
print(f"天线关节 (2 DOF): {antenna_joints}")
```

### 头部关节结构

```
头部: 7 DOF
- [0] body_yaw: 身体绕垂直轴旋转
- [1-6] stewart_platform: 6 个关节用于 Stewart 平台控制
  控制: x, y, z 位置 + roll, pitch, yaw 方向
```

## 快速设置目标

### 设置目标姿态（便捷方法）

```python
# 使用单个参数设置目标
reachy_mini.set_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.1, 0.0),  # (x, y, z, roll, pitch, yaw)
    antennas=None,
    body_yaw=None
)
```

### 移动到目标（平滑运动）

```python
# 平滑移动到目标
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.1, 0.0, 0.1, 0.0),
    antennas=(0.5, 0.5),
    duration=2.0,
    method="MIN_JERK",
    body_yaw=0.0
)
```

## 插值方法

可用于平滑运动的方法：

| 方法 | 描述 |
|--------|-------------|
| `"LINEAR"` | 线性插值 |
| `"MIN_JERK"` | 最小加加速度轨迹（最平滑） |
| `"EASE_IN_OUT"` | 缓入缓出 |
| `"CARTOON"` | 卡通式弹跳效果 |

```python
# 使用不同的插值方法
methods = ["LINEAR", "MIN_JERK", "EASE_IN_OUT", "CARTOON"]

for method in methods:
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
        duration=1.0,
        method=method
    )
```

## 示例：点头动作

```python
import time

# 创建点头动作
for _ in range(3):
    # 向下看
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.05, 0.0, 0.2, 0.0),
        duration=0.5,
        method="MIN_JERK"
    )
    time.sleep(0.5)

    # 向前看
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        duration=0.5,
        method="MIN_JERK"
    )
    time.sleep(0.5)
```

## 示例：扫描动作

```python
# 从左到右扫描
positions = [
    (0.0, -0.1, 0.0, 0.0, 0.0, -0.3),  # 向左看
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),     # 中间
    (0.0, 0.1, 0.0, 0.0, 0.0, 0.3),    # 向右看
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),     # 中间
]

for pos in positions:
    reachy_mini.goto_target(
        head=pos,
        duration=1.0,
        method="MIN_JERK"
    )
    time.sleep(1.0)
```

## 姿态矩阵辅助函数

```python
from reachy_mini.utils import create_head_pose

# 从组件创建姿态矩阵
pose = create_head_pose(
    x=0.1,    # x 位置（米）
    y=0.05,   # y 位置（米）
    z=0.2,    # z 位置（米）
    roll=0.0,    # roll 旋转（弧度）
    pitch=0.1,   # pitch 旋转（弧度）
    yaw=0.0,     # yaw 旋转（弧度）
    mm=False,    # 使用米（True 为毫米）
    degrees=False # 使用弧度（True 为度）
)
```
