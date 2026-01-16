# 录制 - Reachy Mini SDK

## 概述

Reachy Mini 可以在操作期间记录各种数据，包括关节位置、时间戳和其他遥测数据。这对于：
- 记录运动以供回放
- 分析机器人行为
- 创建运动数据集
- 调试

很有用

## 开始录制

```python
# 开始录制
reachy_mini.start_recording()

print("录制已开始...")
```

## 停止录制

```python
# 停止录制并获取数据
data = reachy_mini.stop_recording()

if data:
    print(f"已录制 {len(data)} 帧")
    print(f"第一帧: {data[0]}")
```

## 录制数据结构

每个录制的帧包含：

```python
{
    'timestamp': float,        # 自开始以来的时间
    'head_joints': [...],      # 头部关节位置（7 DOF）
    'antenna_joints': [...],   # 天线关节位置（2 DOF）
    'body_yaw': float,         # 身体偏航角
    # 可能存在其他字段
}
```

## 示例：录制和回放运动

```python
import time

# 开始录制
reachy_mini.start_recording()

# 执行运动
poses = [
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, -0.1, 0.1, 0.0, 0.0, -0.3),
    (0.0, 0.0, 0.15, 0.0, -0.1, 0.0),
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
]

for pose in poses:
    reachy_mini.goto_target(
        head=pose,
        duration=1.0,
        method="MIN_JERK"
    )
    time.sleep(1.1)

# 停止录制
data = reachy_mini.stop_recording()

print(f"已录制 {len(data)} 帧")
```

## 示例：回放录制的运动

```python
import time

# 假设我们有录制的数据
if data:
    print(f"正在回放 {len(data)} 帧...")

    for frame in data:
        # 提取关节位置
        head_joints = frame.get('head_joints')
        antenna_joints = frame.get('antenna_joints')

        if head_joints:
            # 设置关节位置
            reachy_mini.set_target_antenna_joint_positions(
                antennas=antenna_joints
            )

        # 等待下一帧（根据录制速度调整）
        time.sleep(0.033)  # ~30 FPS
```

## 示例：将录制保存到文件

```python
import json
from datetime import datetime

# 录制一些数据
reachy_mini.start_recording()

# ... 执行运动 ...

data = reachy_mini.stop_recording()

# 保存到文件
if data:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"recording_{timestamp}.json"

    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)

    print(f"已保存录制到 {filename}")
```

## 示例：从文件加载录制

```python
import json

# 加载录制
with open('recording_20250110_140000.json', 'r') as f:
    data = json.load(f)

print(f"已加载 {len(data)} 帧")

# 回放
import time

for frame in data:
    head_joints = frame.get('head_joints')
    antenna_joints = frame.get('antenna_joints')

    if antenna_joints:
        reachy_mini.set_target_antenna_joint_positions(
            antennas=antenna_joints
        )

    time.sleep(0.033)
```

## 示例：分析录制

```python
import json
import numpy as np

# 加载录制
with open('recording.json', 'r') as f:
    data = json.load(f)

# 提取关节轨迹
head_trajectories = []
antenna_trajectories = []

for frame in data:
    if 'head_joints' in frame:
        head_trajectories.append(frame['head_joints'])
    if 'antenna_joints' in frame:
        antenna_trajectories.append(frame['antenna_joints'])

# 转换为 numpy 数组
head_trajectories = np.array(head_trajectories)
antenna_trajectories = np.array(antenna_trajectories)

# 分析
print(f"录制持续时间: {data[-1]['timestamp']:.2f} 秒")
print(f"平均头部位置: {np.mean(head_trajectories, axis=0)}")
print(f"头部位置范围: {np.ptp(head_trajectories, axis=0)}")

# 查找最大速度
if len(head_trajectories) > 1:
    velocities = np.diff(head_trajectories, axis=0)
    max_velocity = np.max(np.abs(velocities))
    print(f"最大速度: {max_velocity:.4f} rad/帧")
```

## 示例：可视化录制

```python
import json
import matplotlib.pyplot as plt

# 加载录制
with open('recording.json', 'r') as f:
    data = json.load(f)

# 提取数据
timestamps = [frame['timestamp'] for frame in data]
antenna_right = [frame.get('antenna_joints', [0, 0])[0] for frame in data]
antenna_left = [frame.get('antenna_joints', [0, 0])[1] for frame in data]

# 绘图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# 右天线
ax1.plot(timestamps, antenna_right, 'b-', linewidth=2)
ax1.set_ylabel('右天线 (rad)')
ax1.set_title('录制的天线运动')
ax1.grid(True)

# 左天线
ax2.plot(timestamps, antenna_left, 'r-', linewidth=2)
ax2.set_xlabel('时间 (s)')
ax2.set_ylabel('左天线 (rad)')
ax2.grid(True)

plt.tight_layout()
plt.show()
```

## 示例：使用元数据录制

```python
import json
from datetime import datetime

def create_recording_metadata(description, tags=None):
    """为录制创建元数据"""
    return {
        'timestamp': datetime.now().isoformat(),
        'description': description,
        'tags': tags or [],
        'version': '1.0'
    }

# 开始录制
reachy_mini.start_recording()

# ... 执行运动 ...

data = reachy_mini.stop_recording()

# 创建带元数据的录制
recording = {
    'metadata': create_recording_metadata(
        description='点头动作',
        tags=['演示', '点头']
    ),
    'frames': data
}

# 保存
with open('recording_with_metadata.json', 'w') as f:
    json.dump(recording, f, indent=2)
```

## 示例：循环录制

```python
import time

# 循环录制
recordings = []

for i in range(3):
    print(f"正在录制第 {i+1} 次...")

    # 开始录制
    reachy_mini.start_recording()

    # 执行运动
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
        duration=1.0,
        method="MIN_JERK"
    )
    time.sleep(1.1)

    # 停止录制
    data = reachy_mini.stop_recording()
    recordings.append(data)

    # 回到起点
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        duration=0.5
    )
    time.sleep(0.6)

print(f"已录制 {len(recordings)} 次")
```

## 录制技巧

### 1. 保持录制会话专注

```python
# 一次录制一个特定的运动
reachy_mini.start_recording()
# ... 执行单个运动 ...
data = reachy_mini.stop_recording()
```

### 2. 在运动之间添加暂停

```python
# 在开始和结束时添加轻微暂停
time.sleep(0.5)  # 之前暂停
reachy_mini.start_recording()
# ... 运动 ...
data = reachy_mini.stop_recording()
time.sleep(0.5)  # 之后暂停
```

### 3. 使用一致的时间

```python
# 使用固定持续时间的运动
duration = 1.0
reachy_mini.goto_target(head=pose, duration=duration)
time.sleep(duration + 0.1)  # 一致的等待
```

### 4. 检查录制成功

```python
# 验证录制是否成功
data = reachy_mini.stop_recording()

if data is None or len(data) == 0:
    print("警告：没有录制数据！")
else:
    print(f"成功录制了 {len(data)} 帧")
```
