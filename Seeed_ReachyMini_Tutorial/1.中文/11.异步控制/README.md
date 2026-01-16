# 异步操作 - Reachy Mini SDK

## 概述

Reachy Mini 支持异步操作，允许您启动在后台运行的运动，同时您的代码继续执行。这对于：
- 非阻塞运动
- 并行操作
- 复杂序列
- 实时控制

很有用

## 异步运动回放

### 基本异步回放

```python
import asyncio

# 异步播放运动
await reachy_mini.async_play_move(
    move=your_move_object,
    play_frequency=100,  # Hz
    initial_goto_duration=1.0,
    sound=None  # 可选声音文件
)
```

### 带声音的异步

```python
# 播放带声音的运动
await reachy_mini.async_play_move(
    move=your_move_object,
    play_frequency=100,
    initial_goto_duration=1.0,
    sound="path/to/sound.wav"  # 在运动期间播放声音
)
```

## 创建自定义运动

运动通过实现 `Move` 接口的类来定义：

```python
from reachy_mini.motion import Move
import numpy as np

class CustomMove(Move):
    def __init__(self):
        super().__init__()
        self.duration = 2.0  # 运动持续时间（秒）

    def evaluate(self, t):
        """
        在时间 t（0 到 duration）评估运动
        返回: (head_pose, antennas, body_yaw)
        """
        # 在时间 t 计算头部姿态
        if t < 1.0:
            # 前半段：向左看
            head = (0.0, -0.1 * t, 0.1 * t, 0.0, 0.0, -0.3 * t)
        else:
            # 后半段：向右看
            t2 = t - 1.0
            head = (0.0, 0.1 * t2, 0.1 - 0.05 * t2, 0.0, 0.0, 0.3 * t2)

        antennas = (0.5, 0.5)
        body_yaw = 0.0

        return head, antennas, body_yaw

# 使用运动
move = CustomMove()
await reachy_mini.async_play_move(move, play_frequency=100)
```

## 示例：平滑动画运动

```python
import numpy as np
from reachy_mini.motion import Move

class SmoothCircleMove(Move):
    """头部平滑圆周运动"""

    def __init__(self, duration=3.0):
        super().__init__()
        self.duration = duration
        self.radius = 0.05

    def evaluate(self, t):
        # 计算角度（0 到 2π）
        theta = 2 * np.pi * t / self.duration

        # x-y 平面中的圆
        x = self.radius * np.cos(theta)
        y = self.radius * np.sin(theta)
        z = 0.15

        # 轻微倾斜以获得自然外观
        roll = 0.05 * np.sin(2 * theta)
        pitch = 0.05 * np.cos(2 * theta)
        yaw = 0.0

        head = (x, y, z, roll, pitch, yaw)
        antennas = (0.5 + 0.1 * np.sin(theta), 0.5 + 0.1 * np.cos(theta))
        body_yaw = 0.0

        return head, antennas, body_yaw

# 播放运动
move = SmoothCircleMove(duration=3.0)
await reachy_mini.async_play_move(move, play_frequency=100)
```

## 示例：序列运动

```python
import asyncio

async def play_sequence():
    """按顺序播放多个运动"""

    # 定义运动
    moves = [
        NodMove(),
        ShakeMove(),
        LookAroundMove()
    ]

    # 按顺序播放
    for move in moves:
        await reachy_mini.async_play_move(
            move=move,
            play_frequency=100,
            initial_goto_duration=0.5
        )

# 运行序列
await play_sequence()
```

## 示例：并行任务

```python
import asyncio

async def head_movement():
    """控制头部运动"""
    move = HeadScanMove()
    await reachy_mini.async_play_move(move, play_frequency=100)

async def antenna_dance():
    """独立控制天线"""
    while True:
        reachy_mini.set_target_antenna_joint_positions([0.8, 0.2])
        await asyncio.sleep(0.5)
        reachy_mini.set_target_antenna_joint_positions([0.2, 0.8])
        await asyncio.sleep(0.5)

async def parallel_demo():
    """并行运行头部和天线"""
    # 创建任务
    head_task = asyncio.create_task(head_movement())
    antenna_task = asyncio.create_task(antenna_dance())

    # 等待头部完成
    await head_task

    # 取消天线任务
    antenna_task.cancel()

# 运行
await parallel_demo()
```

## 示例：取消

```python
import asyncio

async def play_with_timeout():
    """播放带超时的运动"""

    # 创建任务
    task = asyncio.create_task(
        reachy_mini.async_play_move(move, play_frequency=100)
    )

    try:
        # 最多等待 5 秒
        await asyncio.wait_for(task, timeout=5.0)
    except asyncio.TimeoutError:
        print("运动超时，正在取消...")
        task.cancel()
```

## 示例：实时调整

```python
import asyncio

async def interactive_movement():
    """可以实时调整的运动"""

    class InteractiveMove(Move):
        def __init__(self):
            super().__init__()
            self.duration = 10.0  # 长持续时间
            self.target_z = 0.15  # 可以修改

        def evaluate(self, t):
            # 注视可调整的目标
            return (0.0, 0.0, self.target_z, 0.0, 0.0, 0.0), (0.5, 0.5), 0.0

    move = InteractiveMove()

    # 开始运动
    task = asyncio.create_task(
        reachy_mini.async_play_move(move, play_frequency=50)
    )

    # 在运动期间调整目标
    for i in range(5):
        await asyncio.sleep(1.0)
        move.target_z = 0.15 + i * 0.02  # 抬起头部
        print(f"已将目标调整为 {move.target_z}")

    await task
```

## 同步包装器

对于非异步代码，使用同步包装器：

```python
# 同步回放（阻塞直到完成）
reachy_mini.play_move(
    move=your_move_object,
    play_frequency=100,
    initial_goto_duration=1.0,
    sound=None
)
```

## 最佳实践

### 1. 处理异常

```python
try:
    await reachy_mini.async_play_move(move, play_frequency=100)
except Exception as e:
    print(f"运动失败: {e}")
```

### 2. 清理资源

```python
async def safe_movement():
    try:
        await reachy_mini.async_play_move(move, play_frequency=100)
    finally:
        # 确保电机处于安全状态
        reachy_mini.disable_motors()
```

### 3. 使用适当的频率

```python
# 更高频率 = 更平滑但更多 CPU
# 更低频率 = 更高效但不太平滑

# 对于平滑运动：100 Hz
await reachy_mini.async_play_move(move, play_frequency=100)

# 对于简单运动：50 Hz
await reachy_mini.async_play_move(move, play_frequency=50)
```

### 4. 检查运动状态

```python
# 对于长运动，添加检查点
async def monitored_movement(move):
    task = asyncio.create_task(
        reachy_mini.async_play_move(move, play_frequency=100)
    )

    while not task.done():
        # 检查状态
        print("运动进行中...")
        await asyncio.sleep(1.0)

    await task
    print("运动完成！")
```
