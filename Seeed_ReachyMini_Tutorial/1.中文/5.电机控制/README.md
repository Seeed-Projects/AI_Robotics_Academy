# 电机控制 - Reachy Mini SDK

## 概述

Reachy Mini 使用多个电机来控制其关节。电机控制系统允许您启用、禁用和配置单个电机或电机组。

## 启用/禁用电机

### 启用所有电机

```python
# 启用所有电机
reachy_mini.enable_motors()
```

### 启用特定电机

```python
# 按 ID 启用特定电机
reachy_mini.enable_motors(ids=[1, 2, 3])

# 启用单个电机
reachy_mini.enable_motors(ids=1)
```

### 禁用所有电机

```python
# 禁用所有电机（机器人变松）
reachy_mini.disable_motors()
```

### 禁用特定电机

```python
# 禁用特定电机
reachy_mini.disable_motors(ids=[1, 2, 3])

# 禁用单个电机
reachy_mini.disable_motors(ids=1)
```

## 重力补偿

重力补偿帮助机器人在没有主动控制的情况下保持其位置，使其更安全、更节能。

### 启用重力补偿

```python
# 启用重力补偿
reachy_mini.enable_gravity_compensation()
```

### 禁用重力补偿

```python
# 禁用重力补偿
reachy_mini.disable_gravity_compensation()
```

## 电机 ID

Reachy Mini 有多个电机用于不同的关节：

| 电机 ID | 关节 | 描述 |
|----------|-------|-------------|
| 1 | 身体偏航 | 主体旋转 |
| 2-7 | Stewart 平台 | 头部位置控制（6 DOF） |
| 8 | 右天线 | 右天线角度 |
| 9 | 左天线 | 左天线角度 |

注意：电机 ID 可能因硬件配置而异。请查看机器人的文档以获取确切的 ID。

## 示例：选择性电机控制

```python
# 仅启用头部电机，禁用天线
reachy_mini.enable_motors(ids=[1, 2, 3, 4, 5, 6, 7])
reachy_mini.disable_motors(ids=[8, 9])

# 现在头部可以移动但天线已禁用
```

## 示例：安全电机管理

```python
from contextlib import contextmanager

@contextmanager
def motor_control(reachy_mini, motor_ids):
    """安全电机控制的上下文管理器"""
    try:
        # 启用电机
        reachy_mini.enable_motors(ids=motor_ids)
        yield reachy_mini
    finally:
        # 完成后禁用电机
        reachy_mini.disable_motors(ids=motor_ids)

# 使用
with motor_control(reachy_mini, motor_ids=[1, 2, 3]):
    # 执行运动
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
        duration=1.0
    )
    import time
    time.sleep(1.0)

# 电机自动禁用
```

## 示例：紧急停止

```python
import time

def emergency_stop(reachy_mini):
    """立即停止所有电机控制"""
    # 禁用所有电机
    reachy_mini.disable_motors()
    # 禁用重力补偿
    reachy_mini.disable_gravity_compensation()
    print("紧急停止已激活！")

# 紧急情况下
emergency_stop(reachy_mini)
```

## 示例：电机测试

```python
def test_motor(reachy_mini, motor_id):
    """测试电机是否工作"""
    import time

    print(f"正在测试电机 {motor_id}...")

    # 启用电机
    reachy_mini.enable_motors(ids=motor_id)
    time.sleep(0.5)

    # 禁用电机
    reachy_mini.disable_motors(ids=motor_id)

    print(f"电机 {motor_id} 测试完成")

# 测试所有电机
for motor_id in range(1, 10):
    test_motor(reachy_mini, motor_id)
    time.sleep(1.0)
```

## 最佳实践

### 1. 完成后始终禁用电机

```python
try:
    # 您的代码在这里
    reachy_mini.goto_target(head=target, duration=1.0)
finally:
    # 始终禁用电机
    reachy_mini.disable_motors()
```

### 2. 使用重力补偿保持位置

```python
# 为静止位置启用重力补偿
reachy_mini.enable_gravity_compensation()
reachy_mini.goto_target(head=target, duration=1.0)

# 机器人将更自然地保持位置
```

### 3. 运动前检查电机状态

```python
# 确保在运动前启用电机
reachy_mini.enable_motors()

# 继续运动
reachy_mini.goto_target(head=target, duration=1.0)
```

## 电机配置（高级）

对于高级用户，可以使用工具模块配置电机：

```python
from reachy_mini.tools.setup_motor import setup_motor, change_id

# 配置电机波特率
setup_motor(
    motor_config="path/to/config.yaml",
    serial_port="/dev/ttyUSB0",
    from_baudrate=1000000,
    target_baudrate=1000000
)

# 更改电机 ID
change_id(
    serial_port="/dev/ttyUSB0",
    current_id=1,
    new_id=10,
    baudrate=1000000
)
```

警告：电机配置应仅由经验丰富的用户执行。错误的设置可能会损坏机器人。
