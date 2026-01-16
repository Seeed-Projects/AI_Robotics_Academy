# 快速开始 - Reachy Mini SDK

## 简介

Reachy Mini 是一款用于教授机器人学和编程的教育机械臂。该 SDK 提供了一个全面的 Python 接口，用于控制机器人的所有方面。

## 安装

```bash
pip install reachy-mini
```

## 快速入门

### 基本连接

```python
from reachy_mini import ReachyMini

# 连接到机器人
reachy_mini = ReachyMini()

# 机器人现在已准备就绪！
print(f"已连接: {reachy_mini.io_client.is_connected()}")
```

### 连接模式

```python
from reachy_mini import ReachyMini

# 自动模式（先尝试 localhost，然后尝试网络）
reachy_mini = ReachyMini(connection_mode="auto")

# 仅 localhost
reachy_mini = ReachyMini(connection_mode="localhost_only")

# 网络发现
reachy_mini = ReachyMini(connection_mode="network")

# 使用自定义超时
reachy_mini = ReachyMini(timeout=10.0)
```

### 仿真模式

```python
# 使用仿真模式，无需物理机器人
reachy_mini = ReachyMini(use_sim=True)
```

## 构造函数参数

| 参数 | 类型 | 默认值 | 描述 |
|-----------|------|---------|-------------|
| `robot_name` | str | `"reachy_mini"` | 机器人名称 |
| `connection_mode` | str | `"auto"` | 连接策略 |
| `spawn_daemon` | bool | `False` | 生成守护进程 |
| `use_sim` | bool | `False` | 使用仿真模式 |
| `timeout` | float | `5.0` | 连接超时（秒） |
| `automatic_body_yaw` | bool | `True` | 自动身体偏航调整 |
| `log_level` | str | `"INFO"` | 日志级别 |
| `media_backend` | str | `"default"` | 媒体后端选择 |

## 基本操作

### 唤醒和休眠

```python
# 执行唤醒动画
reachy_mini.wake_up()

# 进入休眠位置
reachy_mini.goto_sleep()
```

### 检查连接状态

```python
# 检查是否已连接
is_connected = reachy_mini.io_client.is_connected()

# 获取机器人状态
status = reachy_mini.io_client.get_status()
print(status)
```

### 断开连接

```python
# 正确断开连接
reachy_mini.io_client.disconnect()
```

## 下一步

- [头部控制](./02-head-control.md) - 控制机器人头部
- [天线控制](./03-antenna-control.md) - 控制天线
- [身体控制](./04-body-control.md) - 控制身体运动
- [媒体](./05-media.md) - 摄像头和音频
- [电机控制](./06-motor-control.md) - 直接电机访问
- [IMU](./07-imu.md) - 惯性测量单元
