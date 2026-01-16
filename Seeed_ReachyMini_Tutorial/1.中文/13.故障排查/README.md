# 故障排除 - Reachy Mini SDK

## 连接问题

### 找不到机器人

**问题：** 无法连接到 Reachy Mini

```python
reachy_mini = ReachyMini()
# 超时或连接错误
```

**解决方案：**

1. 检查机器人是否已开机
2. 验证网络连接
3. 尝试特定连接模式：

```python
# 仅尝试 localhost
reachy_mini = ReachyMini(connection_mode="localhost_only")

# 或仅尝试网络
reachy_mini = ReachyMini(connection_mode="network")

# 增加超时
reachy_mini = ReachyMini(timeout=10.0)
```

4. 检查守护进程是否正在运行：

```python
# 检查状态
status = reachy_mini.io_client.get_status()
print(status)
```

### 连接中断

**问题：** 机器人随机断开连接

**解决方案：**

1. 使用有线连接而不是 WiFi
2. 检查网络稳定性
3. 实现重连逻辑：

```python
def safe_connect(max_retries=3):
    for i in range(max_retries):
        try:
            reachy_mini = ReachyMini(timeout=10.0)
            if reachy_mini.io_client.is_connected():
                return reachy_mini
        except Exception as e:
            print(f"尝试 {i+1} 失败: {e}")
            time.sleep(2.0)

    raise ConnectionError("重试后连接失败")
```

## 运动问题

### 电机不移动

**问题：** 已发送命令但没有运动

**解决方案：**

1. 检查电机是否已启用：

```python
# 启用电机
reachy_mini.enable_motors()
```

2. 检查错误：

```python
# 获取状态
status = reachy_mini.io_client.get_status()
```

3. 尝试重力补偿：

```python
reachy_mini.enable_gravity_compensation()
```

### 运动不平稳

**问题：** 运动不平滑

**解决方案：**

1. 使用最小加加速度插值：

```python
reachy_mini.goto_target(
    head=target,
    duration=1.0,
    method="MIN_JERK"
)
```

2. 增加持续时间：

```python
# 较慢 = 更平滑
duration = 2.0
```

3. 降低频率（对于异步）：

```python
await reachy_mini.async_play_move(
    move,
    play_frequency=50  # 较低频率
)
```

### 超出范围

**问题：** 目标位置无法到达

**解决方案：**

1. 检查工作空间限制：

```python
# 使用更保守的目标
target = (0.0, 0.0, 0.1, 0.0, 0.0, 0.0)  # 保持在范围内
```

2. 启用自动身体偏航：

```python
reachy_mini.set_automatic_body_yaw(True)
```

3. 测试可达位置：

```python
# 尝试较小的运动
for offset in [0.05, 0.1, 0.15]:
    try:
        reachy_mini.goto_target(
            head=(0.0, 0.0, offset, 0.0, 0.0, 0.0),
            duration=1.0
        )
        print(f"偏移 {offset} 正常")
    except:
        print(f"偏移 {offset} 失败")
```

## 媒体问题

### 相机不工作

**问题：** 相机返回 None 或错误

**解决方案：**

1. 检查媒体后端：

```python
# 尝试不同的后端
reachy_mini = ReachyMini(media_backend="DEFAULT")
```

2. 检查相机是否已连接：

```python
frame = reachy_mini.media.get_frame()
if frame is None:
    print("相机不可用")
```

3. 检查相机规格：

```python
specs = reachy_mini.media.camera.get_camera_specs()
print(specs)
```

### 音频不工作

**问题：** 无法播放或录制声音

**解决方案：**

1. 检查音频后端：

```python
# 仅音频（无相机）
reachy_mini = ReachyMini(media_backend="DEFAULT_NO_VIDEO")
```

2. 测试音频设备：

```python
# 尝试播放
reachy_mini.media.play_sound("test.wav")
```

3. 检查采样率：

```python
sample_rate = reachy_mini.media.audio.get_input_audio_samplerate()
print(f"采样率: {sample_rate}")
```

## IMU 问题

### IMU 数据不可用

**问题：** `reachy_mini.imu` 返回 None

**解决方案：**

1. 检查 IMU 是否已配置：

```python
imu_data = reachy_mini.imu

if imu_data is None:
    print("此配置上 IMU 不可用")
else:
    print("IMU 工作正常！")
```

2. 某些配置可能没有 IMU

## 性能问题

### 响应缓慢

**问题：** 命令执行时间过长

**解决方案：**

1. 减少插值复杂性：

```python
# 使用 LINEAR 而不是 MIN_JERK 以获得更快响应
reachy_mini.goto_target(
    head=target,
    duration=0.3,
    method="LINEAR"
)
```

2. 使用异步操作：

```python
# 非阻塞运动
await reachy_mini.async_play_move(move, play_frequency=100)
```

3. 优化频率：

```python
# 为简单运动使用较低频率
play_frequency = 50  # 而不是 100
```

### CPU 使用率高

**问题： SDK 占用过多 CPU

**解决方案：**

1. 降低轮询频率：

```python
# 不要轮询太快
while True:
    frame = reachy_mini.media.get_frame()
    time.sleep(0.1)  # 10 FPS 而不是 30+
```

2. 正确使用异步：

```python
# 不要创建太多并发任务
# 将后台运动限制为 1-2 个
```

## 调试技巧

### 启用调试日志

```python
# 启用调试日志
reachy_mini = ReachyMini(log_level="DEBUG")
```

### 定期检查状态

```python
# 监控状态
def check_status(reachy_mini):
    status = reachy_mini.io_client.get_status()
    connected = reachy_mini.io_client.is_connected()

    print(f"已连接: {connected}")
    print(f"状态: {status}")

    # 检查 IMU
    imu = reachy_mini.imu
    if imu:
        print(f"IMU 温度: {imu['temperature']}°C")
```

### 测试各个组件

```python
# 测试电机
reachy_mini.enable_motors()
time.sleep(0.5)
reachy_mini.disable_motors()

# 测试运动
reachy_mini.goto_target(
    head=(0.0, 0.0, 0.05, 0.0, 0.0, 0.0),
    duration=0.5
)

# 测试相机
frame = reachy_mini.media.get_frame()
print(f"帧: {frame.shape if frame is not None else 'None'}")

# 测试音频
reachy_mini.media.play_sound("test.wav")
```

## 获取帮助

如果问题仍然存在：

1. 检查 SDK 版本：

```python
import reachy_mini
print(reachy_mini.__version__)
```

2. 检查系统要求：

```bash
python --version  # 应该是 3.8+
pip list | grep reachy
```

3. 启用详细日志并捕获输出

4. 报告问题时请包含：
   - SDK 版本
   - Python 版本
   - 操作系统
   - 错误消息
   - 最小重现代码
