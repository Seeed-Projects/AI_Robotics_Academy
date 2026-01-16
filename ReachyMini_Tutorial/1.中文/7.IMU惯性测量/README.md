# IMU（惯性测量单元）- Reachy Mini SDK

## 概述

Reachy Mini 包含一个 IMU 传感器，提供加速度计、陀螺仪、四元数（方向）和温度数据。这允许机器人感知自身的运动和方向。

## 访问 IMU 数据

### 获取所有 IMU 数据

```python
# 获取 IMU 数据
imu_data = reachy_mini.imu

if imu_data is not None:
    print(f"加速度计: {imu_data['accelerometer']}")
    print(f"陀螺仪: {imu_data['gyroscope']}")
    print(f"四元数: {imu_data['quaternion']}")
    print(f"温度: {imu_data['temperature']} °C")
else:
    print("IMU 数据不可用")
```

## IMU 数据结构

```python
{
    'accelerometer': [x, y, z],  # m/s²
    'gyroscope': [x, y, z],      # rad/s
    'quaternion': [w, x, y, z],  # 方向四元数
    'temperature': float          # °C
}
```

## 加速度计

测量proper加速度（重力 + 运动加速度）。

### 获取加速度计数据

```python
imu_data = reachy_mini.imu

if imu_data:
    ax, ay, az = imu_data['accelerometer']
    print(f"X 加速度: {ax:.2f} m/s²")
    print(f"Y 加速度: {ay:.2f} m/s²")
    print(f"Z 加速度: {az:.2f} m/s²")
```

### 检测运动

```python
def is_moving(imu_data, threshold=0.1):
    """基于加速度计检查机器人是否在移动"""
    if imu_data is None:
        return False

    ax, ay, az = imu_data['accelerometer']

    # 计算排除重力的幅度
    magnitude = (ax**2 + ay**2 + (az - 9.81)**2)**0.5

    return magnitude > threshold

# 使用
imu_data = reachy_mini.imu
if is_moving(imu_data):
    print("机器人正在移动！")
else:
    print("机器人静止")
```

## 陀螺仪

测量角速度（旋转速率）。

### 获取陀螺仪数据

```python
imu_data = reachy_mini.imu

if imu_data:
    gx, gy, gz = imu_data['gyroscope']
    print(f"X 角速度: {gx:.4f} rad/s")
    print(f"Y 角速度: {gy:.4f} rad/s")
    print(f"Z 角速度: {gz:.4f} rad/s")
```

### 计算旋转

```python
import time

def track_rotation(reachy_mini, duration=5.0):
    """随时间跟踪总旋转"""
    start_time = time.time()

    total_rotation = [0.0, 0.0, 0.0]  # [roll, pitch, yaw]

    while time.time() - start_time < duration:
        imu_data = reachy_mini.imu

        if imu_data:
            gx, gy, gz = imu_data['gyroscope']

            # 积分角速度
            dt = 0.01  # 100 Hz
            total_rotation[0] += gx * dt
            total_rotation[1] += gy * dt
            total_rotation[2] += gz * dt

        time.sleep(dt)

    return total_rotation

# 使用
rotation = track_rotation(reachy_mini, duration=2.0)
print(f"总旋转: {rotation}")
```

## 四元数（方向）

表示机器人在 3D 空间中的方向。

### 获取方向

```python
imu_data = reachy_mini.imu

if imu_data:
    w, x, y, z = imu_data['quaternion']
    print(f"四元数: [{w:.4f}, {x:.4f}, {y:.4f}, {z:.4f}]")
```

### 将四元数转换为欧拉角

```python
import math

def quaternion_to_euler(w, x, y, z):
    """将四元数转换为欧拉角（roll, pitch, yaw）"""
    # Roll（x 轴旋转）
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch（y 轴旋转）
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw（z 轴旋转）
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# 使用
imu_data = reachy_mini.imu

if imu_data:
    w, x, y, z = imu_data['quaternion']
    roll, pitch, yaw = quaternion_to_euler(w, x, y, z)

    print(f"Roll: {math.degrees(roll):.2f}°")
    print(f"Pitch: {math.degrees(pitch):.2f}°")
    print(f"Yaw: {math.degrees(yaw):.2f}°")
```

## 温度

测量 IMU 传感器温度。

### 获取温度

```python
imu_data = reachy_mini.imu

if imu_data:
    temp = imu_data['temperature']
    print(f"IMU 温度: {temp:.2f} °C")
```

## 示例：检测轻拍/摇晃

```python
import time

def detect_shake(imu_data, threshold=2.0):
    """检测机器人是否正在被摇晃"""
    if imu_data is None:
        return False

    ax, ay, az = imu_data['accelerometer']

    # 计算幅度
    magnitude = (ax**2 + ay**2 + az**2)**0.5

    return magnitude > threshold

# 监控摇晃事件
print("正在监控摇晃事件... 按 Ctrl+C 停止")

try:
    while True:
        imu_data = reachy_mini.imu

        if detect_shake(imu_data):
            print("检测到摇晃！")
            # 对摇晃做出反应
            reachy_mini.goto_target(
                antennas=[0.8, 0.8],  # 天线抬起
                duration=0.2
            )

        time.sleep(0.1)

except KeyboardInterrupt:
    print("停止监控")
```

## 示例：方向监控

```python
import time
import math

def quaternion_to_euler(w, x, y, z):
    """将四元数转换为欧拉角"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# 监控方向
print("正在监控方向... 按 Ctrl+C 停止")

try:
    while True:
        imu_data = reachy_mini.imu

        if imu_data:
            w, x, y, z = imu_data['quaternion']
            roll, pitch, yaw = quaternion_to_euler(w, x, y, z)

            print(f"\rRoll: {math.degrees(roll):6.2f}° | "
                  f"Pitch: {math.degrees(pitch):6.2f}° | "
                  f"Yaw: {math.degrees(yaw):6.2f}°", end='')

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n停止监控")
```

## 示例：自由落体检测

```python
def detect_freefall(imu_data, threshold=0.5):
    """检测机器人是否处于自由落体状态"""
    if imu_data is None:
        return False

    ax, ay, az = imu_data['accelerometer']

    # 在自由落体中，所有轴应读数接近零
    #（感觉不到重力）
    magnitude = (ax**2 + ay**2 + az**2)**0.5

    return magnitude < threshold

# 监控自由落体
print("正在监控自由落体...")

try:
    while True:
        imu_data = reachy_mini.imu

        if detect_freefall(imu_data):
            print("检测到自由落体！准备着陆！")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("停止监控")
```

## IMU 可用性

注意：IMU 数据可能在所有 Reachy Mini 配置上都不可用。在访问之前始终检查 `imu_data` 是否不为 None。

```python
imu_data = reachy_mini.imu

if imu_data is not None:
    # IMU 可用
    print("IMU 数据可用")
else:
    # IMU 不可用
    print("此配置上 IMU 数据不可用")
```
