# 媒体 - Reachy Mini SDK

## 概述

Reachy Mini 的媒体系统提供对相机和音频设备的访问。MediaManager 提供了用于捕获图像、播放声音和录制音频的统一接口。

## 媒体后端

SDK 支持多种后端用于不同用例：

| 后端 | 描述 | 相机 | 音频 |
|---------|-------------|--------|-------|
| `NO_MEDIA` | 无媒体设备 | ❌ | ❌ |
| `DEFAULT` | OpenCV + SoundDevice | ✅ | ✅ |
| `DEFAULT_NO_VIDEO` | 仅 SoundDevice | ❌ | ✅ |
| `GSTREAMER` | GStreamer 后端 | ✅ | ✅ |
| `GSTREAMER_NO_VIDEO` | 仅 GStreamer 音频 | ❌ | ✅ |
| `WEBRTC` | WebRTC 流媒体 | ✅ | ✅ |

### 使用媒体后端初始化

```python
from reachy_mini import ReachyMini

# 默认后端（OpenCV + SoundDevice）
reachy_mini = ReachyMini(media_backend="default")

# 无媒体
reachy_mini = ReachyMini(media_backend="NO_MEDIA")

# 仅音频
reachy_mini = ReachyMini(media_backend="DEFAULT_NO_VIDEO")
```

## 相机

### 捕获帧

```python
# 从相机获取单帧
frame = reachy_mini.media.get_frame()

if frame is not None:
    print(f"帧形状: {frame.shape}")  # (高度, 宽度, 通道)
    print(f"帧数据类型: {frame.dtype}")  # uint8
```

### 连续捕获

```python
import cv2
import time

# 捕获并显示帧
for _ in range(100):  # 捕获 100 帧
    frame = reachy_mini.media.get_frame()

    if frame is not None:
        cv2.imshow('Reachy Mini 相机', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    time.sleep(0.033)  # ~30 FPS

cv2.destroyAllWindows()
```

### 设置相机分辨率

```python
# 设置相机分辨率
reachy_mini.media.camera.set_resolution((640, 480))

# 获取当前分辨率
width, height = reachy_mini.media.camera.get_resolution()
print(f"分辨率: {width}x{height}")
```

### 获取相机规格

```python
# 获取相机规格
specs = reachy_mini.media.camera.get_camera_specs()

if specs:
    print(f"分辨率: {specs.resolution}")
    print(f"FPS: {specs.fps}")
    print(f"视野: {specs.fov}")
```

## 音频

### 播放声音

```python
# 播放声音文件
reachy_mini.media.play_sound("path/to/sound.wav")

# 播放是非阻塞的
import time
time.sleep(2.0)  # 等待声音完成
```

### 停止播放

```python
# 停止当前播放的声音
reachy_mini.media.stop_playing()
```

### 录制音频

```python
# 开始录制
reachy_mini.media.start_recording()

# 录制 3 秒
import time
time.sleep(3.0)

# 停止录制并获取音频数据
audio_data = reachy_mini.media.stop_recording()

if audio_data is not None:
    print(f"音频形状: {audio_data.shape}")
    print(f"音频数据类型: {audio_data.dtype}")  # float32
```

### 获取音频样本

```python
# 开始录制
reachy_mini.media.start_recording()

# 实时获取音频样本
for _ in range(100):  # 获取 100 个样本
    sample = reachy_mini.media.get_audio_sample()

    if sample is not None:
        print(f"样本形状: {sample.shape}")
        # 在这里处理音频样本

    time.sleep(0.01)  # 100 Hz

# 停止录制
reachy_mini.media.stop_recording()
```

### 音频属性

```python
# 获取输入采样率
sample_rate = reachy_mini.media.audio.get_input_audio_samplerate()
print(f"采样率: {sample_rate} Hz")

# 获取输入通道
channels = reachy_mini.media.audio.get_input_channels()
print(f"通道: {channels}")

# 获取输出通道
output_channels = reachy_mini.media.audio.get_output_channels()
print(f"输出通道: {output_channels}")
```

## 声波到达方向（DOA）

如果使用 ReSpeaker 麦克风阵列，您可以获取声源的方向：

```python
# 获取声波到达方向
doa = reachy_mini.media.get_doa()

if doa is not None:
    print(f"声音方向: {doa} 弧度")
    # 使用它让机器人看向声源
    reachy_mini.look_at_world(
        x=0.5, y=0.0, z=0.2,
        duration=0.5
    )
```

## 示例：注视相机中的人脸

```python
import cv2

# 捕获帧
frame = reachy_mini.media.get_frame()

if frame is not None:
    # 检测人脸（使用 Haar 级联或其他检测器）
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    if len(faces) > 0:
        # 获取第一个人脸
        x, y, w, h = faces[0]

        # 计算图像坐标中的人脸中心
        u = x + w // 2
        v = y + h // 2

        # 注视人脸
        reachy_mini.look_at_image(u=u, v=v, duration=0.5)
```

## 示例：声音反应性

```python
import numpy as np

def get_audio_level(audio_sample):
    """从样本计算音频电平"""
    return np.sqrt(np.mean(audio_sample**2))

# 开始录制
reachy_mini.media.start_recording()

threshold = 0.1  # 根据您的环境调整

try:
    while True:
        # 获取音频样本
        sample = reachy_mini.media.get_audio_sample()

        if sample is not None:
            # 计算音频电平
            level = get_audio_level(sample)

            # 对声音做出反应
            if level > threshold:
                # 看向声音方向
                doa = reachy_mini.media.get_doa()
                if doa is not None:
                    # 将 DOA 转换为注视方向
                    reachy_mini.goto_target(
                        body_yaw=doa,
                        duration=0.2
                    )

except KeyboardInterrupt:
    # 停止录制
    reachy_mini.media.stop_recording()
```

## 示例：保存相机帧

```python
import cv2
from datetime import datetime

# 捕获帧
frame = reachy_mini.media.get_frame()

if frame is not None:
    # 生成带时间戳的文件名
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"reachy_mini_{timestamp}.jpg"

    # 保存帧
    cv2.imwrite(filename, frame)
    print(f"已保存帧到 {filename}")
```

## 清理

```python
# 完成后始终关闭媒体
reachy_mini.media.close()

# 断开连接时也会自动调用
reachy_mini.io_client.disconnect()
```

## 媒体管理器属性

```python
# 直接访问相机
camera = reachy_mini.media.camera
frame = camera.read()

# 直接访问音频
audio = reachy_mini.media.audio
audio.play_sound("file.wav")
```
