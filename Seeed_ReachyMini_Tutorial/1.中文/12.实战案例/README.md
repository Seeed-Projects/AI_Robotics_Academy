# 完整示例 - Reachy Mini SDK

## 概述

本节提供完整的、可即运行的示例，展示 Reachy Mini 的各种功能。

## 示例 1：你好 Reachy

```python
#!/usr/bin/env python3
"""
Reachy Mini 的基本 "Hello World" 示例
"""

from reachy_mini import ReachyMini
import time

def main():
    # 连接到机器人
    print("正在连接到 Reachy Mini...")
    reachy_mini = ReachyMini()

    if not reachy_mini.io_client.is_connected():
        print("连接失败！")
        return

    print("连接成功！")

    # 执行唤醒
    print("正在唤醒...")
    reachy_mini.wake_up()
    time.sleep(2.0)

    # 简单运动
    print("正在移动头部...")
    reachy_mini.goto_target(
        head=(0.0, 0.0, 0.1, 0.0, 0.0, 0.0),
        duration=1.0
    )
    time.sleep(1.5)

    # 移动天线
    print("正在移动天线...")
    reachy_mini.goto_target(
        antennas=(0.7, 0.3),
        duration=0.5
    )
    time.sleep(1.0)

    # 进入休眠
    print("正在休眠...")
    reachy_mini.goto_sleep()
    time.sleep(2.0)

    # 断开连接
    print("正在断开连接...")
    reachy_mini.io_client.disconnect()
    print("完成！")

if __name__ == "__main__":
    main()
```

## 示例 2：人脸跟踪

```python
#!/usr/bin/env python3
"""
使用相机跟踪人脸并移动头部跟随
"""

from reachy_mini import ReachyMini
import cv2
import time

class FaceTracker:
    def __init__(self):
        # 连接到机器人
        self.reachy_mini = ReachyMini()

        # 加载人脸检测器
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

        # 跟踪状态
        self.tracking = True
        self.last_look_time = 0
        self.min_look_interval = 0.1  # 两次注视之间间隔 100ms

    def get_face_center(self, frame):
        """获取检测到的人脸中心"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
        )

        if len(faces) > 0:
            # 获取最大的人脸
            face = max(faces, key=lambda f: f[2] * f[3])
            x, y, w, h = face
            return (x + w // 2, y + h // 2), face

        return None, None

    def track_face(self):
        """主跟踪循环"""
        print("开始人脸跟踪... 按 'q' 退出")

        try:
            while self.tracking:
                # 获取相机帧
                frame = self.reachy_mini.media.get_frame()

                if frame is None:
                    continue

                # 查找人脸
                center, face_rect = self.get_face_center(frame)

                if center:
                    # 检查是否经过足够的时间
                    current_time = time.time()
                    if current_time - self.last_look_time >= self.min_look_interval:
                        # 注视人脸
                        u, v = center
                        self.reachy_mini.look_at_image(u=u, v=v, duration=0.2)
                        self.last_look_time = current_time

                    # 绘制矩形
                    if face_rect is not None:
                        x, y, w, h = face_rect
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # 显示帧
                cv2.imshow('人脸跟踪', frame)

                # 检查退出
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        self.tracking = False
        cv2.destroyAllWindows()
        self.reachy_mini.io_client.disconnect()
        print("跟踪已停止")

if __name__ == "__main__":
    tracker = FaceTracker()
    tracker.track_face()
```

## 示例 3：声音反应

```python
#!/usr/bin/env python3
"""
通过转身注视声源来对声音做出反应
"""

from reachy_mini import ReachyMini
import numpy as np
import time

class SoundReactive:
    def __init__(self, threshold=0.1):
        self.reachy_mini = ReachyMini()
        self.threshold = threshold
        self.running = True

    def get_audio_level(self, audio_sample):
        """从样本计算音频电平"""
        if audio_sample is None:
            return 0.0
        return np.sqrt(np.mean(audio_sample**2))

    def react_to_sound(self):
        """主反应循环"""
        print("开始声音反应... 按 Ctrl+C 停止")

        # 开始录制
        self.reachy_mini.media.start_recording()

        try:
            while self.running:
                # 获取音频样本
                sample = self.reachy_mini.media.get_audio_sample()

                if sample is not None:
                    level = self.get_audio_level(sample)

                    # 如果高于阈值则做出反应
                    if level > self.threshold:
                        # 获取声波到达方向
                        doa = self.reachy_mini.media.get_doa()

                        if doa is not None:
                            print(f"检测到声音！电平: {level:.3f}, DOA: {doa:.2f}")

                            # 转向声音
                            self.reachy_mini.goto_target(
                                body_yaw=doa,
                                duration=0.3
                            )

                            # 挺起天线
                            self.reachy_mini.set_target_antenna_joint_positions(
                                antennas=[0.8, 0.8]
                            )

                        time.sleep(0.5)

                        # 恢复天线
                        self.reachy_mini.set_target_antenna_joint_positions(
                            antennas=[0.5, 0.5]
                        )

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n正在停止...")

        finally:
            self.cleanup()

    def cleanup(self):
        """清理资源"""
        self.running = False
        self.reachy_mini.media.stop_recording()
        self.reachy_mini.io_client.disconnect()
        print("声音反应已停止")

if __name__ == "__main__":
    reactor = SoundReactive(threshold=0.15)
    reactor.react_to_sound()
```

## 示例 4：手势序列

```python
#!/usr/bin/env python3
"""
执行一系列表情手势
"""

from reachy_mini import ReachyMini
import time

class GestureController:
    def __init__(self):
        self.reachy_mini = ReachyMini()

    def nod(self, count=3):
        """点头"""
        for _ in range(count):
            self.reachy_mini.goto_target(
                head=(0.0, 0.0, 0.05, 0.0, 0.2, 0.0),
                duration=0.3,
                method="EASE_IN_OUT"
            )
            time.sleep(0.35)

            self.reachy_mini.goto_target(
                head=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                duration=0.3,
                method="EASE_IN_OUT"
            )
            time.sleep(0.35)

    def shake(self, count=3):
        """摇头"""
        for i in range(count):
            direction = 1 if i % 2 == 0 else -1

            self.reachy_mini.goto_target(
                head=(0.0, direction * 0.05, 0.05, 0.0, 0.0, direction * 0.3),
                duration=0.3,
                method="EASE_IN_OUT"
            )
            time.sleep(0.35)

        # 回到中间
        self.reachy_mini.goto_target(
            head=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            duration=0.3
        )
        time.sleep(0.35)

    def look_around(self):
        """四处看看"""
        positions = [
            (0.0, -0.1, 0.1, 0.0, 0.0, -0.4),  # 左
            (0.0, 0.0, 0.15, 0.0, -0.2, 0.0),  # 上
            (0.0, 0.1, 0.1, 0.0, 0.0, 0.4),    # 右
            (0.0, 0.0, 0.05, 0.0, 0.1, 0.0),   # 下
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),    # 中间
        ]

        for pos in positions:
            self.reachy_mini.goto_target(
                head=pos,
                duration=0.8,
                method="MIN_JERK"
            )
            time.sleep(0.9)

    def antenna_wave(self, count=3):
        """摆动天线"""
        for _ in range(count):
            self.reachy_mini.goto_target(
                antennas=[0.8, 0.3],
                duration=0.2
            )
            time.sleep(0.25)

            self.reachy_mini.goto_target(
                antennas=[0.3, 0.8],
                duration=0.2
            )
            time.sleep(0.25)

        # 回到中间
        self.reachy_mini.goto_target(
            antennas=[0.5, 0.5],
            duration=0.3
        )
        time.sleep(0.35)

    def agree(self):
        """同意手势（点头 + 天线开心）"""
        self.reachy_mini.goto_target(
            antennas=[0.7, 0.7],
            duration=0.3
        )
        time.sleep(0.35)
        self.nod(count=2)

    def disagree(self):
        """不同意手势（摇头 + 天线向下）"""
        self.reachy_mini.goto_target(
            antennas=[0.3, 0.3],
            duration=0.3
        )
        time.sleep(0.35)
        self.shake(count=2)

    def curious(self):
        """好奇手势"""
        # 向左看并带着天线
        self.reachy_mini.goto_target(
            head=(0.0, -0.1, 0.15, 0.0, -0.1, -0.4),
            antennas=[0.8, 0.4],
            duration=1.0,
            method="MIN_JERK"
        )
        time.sleep(1.5)

        # 向右看
        self.reachy_mini.goto_target(
            head=(0.0, 0.1, 0.15, 0.0, -0.1, 0.4),
            antennas=[0.4, 0.8],
            duration=1.0,
            method="MIN_JERK"
        )
        time.sleep(1.5)

        # 回到中间
        self.reachy_mini.goto_target(
            head=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            antennas=[0.5, 0.5],
            duration=0.8
        )
        time.sleep(1.0)

    def cleanup(self):
        """清理"""
        self.reachy_mini.goto_sleep()
        time.sleep(2.0)
        self.reachy_mini.io_client.disconnect()

def main():
    controller = GestureController()

    try:
        # 唤醒
        print("正在唤醒...")
        controller.reachy_mini.wake_up()
        time.sleep(2.0)

        # 执行手势
        print("正在点头...")
        controller.nod(count=3)

        print("正在摇头...")
        controller.shake(count=3)

        print("正在四处看看...")
        controller.look_around()

        print("正在摆动天线...")
        controller.antenna_wave(count=3)

        print("表示同意...")
        controller.agree()

        print("表示不同意...")
        controller.disagree()

        print("感到好奇...")
        controller.curious()

        print("正在休眠...")
        controller.cleanup()

    except Exception as e:
        print(f"错误: {e}")
        controller.cleanup()

if __name__ == "__main__":
    main()
```

## 示例 5：录制和回放

```python
#!/usr/bin/env python3
"""
录制运动并回放
"""

from reachy_mini import ReachyMini
import json
import time

def record_movement():
    """录制运动序列"""
    reachy_mini = ReachyMini()

    print("=== 录制模式 ===")
    print("准备录制...")

    # 唤醒
    reachy_mini.wake_up()
    time.sleep(2.0)

    input("按 Enter 开始录制...")
    print("录制已开始！手动移动机器人或使用 goto_target。")
    input("按 Enter 停止录制...")

    # 停止录制
    data = reachy_mini.stop_recording()

    if data:
        print(f"已录制 {len(data)} 帧")

        # 保存到文件
        filename = "recorded_movement.json"
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"已保存到 {filename}")

    # 进入休眠
    reachy_mini.goto_sleep()
    time.sleep(2.0)
    reachy_mini.io_client.disconnect()

def play_back_movement(filename="recorded_movement.json"):
    """回放录制的运动"""
    reachy_mini = ReachyMini()

    print("=== 回放模式 ===")

    # 加载录制
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"录制文件 '{filename}' 未找到！")
        return

    print(f"已加载 {len(data)} 帧")
    print("3 秒后开始回放...")
    time.sleep(3.0)

    # 回放
    start_time = time.time()

    for i, frame in enumerate(data):
        # 计算时间
        frame_time = frame['timestamp']
        elapsed = time.time() - start_time

        if elapsed < frame_time:
            time.sleep(frame_time - elapsed)

        # 提取并设置位置
        antenna_joints = frame.get('antenna_joints')
        if antenna_joints:
            reachy_mini.set_target_antenna_joint_positions(
                antennas=antenna_joints
            )

        # 进度
        print(f"\r帧 {i+1}/{len(data)}", end='')

    print("\n回放完成！")

    # 进入休眠
    time.sleep(1.0)
    reachy_mini.goto_sleep()
    time.sleep(2.0)
    reachy_mini.io_client.disconnect()

def main():
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "play":
        # 回放模式
        filename = sys.argv[2] if len(sys.argv) > 2 else "recorded_movement.json"
        play_back_movement(filename)
    else:
        # 录制模式
        record_movement()

if __name__ == "__main__":
    main()
```

## 运行示例

```bash
# 你好 Reachy
python 01_hello_reachy.py

# 人脸跟踪（需要相机）
python 02_face_tracking.py

# 声音反应（需要麦克风）
python 03_sound_reactive.py

# 手势序列
python 04_gesture_sequence.py

# 录制运动
python 05_record_playback.py

# 回放录制
python 05_record_playback.py play recorded_movement.json
```
