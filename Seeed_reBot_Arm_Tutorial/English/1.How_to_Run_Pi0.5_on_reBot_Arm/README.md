# How to Run Pi0.5 (π0.5) on reBot Arm — Theory, Setup, Training & Deployment

<p align="center">
    <img width="600" src="https://files.seeedstudio.com/wiki/robotics/projects/rebot_arm/rebot_dm.png" alt="reBot Arm B601-DM" />
</p>

<p align="center">
    <a href="https://github.com/Seeed-Projects/reBot-DevArm/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT" /></a>
    <img src="https://img.shields.io/badge/Python-3.12+-blue.svg" alt="Python Version" />
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Ubuntu%20%7C%20Jetson%20Orin-orange.svg" alt="Platform" />
    <img src="https://img.shields.io/badge/Policy-pi05%20%CF%800.5-purple.svg" alt="Policy: Pi0.5" />
</p>

**π0.5 is a Vision-Language-Action (VLA) model with open-world generalization, released by [Physical Intelligence](https://www.physicalintelligence.company/)** ([arXiv:2504.16054](https://arxiv.org/abs/2504.16054)). It is available in [LeRobot](https://github.com/huggingface/lerobot) as the policy type `pi05`, with a pretrained checkpoint `lerobot/pi05_base` you can fine-tune on your own demonstrations.

This tutorial shows you **the complete path from zero to a running π0.5 policy on a reBot Arm**: the theory behind the model, the hardware and shopping links, the software environment, teleoperation and data collection, fine-tuning `pi05` on your own dataset, and finally running the trained policy on the real arm.

> 📚 **Want the full tutorial and more resources?** This GitHub guide is a companion to the complete documentation on the **[Seeed Studio Wiki](https://wiki.seeedstudio.com/robotics_page/)**, which is continuously updated and includes video walkthroughs, hardware assembly, and troubleshooting:
> - **Full wiki tutorials:** [reBot Arm B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/) · [reBot Arm B601-RS with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_rs_lerobot/) · [SO-ARM101 with LeRobot](https://wiki.seeedstudio.com/lerobot_so100m_new/)
> - **π0.5 theory deep-dive:** [π0.5 — Open-World Generalization](https://wiki.seeedstudio.com/robotics/paper_reading_pi05/)
> - **Hardware setup:** [reBot Arm B601-DM Getting Started](https://wiki.seeedstudio.com/rebot_b601_dm_getting_started/) · [reBot Arm B601-RS Getting Started](https://wiki.seeedstudio.com/rebot_b601_rs_getting_started/)

It is part of the [Seeed AI Robotics Academy](../../README.md) — a free, open course series paired with the **100% open-source reBot robotic arm** (commercial use and reproduction allowed). If this guide helps you, please ⭐ star [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm/) and [AI_Robotics_Academy](https://github.com/Seeed-Projects/AI_Robotics_Academy).

> ✅ **Verified hardware:** everything below runs on the **reBot Arm B601-DM**, **reBot Arm B601-RS**, and **SO-ARM101 (SO-101)** leader-follower kits. The same workflow applies to any LeRobot-compatible arm.

---

## Table of Contents

- [Which Robotic Arms Are Compatible with Pi0.5?](#which-robotic-arms-are-compatible-with-pi05)
- [Pi0.5 Theory Deep-Dive](#pi05-theory-deep-dive)
- [Hardware You Need (with Shopping Links)](#hardware-you-need-with-shopping-links)
- [No-Code Alternative: SenseCraft Robotics (Windows / macOS / Linux)](#no-code-alternative-sensecraft-robotics-windows--macos--linux)
- [Environment Setup](#environment-setup)
- [Step-by-Step Practice](#step-by-step-practice)
  - [Step 1: Calibrate the Robotic Arm](#step-1-calibrate-the-robotic-arm)
  - [Step 2: Teleoperate the Arm](#step-2-teleoperate-the-arm)
  - [Step 3: Add Cameras](#step-3-add-cameras)
  - [Step 4: Record a Dataset](#step-4-record-a-dataset)
  - [Step 5: Train Pi0.5](#step-5-train-pi05)
  - [Step 6: Evaluate on the Real Arm](#step-6-evaluate-on-the-real-arm)
- [FAQ](#faq)
- [References](#references)

---

## Which Robotic Arms Are Compatible with Pi0.5?

**Short answer: any robotic arm with a [LeRobot](https://github.com/huggingface/lerobot) robot class can run a fine-tuned Pi0.5 (π0.5) policy**, because π0.5 is implemented in LeRobot as the policy type `pi05` and adapts automatically to your robot's motor states, motor actions, and number of cameras — this information is stored in the LeRobot dataset you record.

The arms below are **verified to run π0.5 fine-tuning end-to-end** with Seeed's open hardware:

| Robotic arm | Actuation | Interface | Status in this tutorial | Buy |
| :--- | :--- | :--- | :--- | :--- |
| **reBot Arm B601-DM** (Seeed) | Damiao DM brushless servos | USB-CAN bridge (`/dev/ttyACM0`) | ✅ Verified, main example in this guide | [reBot Arm B601-DM Bundle](https://www.seeedstudio.com/reBot-Arm-B601-DM-Bundle.html) |
| **reBot Arm B601-RS** (Seeed) | RobStride RS actuators | SocketCAN (`can0`) / PCAN | ✅ Verified (commands included) | [reBot Arm B601-RS Kit](https://www.seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html) |
| **SO-ARM101 / SO-101** (Seeed × LeRobot) | Feetech serial servos | USB serial (`/dev/ttyACM0`) | ✅ Verified (commands included) | [SO-101 Low-Cost AI Arm Kit Pro](https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html) |

Also supported through the official LeRobot ecosystem (not covered step-by-step here):

- **SO-ARM100 / SO-100** — the official SO arm series from [TheRobotStudio](https://github.com/TheRobotStudio/SO-ARM100), fine-tuning workflows in the [LeRobot docs](https://huggingface.co/docs/lerobot/so101).
- **Koch v1.1 / ALOHA-style bimanual arms** — supported by LeRobot; the original π0 / π0.5 research platforms were bimanual mobile manipulators (two 6-DoF arms + grippers, a mobile base, and a lift — 18/19-dimensional action spaces).

> 💡 **Why this works:** π0.5's action expert outputs an action chunk (a sequence of future control points) that is mapped to your robot's joints and gripper through the LeRobot dataset format. Record demonstrations with any LeRobot-compatible arm, fine-tune `lerobot/pi05_base`, and the policy automatically matches your robot's state/action dimensions and camera count.

The rest of this guide uses the **reBot Arm B601-DM** as the running example, with command variants for **B601-RS** and **SO-ARM101** wherever they differ.

---

## Pi0.5 Theory Deep-Dive

*Condensed from the Seeed paper reading course: [π0.5 — Open-World Generalization](https://wiki.seeedstudio.com/robotics/paper_reading_pi05/). Paper: [π0.5: a Vision-Language-Action Model with Open-World Generalization](https://arxiv.org/abs/2504.16054).*

### What π0.5 Is

π0.5 is a **Vision-Language-Action model** built on π0 and improved for **adaptation to unfamiliar environments**: it is trained jointly (co-trained) on heterogeneous data — demonstrations from multiple robots, subtask annotations, and web image-text data — so that it can tidy and clean in homes it has never seen.

A VLA receives images, task text, and robot state, and outputs control actions. For "put the plate in the sink": images locate the plate, state describes the arm, the instruction defines the goal, and actions control grasping and placement.

### Hierarchical Actions: Select the Subtask First

Mapping "clean the kitchen" directly to joint motions mixes *deciding what to do* with *deciding how to move*. π0.5 separates the two inside one model:

```text
Current images + state + overall task  →  Subtask s        (e.g. "pick up the plate")
Current images + state + subtask s    →  Action chunk A   (e.g. 50 future control points)
```

```text
Clean the kitchen → Pick up the plate → Grasping action chunk
Observe again → Put the plate in the sink → Transport action chunk
```

Each prediction covers **50 future control points** — an *action chunk*. The high-level subtask updates slowly; the low level generates chunks conditioned on the subtask; the controller executes the points sequentially. If the high-level choice is wrong, accurate motions still perform the wrong task — which is why the model trains both levels.

### Co-Training on Heterogeneous Data

The training recipe mixes data sources that each teach something different:

| Data source | Learning opportunity |
| :--- | :--- |
| Household mobile-robot data (~400 h, ~100 homes) | Combining movement, lifting, and bimanual manipulation in real homes |
| Stationary arms across environments | More homes, objects, and manipulation scenes |
| Cross-robot laboratory data | More skills and tasks |
| Subtask annotations | Choosing the next step and locating relevant objects |
| Web image-text data | Recognition, scene understanding, and language capabilities |
| Expert language demonstrations | Directing an existing low-level policy through multi-step tasks |

In stage 1, **97.6% of training samples come from outside** the target household mobile-manipulation data — transfer between knowledge sources is what drives generalization.

### Encoded Actions for Training, Continuous Actions for Execution

- **Pre-training:** [FAST](https://arxiv.org/abs/24104959) compresses continuous actions into discrete tokens learned alongside text via next-token prediction.
- **Post-training:** an action expert (~300M parameters) trained with **flow matching** is added, while text and discrete-action prediction are retained.
- **Execution:** generate the subtask text autoregressively, then apply ten flow updates from random noise to obtain a continuous 50-step action chunk.

The training objective is: **total loss = token-prediction loss + 10 × continuous-action loss** (the continuous term is active only in post-training).

### π0 vs π0.5 at a Glance

| Aspect | π0 | π0.5 |
| :--- | :--- | :--- |
| Focus | General manipulation across robots and tasks | Multi-stage manipulation in new homes, new objects and layouts |
| Data | VLM init + mainly cross-robot demonstrations | Co-training on actions, subtasks, localization, web data; expert language demonstrations |
| Action training | Continuous expert in pre- and post-training | FAST tokens during pre-training; expert added in post-training |
| Long-task organization | Person or external high-level VLM supplies intermediate instructions | The same model selects subtasks and generates actions |
| State input | Continuous state projected into the action expert | State discretized as tokens in the VLM |

Both use PaliGemma, a ~300M-parameter action expert, 50 control points, and ten flow updates. π0.5's gains come from the training recipe and task organization — not a bigger model.

> 📖 Want the full derivation — network shapes, flow-matching math, experiment boundaries, and where the paper's claims hold? Read the complete deep-dive: [π0.5 — Open-World Generalization (Seeed Paper Reading)](https://wiki.seeedstudio.com/robotics/paper_reading_pi05/).

---

## Hardware You Need (with Shopping Links)

A complete leader-follower setup for π0.5 fine-tuning:

| Item | Purpose | Link |
| :--- | :--- | :--- |
| **reBot Arm B601-DM Bundle** (follower + reBot Arm 102 leader) | The verified main platform of this guide | [Buy now](https://www.seeedstudio.com/reBot-Arm-B601-DM-Bundle.html) |
| **reBot Arm B601-RS** (assembled kit with gripper) | RS-actuator variant, verified | [Buy now](https://www.seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html) |
| **reBot Arm B601-RS Disassembly Kit + Power Bundle** | Self-assembly variant | [Buy now](https://www.seeedstudio.com/reBot-Arm-B601-RS-Disassembly-Kit-Version-with-Power-Supply-Bundle.html) |
| **SO-101 Low-Cost AI Arm Kit Pro** (leader + follower) | Budget-friendly verified platform | [Buy now](https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html) |
| **Power Adapter Kit for B601-DM** | Stable power for the DM follower | [Buy now](https://www.seeedstudio.com/Power-Adapter-Kit-for-reBot-Arm-B601-DM-p-6874.html) |
| **Power Adapter Kit for B601-RS** | Stable power for the RS follower | [Buy now](https://www.seeedstudio.com/Power-Adapter-Kit-for-reBot-Arm-B601-RS-p-6873.html) |
| **Orbbec Gemini 2 3D camera** | RGB-D perception (optional but recommended) | [Buy now](https://www.seeedstudio.com/Orbbec-Gemini-2-3D-Camera-p-6464.html) |
| **Intel RealSense D435i** | RGB-D camera alternative | [Buy now](https://www.seeedstudio.com/Intel-RealSense-Depth-Camera-D435i-p-4423.html) |
| **Intel RealSense D405** | Short-range stereo depth camera (7–50 cm, tabletop) | [Wiki guide](https://wiki.seeedstudio.com/realsense_d405/) |
| **Robot Arm Camera Mount for SO-10x / reBot B601** | Fixed camera positions for clean data | [Buy now](https://www.seeedstudio.com/Robot-Arm-Camera-Mount-for-LeRobot-SO-10&reBot-B601-p-7003.html) |
| **2× generic USB cameras (640×480, MJPG)** | Minimum two camera views for training | Use any USB webcam; the camera mount above holds them |

**Compute for training:** an NVIDIA GPU machine (e.g. RTX 3060 8GB trains ACT-scale policies; π0.5 fine-tuning is comfortable on RTX 4090 / A100 with `bfloat16`), or a Jetson Orin for edge training/inference. GPU-less? Use cloud training via the [LeRobot pi0.5 docs](https://huggingface.co/docs/lerobot/pi05) or the no-code option below.

---

## No-Code Alternative: SenseCraft Robotics (Windows / macOS / Linux)

If you do not want to configure a Python environment at all, **[SenseCraft Robotics](https://sensecraft.seeed.cc/en?utm_source=rebot_wiki&utm_medium=wiki&utm_campaign=sensecraft_banner)** is Seeed's no-code platform for exactly these arms — **SO-ARM101, reBot Arm 102 + B601-RS and B601-DM**. Connect the arms, calibrate, collect demonstrations, train and run inference from a guided desktop client — no Python, no terminal:

- ⚡ 6-step guided workflow (device setup → data collection → training → run)
- ☁️ Cloud training — no local GPU required
- 🖥️ Windows / macOS / Linux client (serial-permission notes per OS are covered in the [FAQ](https://wiki.seeedstudio.com/sensecraft_robotics_common_issues/))
- 🤖 SO-ARM101 · B601-RS · B601-DM all supported

The full no-code workflow — installation, account setup, device binding, calibration, data collection, cloud training, and deployment — is available in the companion guide in this repository: **[SenseCraft Robotics No-Code Guide](../2.SenseCraft_Robotics_No_Code_Guide/README.md)**, ported from the [official wiki tutorial](https://wiki.seeedstudio.com/sensecraft_robotics/).

The rest of this tutorial covers the open-source **LeRobot + Pi0.5** path.

---

## Environment Setup

**Supported platforms:**

| Platform | Requirements |
| :--- | :--- |
| **Ubuntu x86** | Ubuntu 22.04 / 24.04 · NVIDIA GPU with CUDA 12+ (optional but recommended) · Python 3.12 · Torch 2.6 |
| **Jetson Orin** | JetPack 6.0 or 6.1 (6.2 **not** supported) · Python 3.12 · Torch 2.3+ |

**Step 1 — Install Miniforge**

```bash
cd ~
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh

~/miniforge3/bin/conda init bash
source ~/.bashrc
```

**Step 2 — Clone the LeRobot repository**

Use Seeed's verified fork — it ships the reBot arm support packages and is tested with this tutorial:

```bash
mkdir ~/rebot_lerobot
cd ~/rebot_lerobot
git clone https://github.com/Seeed-Projects/lerobot.git
```

**Step 3 — Create the environment and install LeRobot**

```bash
cd ~/rebot_lerobot

# Create conda environment (Python 3.12)
conda create -y -n lerobot python=3.12
conda activate lerobot

# Install lerobot main project (editable mode)
pip install -e ./lerobot

# Add the reBot arm support packages
pip install lerobot-teleoperator-rebot-arm-102
pip install lerobot-robot-seeed-b601
pip install motorbridge
```

Package details: [lerobot-teleoperator-rebot-arm-102](https://github.com/Seeed-Projects/lerobot-teleoperator-rebot-arm-102) · [lerobot-robot-seeed-b601](https://github.com/Seeed-Projects/lerobot-robot-seeed-b601)

**Step 4 — Install the Pi0.5 extra and video dependencies**

```bash
# Pi0 / Pi0.5 dependencies (the pi extra)
cd ~/rebot_lerobot/lerobot
pip install -e ".[pi]"
cd ~

# ffmpeg for video encoding
conda install ffmpeg -c conda-forge
```

> **Tip:** if you hit version issues, pin `ffmpeg`:
> ```bash
> conda install ffmpeg=7.1.1 -c conda-forge
> ```
> Check the `libsvtav1` encoder with `ffmpeg -encoders | grep svtav1`.

> **Jetson Orin (JetPack 6.0+):** install GPU PyTorch/Torchvision per [this guide](https://github.com/Seeed-Projects/reComputer-Jetson-for-Beginners/tree/main/3-Basic-Tools-and-Getting-Started/3.5-Pytorch), then:
> ```bash
> conda install -y -c conda-forge "opencv>=4.10.0.84"
> conda remove opencv
> pip3 install opencv-python==4.10.0.84
> conda uninstall numpy
> pip3 install numpy==1.26.0
> ```

**Step 5 — Check PyTorch**

```bash
python3
```

```python
import torch
print(torch.cuda.is_available())  # True when GPU acceleration is active
exit()
```

`False` means the CPU build — fine for basic workflows, but training will be slow. Install a CUDA-matched PyTorch from [pytorch.org](https://pytorch.org/index.html) if you have an NVIDIA GPU. **RTX 50-series** needs the preview build with CUDA 12.8+:

```bash
pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu128
```

📹 Video walkthrough: [Install LeRobot on reBot Arm](https://www.youtube.com/watch?v=mWrWeqAPDSY)

---

## Step-by-Step Practice

### Step 0: Safety First

> ⚠️ **Danger:** during teleoperation, if the leader-follower arm pair loses power, has poor power contact, or a signal line detaches, **stop the program and return the arm to its home zero position before reconnecting power and restarting**. Otherwise disordered data can cause the arm to run away — a real safety hazard.

Grant serial permissions before every session:

```bash
sudo chmod 666 /dev/ttyUSB*  # Leader arm
sudo chmod 666 /dev/ttyACM*  # Follower arm (serial bridge)
```

### Step 1: Calibrate the Robotic Arm

Calibration ensures leader and follower report the same values at the same physical position — it is what lets a policy trained on one arm run on another. Calibration data is stored in `~/.cache/huggingface/lerobot/calibration/`; delete the files there to recalibrate.

**reBot Arm B601-DM — follower** (zero position = gripper fully closed, see image):

```bash
lerobot-calibrate \
    --robot.type=seeed_b601_dm_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=follower1 \
    --robot.can_adapter=damiao
```

**reBot Arm B601-RS — follower** (bring up SocketCAN first):

```bash
sudo ip link set can0 up

lerobot-calibrate \
    --robot.type=seeed_b601_rs_follower \
    --robot.port=can0 \
    --robot.id=follower1 \
    --robot.can_adapter=socketcan
```

**SO-ARM101 — follower** (move all joints to mid-range, then sweep each joint through its full range when prompted):

```bash
lerobot-calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=my_awesome_follower_arm
```

**Leader arm** (all three platforms use the reBot Arm 102 leader; SO-ARM kits use an SO101 leader):

```bash
# reBot Arm 102 leader (B601-DM and B601-RS kits)
lerobot-calibrate \
    --teleop.type=rebot_arm_102_leader \
    --teleop.port=/dev/ttyUSB0 \
    --teleop.id=rebot_arm_102_leader

# SO-ARM101 leader
lerobot-calibrate \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=my_awesome_leader_arm
```

> **Note:** if `/dev/ttyACM0` cannot be found on first connection, `brltty` is occupying the serial port:
> ```bash
> sudo dmesg | grep ttyUSB   # last line shows "disconnected"
> sudo apt remove brltty
> ```

📹 Video walkthrough: [Calibrate the reBot Arm](https://www.youtube.com/watch?v=v8Ek1Ad1VWo)

### Step 2: Teleoperate the Arm

Sanity-check the leader-follower pairing before recording data.

**B601-DM:**

```bash
lerobot-teleoperate \
    --robot.type=seeed_b601_dm_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=follower1 \
    --robot.can_adapter=damiao \
    --teleop.type=rebot_arm_102_leader \
    --teleop.port=/dev/ttyUSB0 \
    --teleop.id=rebot_arm_102_leader
```

**B601-RS:**

```bash
lerobot-teleoperate \
    --robot.type=seeed_b601_rs_follower \
    --robot.port=can0 \
    --robot.id=follower1 \
    --robot.can_adapter=socketcan \
    --teleop.type=rebot_arm_102_leader \
    --teleop.port=/dev/ttyUSB0 \
    --teleop.id=rebot_arm_102_leader
```

**SO-ARM101:**

```bash
lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=my_awesome_follower_arm \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=my_awesome_leader_arm
```

📹 Video walkthrough: [Teleoperate the reBot Arm](https://www.youtube.com/watch?v=6_hXd_QXO2A)

### Step 3: Add Cameras

Two camera views (e.g. `front` + `side`) are the practical minimum for π0.5 fine-tuning. Find your camera indices:

```bash
lerobot-find-cameras opencv
```

Test the cameras inside teleoperation (B601-DM example; swap robot/teleop blocks for RS / SO-ARM as in Step 2):

```bash
lerobot-teleoperate \
    --robot.type=seeed_b601_dm_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=follower1 \
    --robot.can_adapter=damiao \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: "MJPG"}, side: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30, fourcc: "MJPG"}}" \
    --teleop.type=rebot_arm_102_leader \
    --teleop.port=/dev/ttyUSB0 \
    --teleop.id=rebot_arm_102_leader \
    --display_data=true
```

Recommended settings and why:

- **640×480** — balances detail and real-time bandwidth; the model resizes inputs anyway.
- **30 fps** — matches the recording rate; slower cameras cause repeated frames.
- **`fourcc: "MJPG"`** — compressed transfer; supports 3 cameras at 1080p/30fps. `YUYV` forces lower resolution/fps and can stutter.

**Depth cameras (optional):** RealSense D435i / D405 and Orbbec Gemini 2 are supported on the `DepthCameraSupport` branch of the Seeed LeRobot fork:

```bash
git checkout DepthCameraSupport
git pull origin DepthCameraSupport
pip install -e ".[realsense]"   # or ".[orbbec]"
sudo chmod a+rw /dev/bus/usb/*/*
lerobot-find-cameras realsense  # or: lerobot-find-cameras orbbec
```

Then pass the camera blocks (e.g. `realsense_d435i_color`, `orbbec_color` with `serial_number_or_name`) in `--robot.cameras`. Full examples: [reBot Arm wiki — Add Cameras](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/#add-cameras).

> 📌 **Critical:** the camera keys (`front`, `side`, …) used at evaluation **must exactly match** those used during data collection, or training/evaluation will fail with `mean is infinity` errors.

### Step 4: Record a Dataset

**B601-DM — record 5 test episodes locally:**

```bash
lerobot-record \
    --robot.type=seeed_b601_dm_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=follower1 \
    --robot.can_adapter=damiao \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: "MJPG"}, side: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30, fourcc: "MJPG"}}" \
    --teleop.type=rebot_arm_102_leader \
    --teleop.port=/dev/ttyUSB0 \
    --teleop.id=rebot_arm_102_leader \
    --display_data=true \
    --dataset.repo_id=seeed_rebot_b601_dm/test \
    --dataset.num_episodes=5 \
    --dataset.single_task="Grab the black cube" \
    --dataset.push_to_hub=false \
    --dataset.episode_time_s=30 \
    --dataset.reset_time_s=30
```

**B601-RS —** same command with `--robot.type=seeed_b601_rs_follower`, `--robot.port=can0`, `--robot.can_adapter=socketcan`, and e.g. `--dataset.repo_id=seeed_rebot_b601_rs/test`.

**SO-ARM101 —** same command with `--robot.type=so101_follower`, `--robot.id=my_awesome_follower_arm`, `--teleop.type=so101_leader`, `--teleop.port=/dev/ttyACM1`, `--teleop.id=my_awesome_leader_arm`, and e.g. `--dataset.repo_id=seeed_soarm101/test`.

The dataset lands in `~/.cache/huggingface/lerobot/<repo_id>`. Control recording with the keyboard:

| Key | Action |
| :--- | :--- |
| → (Right arrow) | End the current episode early; move to reset/next |
| ← (Left arrow) | Discard the current episode and re-record it |
| ESC | End the session, encode videos, save the dataset |

**Scale up for π0.5.** A VLA fine-tune needs more than 5 episodes:

- Record **≥50 episodes** (e.g. 5 object positions × 10 episodes each) — with π0.5's pretraining doing the heavy lifting, 50–100 good episodes go a long way.
- Keep cameras fixed and grasping behavior consistent; keep the object visible in both views.
- Start with reliable grasping, then add variation (new positions, orientations) gradually.
- Rule of thumb: **you should be able to do the task yourself looking only at the camera images on screen.**

**Upload to the Hugging Face Hub (optional):**

```bash
huggingface-cli login --token ${HUGGINGFACE_TOKEN} --add-to-git-credential
HF_USER=$(huggingface-cli whoami | head -n 1)
```

Re-run the record command with `--dataset.repo_id=${HF_USER}/grab_cube_v1` and `--dataset.push_to_hub=true`. Visualize either way:

```bash
lerobot-dataset-viz \
  --repo-id seeed_rebot_b601_dm/test \
  --episode-index 0 \
  --display-compressed-images=false
```

> 📚 What makes a *good* dataset (synchronization, validation, quality): see the [Seeed paper-reading course — Data Quality](https://wiki.seeedstudio.com/robotics/paper_reading_hardware_and_teleoperation/) and the [LeRobot datasets blog](https://huggingface.co/blog/lerobot-datasets#what-makes-a-good-dataset).

### Step 5: Train Pi0.5

Fine-tune the pretrained `lerobot/pi05_base` on your dataset:

```bash
lerobot-train \
  --dataset.repo_id=seeed_rebot_b601_dm/test \
  --policy.type=pi05 \
  --output_dir=outputs/pi05_training \
  --job_name=pi05_training \
  --policy.repo_id=${HF_USER}/my_pi05_policy \
  --policy.pretrained_path=lerobot/pi05_base \
  --policy.compile_model=true \
  --policy.gradient_checkpointing=true \
  --policy.dtype=bfloat16 \
  --policy.freeze_vision_encoder=false \
  --policy.train_expert_only=false \
  --steps=3000 \
  --policy.device=cuda \
  --batch_size=32 \
  --wandb.enable=false \
  --policy.push_to_hub=false
```

(Use `${HF_USER}/my_dataset` as `--dataset.repo_id` for Hub datasets; drop `--policy.push_to_hub=false` and keep `--policy.repo_id` to publish the fine-tuned model.)

**The parameters that matter for π0.5:**

| Parameter | Meaning |
| :--- | :--- |
| `--policy.type=pi05` | Selects the π0.5 policy in LeRobot |
| `--policy.pretrained_path=lerobot/pi05_base` | Pretrained base; `lerobot/pi05_libero` is the LIBERO-oriented variant |
| `--policy.dtype=bfloat16` | Mixed precision — faster and lighter when supported |
| `--policy.gradient_checkpointing=true` | Big VRAM savings; enable when memory is tight |
| `--policy.compile_model=true` | Compilation optimization (first compile is slower, then faster) |
| `--policy.train_expert_only=true` | VRAM-saving mode: freeze the VLM, train only the action expert + projections — good first step on small GPUs |
| `--policy.freeze_vision_encoder=false` | Unfreeze the vision encoder for fuller adaptation |
| `--steps=3000` | Fine-tuning steps; scale with dataset size and task difficulty |
| `--batch_size=32` | Reduce first if you run out of memory, then grow |

**VRAM notes:** if OOM, lower `--batch_size` first, then enable `train_expert_only`, and keep `gradient_checkpointing=true`. If your dataset's normalization statistics mismatch, force a scheme with `--policy.normalization_mapping=...`.

**Resume from a checkpoint:**

```bash
lerobot-train \
  --config_path=outputs/pi05_training/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

**RTX 50-series:** add `--dataset.video_backend=pyav` to bypass missing APIs in the preview torchvision.

Official reference: [LeRobot Pi0.5 documentation](https://huggingface.co/docs/lerobot/pi05).

### Step 6: Evaluate on the Real Arm

Load the trained checkpoint and record evaluation episodes — the arm now executes the task autonomously:

```bash
lerobot-record \
  --robot.type=seeed_b601_dm_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.can_adapter=damiao \
  --robot.id=follower1 \
  --robot.cameras='{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: "MJPG"}, side: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30, fourcc: "MJPG"} }' \
  --display_data=false \
  --dataset.repo_id=${HF_USER}/eval_my_pi05_test \
  --dataset.single_task="Put lego brick into the transparent box" \
  --dataset.episode_time_s=50 \
  --dataset.num_episodes=10 \
  --policy.path=outputs/pi05_training/checkpoints/last/pretrained_model
```

Checklist:

- `--policy.path` points to your weights (local path as above, or `${HF_USER}/my_pi05_policy` if uploaded to the Hub).
- `--dataset.single_task` should match the description used at collection.
- Camera keys and serial setup must match the collection session exactly.
- If you hit `File exists: .../eval_xxxx`, delete the old `eval_` folder and re-run.
- Name eval datasets with an `eval_` prefix so they stay separate from training data.

**Optional — smoother control with async inference:** π0.5 is a chunked policy, so [async inference](https://huggingface.co/blog/async-robot-inference) (server computes the next chunk while the arm executes the current one) removes the move-pause-move pattern. Install with `pip install -e ".[async]"`, start a policy server, then connect a robot client:

```bash
# Terminal 1 (server)
python -m lerobot.async_inference.policy_server \
  --model_path=outputs/pi05_training/checkpoints/last/pretrained_model \
  --server_address=127.0.0.1:8080

# Terminal 2 (client)
python -m lerobot.async_inference.robot_client \
  --robot.type=seeed_b601_dm_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.can_adapter=damiao \
  --robot.id=follower1 \
  --robot.cameras='{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: "MJPG"} }' \
  --server_address=127.0.0.1:8080 \
  --actions_per_chunk=50 \
  --chunk_size_threshold=0.5 \
  --fixed_update_fps=30
```

Tune `--actions_per_chunk` up if motion is jerky, down if the response lags.

---

## FAQ

**`Could not connect on port /dev/ttyUSB0 or /dev/ttyACM0`** — you forgot serial permissions: `sudo chmod 666 /dev/ttyUSB* /dev/ttyACM*`.

**`No valid stream found in input file`** — install ffmpeg 7.1.1: `conda install ffmpeg=7.1.1 -c conda-forge`.

**How long does training take?** Fine-tuning π0.5 for 3000 steps takes a few hours on an RTX 4090 / A100; smaller GPUs work with `train_expert_only=true` + reduced `batch_size`, or use cloud training / [SenseCraft Robotics](#no-code-alternative-sensecraft-robotics-windows--macos--linux).

**The arm fails to grasp after training** — check data quality first: camera positions, angles and lighting must stay stable during collection; reduce background changes and pedestrians in frame; verify the object was visible in both cameras; and record enough episodes (50+).

**Keyboard shortcuts don't respond during recording** — downgrade pynput: `pip install pynput==1.6.8` (and verify `$DISPLAY` is set).

**Camera image errors** — do not connect USB cameras through a hub; plug them directly into the machine.

**Which repository should I clone?** Use `https://github.com/Seeed-Projects/lerobot.git` (the verified fork this tutorial targets). The upstream `huggingface/lerobot` moves fast and may differ in dataset versions and commands.

**Software or environment issues you can't resolve?** Check the [LeRobot GitHub](https://github.com/huggingface/lerobot) issues, join the [LeRobot Discord](https://discord.gg/8TnwDdjFGU), or the [Seeed discussion](https://github.com/Seeed-Studio/wiki-documents/discussions/69).

---

## References

- π0.5 paper: [π0.5: a Vision-Language-Action Model with Open-World Generalization](https://arxiv.org/abs/2504.16054) (Physical Intelligence)
- LeRobot: [repository](https://github.com/huggingface/lerobot) · [Pi0.5 docs](https://huggingface.co/docs/lerobot/pi05) · [pretrained pi05 models](https://huggingface.co/lerobot)
- Seeed paper-reading deep-dive: [π0.5 — Open-World Generalization](https://wiki.seeedstudio.com/robotics/paper_reading_pi05/)
- Source wiki tutorials: [reBot Arm B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/) · [reBot Arm B601-RS with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_rs_lerobot/) · [SO-ARM101 in LeRobot](https://wiki.seeedstudio.com/lerobot_so100m/)
- Hardware: [reBot-DevArm (open-source arm)](https://github.com/Seeed-Projects/reBot-DevArm/) · [lerobot-robot-seeed-b601](https://github.com/Seeed-Projects/lerobot-robot-seeed-b601) · [lerobot-teleoperator-rebot-arm-102](https://github.com/Seeed-Projects/lerobot-teleoperator-rebot-arm-102)
- No-code platform: [SenseCraft Robotics](https://sensecraft.seeed.cc/en) · [wiki guide](https://wiki.seeedstudio.com/sensecraft_robotics/)

---

## Tech Support & Product Discussion

Thank you for choosing our products! We offer several channels to support your experience:

<div align="center">
  <a href="https://forum.seeedstudio.com/" target="_blank"><strong>Seeed Forum</strong></a> ·
  <a href="https://www.seeedstudio.com/contacts" target="_blank"><strong>Contact Us</strong></a> ·
  <a href="https://discord.gg/eWkprNDMU7" target="_blank"><strong>Discord</strong></a> ·
  <a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" target="_blank"><strong>GitHub Discussion</strong></a>
</div>

---

*This tutorial is part of the [Seeed AI Robotics Academy](../../README.md) — a course that will always be free, paired with the 100% open-source reBot robotic arm. Next in the series: How to run ACT, SmolVLA, π0, π0-FAST, GR00T N1.5, EO-1, Evo-1 and WALL-OSS on reBot Arm.*
