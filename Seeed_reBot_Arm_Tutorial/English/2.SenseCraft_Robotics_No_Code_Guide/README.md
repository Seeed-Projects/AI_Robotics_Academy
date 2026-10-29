# SenseCraft Robotics: Run a Robot Arm Without Writing Code (Windows / macOS / Linux)

<p align="center">
    <img width="600" src="https://files.seeedstudio.com/wiki/sensecraft-robotics/guide-assets/overview/sensecraft-robotics-official-website-en.webp" alt="SenseCraft Robotics" />
</p>

**[SenseCraft™ Robotics](https://sensecraft.seeed.cc/en?utm_source=rebot_wiki&utm_medium=wiki&utm_campaign=sensecraft_banner)** is Seeed's no-code training and runtime platform for real-world robotic arm tasks. It covers the whole workflow — device connection, calibration, data collection, model training, and validation — in a guided desktop client, so you can go from demonstration data to a running arm **without installing Python, without a terminal, and without a local GPU**.

This guide is ported from the official wiki tutorials:
[Software Overview](https://wiki.seeedstudio.com/sensecraft_robotics/) · [SO-ARM101 Guide](https://wiki.seeedstudio.com/sensecraft_robotics_so_arm101/) · [reBot Arm 102 + B601-DM Guide](https://wiki.seeedstudio.com/sensecraft_robotics_rebot_arm_102_b601_dm/) · [reBot Arm 102 + B601-RS Guide](https://wiki.seeedstudio.com/sensecraft_robotics_rebot_arm_102_b601_rs/) · [FAQ](https://wiki.seeedstudio.com/sensecraft_robotics_common_issues/)

> 📚 **Want the full tutorial and more resources?** This GitHub guide is a companion to the complete documentation on the **[Seeed Studio Wiki](https://wiki.seeedstudio.com/robotics_page/)**, which is continuously updated and includes step-by-step screenshots, video walkthroughs, and troubleshooting:
> - **Software overview:** [SenseCraft Robotics Software Overview](https://wiki.seeedstudio.com/sensecraft_robotics/)
> - **Per-arm guides:** [SO-ARM101 Training Guide](https://wiki.seeedstudio.com/sensecraft_robotics_so_arm101/) · [reBot Arm 102 + B601-DM](https://wiki.seeedstudio.com/sensecraft_robotics_rebot_arm_102_b601_dm/) · [reBot Arm 102 + B601-RS](https://wiki.seeedstudio.com/sensecraft_robotics_rebot_arm_102_b601_rs/)
> - **Troubleshooting:** [Common Issues FAQ](https://wiki.seeedstudio.com/sensecraft_robotics_common_issues/)

It is the no-code companion to the [How to Run Pi0.5 on reBot Arm](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md) tutorial in this repository — same arms, same workflow stages, but everything happens in a GUI instead of a Python environment.

---

## Table of Contents

- [Why SenseCraft Robotics](#why-sensecraft-robotics)
- [Supported Arms (with Shopping Links)](#supported-arms-with-shopping-links)
- [Computer and Software Requirements](#computer-and-software-requirements)
- [Install the Software and Create an Account](#install-the-software-and-create-an-account)
- [The 6-Step Workflow](#the-6-step-workflow)
- [Data Collection Best Practices](#data-collection-best-practices)
- [Training: Cloud vs Local](#training-cloud-vs-local)
- [Run the Model on the Arm](#run-the-model-on-the-arm)
- [FAQ Pointers](#faq-pointers)

---

## Why SenseCraft Robotics

| | |
| :--- | :--- |
| ⚡ **No environment setup** | No Python, no conda, no dependency troubleshooting |
| 🖥️ **Cross-platform** | Windows / macOS / Linux desktop client |
| ☁️ **Cloud training** | No local GPU needed; GPU usage is billed per actual runtime |
| 🧭 **Guided workflow** | 6 steps from unboxing to a running model |
| 🔁 **Project-based** | Projects keep devices, datasets, actions, and models together for iteration |

Use cases: education and STEM training, university labs and embodied-AI research, innovation centers building demo prototypes.

## Supported Arms (with Shopping Links)

| Model | Device combination (Leader + Follower) | Buy | Dedicated wiki guide |
| :--- | :--- | :--- | :--- |
| 01 | **SO-ARM101 + SO-ARM101** | [Buy now](https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html?utm_source=sensecraft_wiki&utm_medium=socil) | [SO-ARM101 Guide](https://wiki.seeedstudio.com/sensecraft_robotics_so_arm101/) |
| 02 | **reBot Arm 102 + B601-DM** | [Buy now](https://www.seeedstudio.com/reBot-Arm-B601-DM-p-6740.html?utm_source=sensecraft_wiki&utm_medium=socil) | [B601-DM Guide](https://wiki.seeedstudio.com/sensecraft_robotics_rebot_arm_102_b601_dm/) |
| 03 | **reBot Arm 102 + B601-RS** | [Buy now](https://www.seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html?utm_source=sensecraft_wiki&utm_medium=socil) | [B601-RS Guide](https://wiki.seeedstudio.com/sensecraft_robotics_rebot_arm_102_b601_rs/) |

Each dedicated guide covers that combination's specific wiring, power, and calibration notes. Camera kits and mounts: [Robot Arm Camera Mount](https://www.seeedstudio.com/Robot-Arm-Camera-Mount-for-LeRobot-SO-10&reBot-B601-p-7003.html).

## Computer and Software Requirements

- A Windows, macOS, or Linux computer (local training additionally requires a compatible GPU).
- A stable network connection for login, project management, cloud training, and inference verification.
- The latest version of the SenseCraft Robotics client, from the [official download page](https://sensecraft.seeed.cc/en).
- A registered SenseCraft account.

> **OS-specific notes** (from the official FAQ):
> - **macOS / Linux:** make sure the system allows serial-port access for the client.
> - **B601-RS CAN on Linux / Jetson:** check that the SocketCAN interface exists (e.g. `can0`); on **macOS / Windows**, make sure the PCAN runtime is available.
> - **Windows:** if serial ports are missing, allow the driver installation when the CAN adapter is first plugged in.

## Install the Software and Create an Account

1. **Download** — open the [SenseCraft Robotics official page](https://sensecraft.seeed.cc/en), go to the software download section, and pick the installer for your operating system.
2. **Install** — run the installer and follow the wizard. Reserve disk space for videos, datasets, and models.
3. **Sign up** — in the user center, create an account, verify your email, and set a password.
4. **Log in** — launch the client and click **Log in**; the system browser opens the SenseCraft user center to complete sign-in. The client itself never collects your password. You land on the **Projects** page.

**Settings worth knowing:** click **Settings** in the top navigation for personal information, password changes, third-party service connections (e.g. Hugging Face), and version updates.

## The 6-Step Workflow

The platform workflow — shown here for the SO-ARM101 guide; the other arms follow the same stages with their own wiring details:

### Step 1: Device Setup (model, serial ports, calibration)

1. **Create a project** — **New Project** → name and description → **Create and Start**. The Project Overview shows arm status, device setup, datasets, training, models, and operation.
2. **Select the device model** — in **Device Pairing**, choose your combination (e.g. **SO-ARM101 + SO-ARM101**).
3. **Bind the serial ports** — unplug both arms, click **Rescan**, and note the ports; reconnect the **Leader**, rescan, and bind the new port to the Leader; reconnect the **Follower**, rescan, and bind it to the Follower. Bind only newly appearing ports — never identify devices by COM number alone.
4. **Calibrate** — the page offers **Automatic Calibration** (for arms with reliable joint stops: move both arms to the middle posture, confirm, and start) and **Manual Calibration** (guided, joint-by-joint, when stops are unreliable).
5. **Verify teleoperation** — click **Start Teleoperation**, move the Leader slightly, confirm the Follower follows smoothly and in the right direction, then confirm or return to recalibrate.
6. **Bind the cameras** — connect two cameras, rescan, and assign them to **Front Camera** and **Side Camera**. Two views are required for data collection and training.

### Step 2: Data Collection

Open **Dataset → Collect Data** and configure the task (dataset name, task description, episodes, duration, rest time — see [best practices](#data-collection-best-practices)). Then record episodes: place the object at Point A, click **Start Recording**, use the Leader to walk the Follower through the task, and end the episode (wait, or press the arrow / Space). Use **Action Replay** to record reusable action segments separately from datasets.

### Step 3: Dataset Management

Inspect episodes (video / action replay), batch-delete abnormal ones (deletion creates a copy — the original is kept), and merge datasets whose task descriptions, device combinations, and camera configurations match exactly.

### Step 4: Training

Pick a dataset, choose **Cloud** (GPU, consumes credits) or **Local** (free, needs a compatible local GPU), set the training steps, and click **Start Training**. Progress and history appear under **Current Training Tasks**.

### Step 5: Model

The **Model** page lists training results. Confirm the model matches the current project's task and device combination.

### Step 6: Run

Select a model, choose **Cloud Inference** or **Local Inference**, configure the parameters, and start. The arm now performs the task autonomously — watch it, and keep the workspace clear.

## Data Collection Best Practices

- **Small batches first:** record ~10 episodes per batch; after 2–3 batches keep ~20–30 valid episodes for the first training run; add data for harder tasks.
- **Keep it stable:** fixed A/B points, same object category, fixed camera positions, stable background and lighting. Identical task descriptions when merging.
- **Visibility:** arms, gripper, object, and both points must stay visible in both camera views; never let key actions leave the frame.
- **Consistency over variety first:** keep the procedure identical; add small position/angle changes only when you need generalization.
- **Safety:** stop immediately on any loss-of-control or collision risk; do not disconnect arms mid-run.

Recommended episodes and steps by task difficulty (from the official guide):

| Task difficulty | Example | Episodes | Steps |
| :--- | :--- | ---: | ---: |
| Beginner | Fixed Point A → Point B | ~20 | 10,000–15,000 |
| Intermediate | Random pick and place | ~50 | 30,000–40,000 |
| Advanced | Multi-step / long tasks | ~100 | 80,000–100,000 |

## Training: Cloud vs Local

| | Cloud | Local |
| :--- | :--- | :--- |
| Hardware | Cloud GPU | Your computer's GPU |
| Cost | Credits, billed by actual runtime | Free |
| Best for | No local GPU, quick starts | Iterating on data at your desk |

**Inference comparison** (from the Operation chapter):

| | Cloud Inference | Local Inference |
| :--- | :--- | :--- |
| Hardware | High-performance cloud GPU | Local CPU (current platform) |
| Sensitivity | Network quality | Local CPU performance |
| Key parameters | Action count (default 120), chunk threshold (default 0.9) | Run count, time limit per run |
| Output | Inference verification | Auto-generated dataset from the runs |

## Run the Model on the Arm

1. Select a model marked as supporting the current arms.
2. Choose Cloud or Local inference and configure parameters.
3. Confirm the workspace is safe, then **Start**.
4. Unsatisfactory results → go back to **Dataset**, delete abnormal episodes, add valid data, retrain.

## FAQ Pointers

- **Serial ports not found** — use a data-capable USB cable, power the arm on, connect the Leader before the Follower, allow serial access on macOS/Linux, make sure no other program holds the port; then rescan.
- **B601-RS cannot find CAN** — check the CAN adapter and driver; SocketCAN (`can0`) on Linux/Jetson; PCAN runtime on macOS/Windows.
- **Camera has no image or low fps** — lower resolution/quality, lower sampling speed, close extra previews, rescan and rebind.
- **Calibration fails** — retry per the on-screen flow; consult the [official FAQ](https://wiki.seeedstudio.com/sensecraft_robotics_common_issues/).
- **Credits** — billing, refunds, and expiration are explained in the [Credits and Billing](https://wiki.seeedstudio.com/sensecraft_robotics/#credits) section.

---

## Tech Support & Product Discussion

<div align="center">
  <a href="https://forum.seeedstudio.com/" target="_blank"><strong>Seeed Forum</strong></a> ·
  <a href="https://www.seeedstudio.com/contacts" target="_blank"><strong>Contact Us</strong></a> ·
  <a href="https://discord.gg/eWkprNDMU7" target="_blank"><strong>Discord</strong></a>
</div>

---

*Ported from the Seeed Studio wiki. Prefer the open-source LeRobot + Pi0.5 path instead? See [How to Run Pi0.5 on reBot Arm](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md).*
