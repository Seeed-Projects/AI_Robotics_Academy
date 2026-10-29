# How to Run Pi0 (π0) on reBot Arm — Theory, Setup, Training & Deployment

<p align="center">
    <img width="600" src="https://files.seeedstudio.com/wiki/robotics/projects/rebot_arm/rebot_dm.png" alt="reBot Arm B601-DM" />
</p>

<p align="center">
    <a href="https://github.com/Seeed-Projects/reBot-DevArm/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT" /></a>
    <img src="https://img.shields.io/badge/Python-3.12+-blue.svg" alt="Python Version" />
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Ubuntu%20%7C%20Jetson%20Orin-orange.svg" alt="Platform" />
    <img src="https://img.shields.io/badge/Policy-pi0%20%CF%800-blueviolet.svg" alt="Policy: Pi0" />
</p>

**π0 (Pi0)** is the Vision-Language-Action flow model for general robot control from [Physical Intelligence](https://www.physicalintelligence.company/) ([arXiv:2410.24164](https://arxiv.org/abs/2410.24164)). It combines a pre-trained **PaliGemma** VLM backbone (~3B parameters) with a ~300M-parameter **action expert**, generates continuous action chunks with **conditional flow matching**, and is pre-trained across more than 10,000 hours of cross-robot demonstrations — single-arm, bimanual, and mobile manipulators.

In LeRobot it is the policy type `pi0`, fine-tuned from `lerobot/pi0_base`. You can think of it as: **it sees camera images, understands a natural-language instruction, and outputs the actions that drive the arm** — the base model that π0.5 later extended toward open-world generalization.

This tutorial runs it on the **reBot Arm B601-DM / B601-RS** and **SO-ARM101**: theory, setup, data collection, fine-tuning, and deployment.

> 📚 **Want the full tutorial and more resources?** This GitHub guide is a companion to the complete documentation on the **[Seeed Studio Wiki](https://wiki.seeedstudio.com/robotics_page/)**, which is continuously updated and includes video walkthroughs, hardware assembly, and troubleshooting:
> - **Full wiki tutorials:** [reBot Arm B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/) · [reBot Arm B601-RS with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_rs_lerobot/) · [SO-ARM101 with LeRobot](https://wiki.seeedstudio.com/lerobot_so100m_new/)
> - **π0 theory deep-dive:** [π0 — General-Purpose Robot Control](https://wiki.seeedstudio.com/robotics/paper_reading_pi0/)
> - **Hardware setup:** [reBot Arm B601-DM Getting Started](https://wiki.seeedstudio.com/rebot_b601_dm_getting_started/) · [reBot Arm B601-RS Getting Started](https://wiki.seeedstudio.com/rebot_b601_rs_getting_started/)

Part of the [Seeed reBot Arm VLA tutorial series](../README.md) · [Seeed AI Robotics Academy](../../../README.md). If this guide helps you, please ⭐ star [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm/).

---

## Table of Contents

- [Which Robotic Arms Are Compatible with Pi0?](#which-robotic-arms-are-compatible-with-pi0)
- [Pi0 Theory Deep-Dive](#pi0-theory-deep-dive)
- [Hardware, Environment, and Data Collection](#hardware-environment-and-data-collection)
- [Train Pi0](#train-pi0)
- [Evaluate on the Real Arm](#evaluate-on-the-real-arm)
- [Pi0 vs Pi0.5 vs SmolVLA](#pi0-vs-pi05-vs-smolvla)
- [FAQ](#faq)
- [References](#references)

---

## Which Robotic Arms Are Compatible with Pi0?

**Any LeRobot-compatible arm with a recorded dataset can fine-tune and run Pi0** — the policy auto-adapts to your robot's state/action dimensions and camera count from the dataset. Verified on Seeed's open hardware:

| Robotic arm | Actuation | Interface | Status | Buy |
| :--- | :--- | :--- | :--- | :--- |
| **reBot Arm B601-DM** (Seeed) | Damiao DM brushless servos | USB-CAN bridge (`/dev/ttyACM0`) | ✅ Verified, this guide | [reBot Arm B601-DM Bundle](https://www.seeedstudio.com/reBot-Arm-B601-DM-Bundle.html) |
| **reBot Arm B601-RS** (Seeed) | RobStride RS actuators | SocketCAN (`can0`) / PCAN | ✅ Verified | [reBot Arm B601-RS Kit](https://www.seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html) |
| **SO-ARM101 / SO-101** (Seeed × LeRobot) | Feetech serial servos | USB serial (`/dev/ttyACM0`) | ✅ Verified | [SO-101 Low-Cost AI Arm Kit Pro](https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html) |

The original π0 pre-training covers seven robot-configuration categories — UR5e single/bimanual, Franka, Trossen/ARX/AgileX bimanual, and mobile manipulators (14–18D action spaces) — so ALOHA-style bimanual rigs, Koch v1.1, and other LeRobot arms sit well within its training distribution.

---

## Pi0 Theory Deep-Dive

*Condensed from the Seeed paper-reading deep-dive: [π0 — General-Purpose Robot Control](https://wiki.seeedstudio.com/robotics/paper_reading_pi0/). Paper: [arXiv:2410.24164](https://arxiv.org/abs/2410.24164).*

### The Three Bottlenecks π0 Addresses

1. **Data scale** — one robot, one task rarely covers enough situations, including failures and recoveries.
2. **Architecture** — language models generate discrete tokens; robots need precise, high-frequency, continuous actions.
3. **Training recipe** — which data to use, when, and how pre-training vs post-training divide the work.

### Architecture: VLM Backbone + Action Expert + Flow Matching

```text
Images (≤3 slots) + language instruction + robot state
        ↓
PaliGemma VLM (~3B)  ←→  Action expert (~300M)
   (image + language tokens)   (state + noisy action chunk)
        ↓  conditional flow matching, H=50
Continuous action chunk (50 × up-to-18 dims)
```

- **The VLM supplies understanding; the action expert supplies control.** Two weight sets exchange information through Transformer self-attention, so actions depend on language and images without turning the whole VLM into an action network.
- **Cross-embodiment interface:** up to three image slots (missing views masked), state/action padded to 18 dimensions — a fixed interface while the model learns platform-specific behavior. Padding is a training interface only; your controller receives just the valid dimensions.
- **Data mixture:** internal 903M-timestep dexterous data (68 tasks, 7 configurations) + open data (OXE, Bridge v2, DROID ≈ 9.1%), with task–robot groups weighted n^0.43 to prevent large groups from dominating.

### Conditional Flow Matching: From Noise to Continuous Actions

Instead of autoregressively decoding action tokens, π0 generates the **entire 50-step chunk from noise in parallel**: the action expert is trained to predict the flow vector field that denoises a noisy chunk toward the demonstrated one; at inference, ten iterations of updates turn pure noise into a smooth, executable action chunk.

**Why chunks:** one inference yields 50 control points (high-frequency output without per-dimension token decoding), and the chunk models dependencies across actions (trajectory coherence). π0 predicts H=50 but executes only 16–25 steps before observing again — balancing inference overhead against feedback frequency.

### Pre-Training vs Post-Training

- **Pre-training** (broad, messy data): exposes the model to unusual states, mistakes, and recovery behavior — supporting adaptability.
- **Post-training** (high-quality demonstrations, ~5 hours for simple to 100+ hours for hard tasks): shapes consistent, fluent execution.
- In LeRobot fine-tuning you do the post-training part on top of `lerobot/pi05_base`-style checkpoints.

**Results:** on direct prompting (shirt folding, table clearing, grocery bagging) full π0 beats OpenVLA/Octo/π0-small; on new tasks via post-training, pre-training sometimes nearly doubles performance vs training from scratch. Evidence boundary: "generalist" covers the studied manipulation platforms — not arbitrary robots or domains.

📖 Full derivation: [π0 — General-Purpose Robot Control (Seeed Paper Reading)](https://wiki.seeedstudio.com/robotics/paper_reading_pi0/).

---

## Hardware, Environment, and Data Collection

**Hardware, shopping links, environment setup, calibration, teleoperation, cameras, and recording are shared with the [Pi0.5 tutorial](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md).** One extra dependency — the `pi` extra:

```bash
cd ~/rebot_lerobot/lerobot
pip install -e ".[pi]"
cd ~
```

> **Tip:** on an older LeRobot (e.g. 0.4.0), install the `pi` dependency from GitHub source instead:
> ```bash
> pip install "lerobot[pi]@git+https://github.com/huggingface/lerobot.git"
> ```

**Dataset:** record ~50+ episodes of your task with two camera views (`front` + `side`), exactly as in the [Pi0.5 tutorial — Step 4](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-4-record-a-dataset) (B601-DM / B601-RS / SO-ARM101 command variants included there).

---

## Train Pi0

Fine-tune the pretrained base on your dataset:

```bash
lerobot-train \
  --policy.type=pi0 \
  --dataset.repo_id=${HF_USER}/my_dataset \
  --job_name=pi0_training \
  --output_dir=outputs/pi0_training \
  --policy.pretrained_path=lerobot/pi0_base \
  --policy.repo_id=${HF_USER}/my_pi0_policy \
  --policy.compile_model=true \
  --policy.gradient_checkpointing=true \
  --policy.dtype=bfloat16 \
  --policy.freeze_vision_encoder=false \
  --policy.train_expert_only=false \
  --steps=3000 \
  --policy.device=cuda \
  --batch_size=32 \
  --wandb.enable=false
```

(For a local dataset, use your `repo_id` like `seeed_rebot_b601_dm/test` and add `--policy.push_to_hub=false`.)

**The parameters that matter for π0:**

| Parameter | Meaning |
| :--- | :--- |
| `--policy.pretrained_path=lerobot/pi0_base` | Pretrained base; `lerobot/pi0_libero` is the LIBERO-oriented variant |
| `--policy.compile_model=true` | Compilation optimization — first compile slower, then faster training |
| `--policy.gradient_checkpointing=true` | Large VRAM savings; enable when memory is tight |
| `--policy.dtype=bfloat16` | Mixed precision — faster/lighter when hardware supports it |
| `--policy.train_expert_only=true` | **VRAM-saving trick:** freeze the VLM, train only the action expert + projections — good first experiment or small-data run |
| `--policy.freeze_vision_encoder=false` | Unfreeze the vision encoder for fuller adaptation |
| `--steps=3000` | Fine-tuning steps; scale with dataset size and task difficulty |
| `--batch_size=32` | Lower first if OOM, then grow |

**Resume from a checkpoint:**

```bash
lerobot-train \
  --config_path=outputs/pi0_training/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

Official reference: [LeRobot Pi0 documentation](https://huggingface.co/docs/lerobot/pi0).

---

## Evaluate on the Real Arm

```bash
lerobot-record \
  --robot.type=seeed_b601_dm_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.can_adapter=damiao \
  --robot.cameras='{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: "MJPG"}, side: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30, fourcc: "MJPG"} }' \
  --robot.id=follower1 \
  --display_data=false \
  --dataset.repo_id=${HF_USER}/eval_my_pi0_test \
  --dataset.single_task="Put lego brick into the transparent box" \
  --dataset.episode_time_s=50 \
  --dataset.num_episodes=10 \
  --policy.path=outputs/pi0_training/checkpoints/last/pretrained_model
```

(B601-RS: `--robot.type=seeed_b601_rs_follower --robot.port=can0 --robot.can_adapter=socketcan`; SO-ARM101: `--robot.type=so101_follower` with the `so101_leader` teleoperator at `/dev/ttyACM1` when recording.)

Checklist: `--policy.path` = local checkpoint or Hub repo; camera keys and `single_task` must match the recording session; `eval_`-prefixed datasets keep evaluation data separate.

**Smoother control:** π0 outputs action chunks, so [async inference](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-6-evaluate-on-the-real-arm) (compute next chunk while executing the current one) removes move-pause-move behavior.

---

## Pi0 vs Pi0.5 vs SmolVLA

| | **π0** | **π0.5** | **SmolVLA** |
| :--- | :--- | :--- | :--- |
| Focus | General manipulation across robots/tasks | Open-world generalization + subtask hierarchy | Affordable community-data VLA |
| Backbone | PaliGemma ~3B + 300M expert | Same, plus subtask prediction | 450M (SmolVLM-2, first-half layers) |
| Data | >10,000 h cross-robot demonstrations | π0 recipe + web data + subtasks | Cleaned community datasets |
| Long tasks | Person / external VLM supplies intermediate instructions | Same model selects subtasks itself | Standard chunking |
| Fine-tune cost | GPU, bf16 recommended | Similar | Lightest of the three |
| Start here if… | You want the foundation-model baseline | You want the strongest generalization | You have limited compute |

Same dataset, three policies: run [ACT](../3.How_to_Run_ACT_on_reBot_Arm/README.md) → [SmolVLA](../4.How_to_Run_SmolVLA_on_reBot_Arm/README.md) → **π0** (this tutorial) → [π0.5](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md) and compare.

---

## FAQ

**`Could not connect on port /dev/ttyUSB0 or /dev/ttyACM0`** — `sudo chmod 666 /dev/ttyUSB* /dev/ttyACM*`.

**`No valid stream found in input file`** — `conda install ffmpeg=7.1.1 -c conda-forge`.

**Out of memory** — lower `--batch_size`, enable `--policy.gradient_checkpointing=true`, try `--policy.train_expert_only=true`.

**Older LeRobot version fails to find pi dependencies** — `pip install "lerobot[pi]@git+https://github.com/huggingface/lerobot.git"`.

**`mean is infinity` at evaluation** — camera keys must exactly match the recording session.

**No local GPU?** — cloud training via the [LeRobot pi0 docs](https://huggingface.co/docs/lerobot/pi0), or the no-code [SenseCraft Robotics](../2.SenseCraft_Robotics_No_Code_Guide/README.md) platform.

**Which LeRobot repo?** — the verified fork `https://github.com/Seeed-Projects/lerobot.git`.

---

## References

- π0 paper: [π0: A Vision-Language-Action Flow Model for General Robot Control](https://arxiv.org/abs/2410.24164) (Physical Intelligence)
- LeRobot: [repository](https://github.com/huggingface/lerobot) · [Pi0 docs](https://huggingface.co/docs/lerobot/pi0) · [pretrained pi0_base](https://huggingface.co/lerobot/pi0_base)
- Seeed deep-dive: [π0 — General-Purpose Robot Control](https://wiki.seeedstudio.com/robotics/paper_reading_pi0/)
- Source wiki tutorial: [reBot Arm B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/)
- Series: [Pi0.5](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md) · [ACT](../3.How_to_Run_ACT_on_reBot_Arm/README.md) · [SmolVLA](../4.How_to_Run_SmolVLA_on_reBot_Arm/README.md)

---

## Tech Support & Product Discussion

<div align="center">
  <a href="https://forum.seeedstudio.com/" target="_blank"><strong>Seeed Forum</strong></a> ·
  <a href="https://www.seeedstudio.com/contacts" target="_blank"><strong>Contact Us</strong></a> ·
  <a href="https://discord.gg/eWkprNDMU7" target="_blank"><strong>Discord</strong></a> ·
  <a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" target="_blank"><strong>GitHub Discussion</strong></a>
</div>
