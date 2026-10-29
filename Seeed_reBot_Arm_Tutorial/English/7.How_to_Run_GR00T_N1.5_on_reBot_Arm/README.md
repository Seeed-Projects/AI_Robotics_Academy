# How to Run NVIDIA GR00T N1.5 on reBot Arm — Theory, Setup, Training & Deployment

<p align="center">
    <img width="600" src="https://files.seeedstudio.com/wiki/robotics/projects/rebot_arm/rebot_dm.png" alt="reBot Arm B601-DM" />
</p>

<p align="center">
    <a href="https://github.com/Seeed-Projects/reBot-DevArm/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT" /></a>
    <img src="https://img.shields.io/badge/Python-3.12+-blue.svg" alt="Python Version" />
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20CUDA%20GPU-red.svg" alt="Platform" />
    <img src="https://img.shields.io/badge/Policy-groot%20DiT-76b900.svg?labelColor=black" alt="Policy: GR00T" />
</p>

**[GR00T N1](https://arxiv.org/abs/2503.14734)** is NVIDIA's open foundation model for generalist robots, built around a **dual-system VLA architecture**: System 2 — an Eagle-2 vision-language model — understands images and instructions, while System 1 — a diffusion Transformer (DiT) — generates smooth continuous actions via flow matching. Its successor **GR00T N1.5** ships in [LeRobot](https://huggingface.co/docs/lerobot/groot) as the policy type `groot` (model `nvidia/GR00T-N1.5-3B`, Apache 2.0).

GR00T has **higher environment requirements** than the other policies in this series — it depends on FlashAttention and requires a CUDA GPU. Get comfortable with [ACT](../3.How_to_Run_ACT_on_reBot_Arm/README.md) or [Pi0](../5.How_to_Run_Pi0_on_reBot_Arm/README.md) first, then come here for NVIDIA's cross-embodiment foundation model.

This tutorial covers the theory, the flash-attn environment, fine-tuning on your reBot Arm dataset, and on-robot evaluation.

Part of the [Seeed reBot Arm VLA tutorial series](../README.md) · [Seeed AI Robotics Academy](../../../README.md). If this guide helps you, please ⭐ star [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm/).

---

## Table of Contents

- [Which Robotic Arms Are Compatible with GR00T N1.5?](#which-robotic-arms-are-compatible-with-gr00t-n15)
- [GR00T Theory Deep-Dive](#gr00t-theory-deep-dive)
- [Hardware, Environment, and Data Collection](#hardware-environment-and-data-collection)
- [Install the GR00T Stack (Important)](#install-the-groot-stack-important)
- [Train GR00T N1.5](#train-gr00t-n15)
- [Evaluate on the Real Arm](#evaluate-on-the-real-arm)
- [FAQ](#faq)
- [References](#references)

---

## Which Robotic Arms Are Compatible with GR00T N1.5?

**Any LeRobot-compatible arm with a recorded dataset can fine-tune GR00T N1.5** — its embodiment-specific state/action encoders and decoder adapt the shared DiT core to your robot's dimensions and control semantics. Verified on Seeed's open hardware:

| Robotic arm | Actuation | Interface | Status | Buy |
| :--- | :--- | :--- | :--- | :--- |
| **reBot Arm B601-DM** (Seeed) | Damiao DM brushless servos | USB-CAN bridge (`/dev/ttyACM0`) | ✅ Verified, this guide | [reBot Arm B601-DM Bundle](https://www.seeedstudio.com/reBot-Arm-B601-DM-Bundle.html) |
| **reBot Arm B601-RS** (Seeed) | RobStride RS actuators | SocketCAN (`can0`) / PCAN | ✅ Verified | [reBot Arm B601-RS Kit](https://www.seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html) |
| **SO-ARM101 / SO-101** (Seeed × LeRobot) | Feetech serial servos | USB serial (`/dev/ttyACM0`) | ✅ Verified | [SO-101 Low-Cost AI Arm Kit Pro](https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html) |

The original paper's platform was the Fourier GR-1 humanoid (language-conditioned bimanual tasks), with cross-embodiment pre-training spanning simulation benchmarks across many body types. GR00T N1.5 fine-tuning on SO-101-style tabletop arms is a well-trodden path in the LeRobot ecosystem.

---

## GR00T Theory Deep-Dive

*Condensed from the Seeed paper-reading deep-dive: [NVIDIA GR00T — Adaptation & Fine-Tuning](https://wiki.seeedstudio.com/robotics/paper_reading_gr00t_n1/). Paper: [arXiv:2503.14734](https://arxiv.org/abs/2503.14734).*

### The Same Learning Loop, Foundation-Model Scale

From ACT to GR00T, the workflow is unchanged — teleoperate → record → train → deploy action chunks. The difference: ACT learns one robot and a task family from demonstrations; **GR00T N1 first learns a foundation across bodies and tasks** (robot trajectories, human videos, synthetic data), then adapts to your particular robot.

### Dual-System Architecture

| System | Module | Role | Rate |
| :--- | :--- | :--- | :--- |
| **System 2** (thinks) | Eagle-2 VLM | Interpret images, language, and tasks | ~10 Hz |
| **System 1** (acts) | Diffusion Transformer (DiT) | Generate smooth continuous actions via flow matching | Higher, closed-loop |

Not independent programs: cross-attention connects them, and training optimizes them jointly end-to-end. Eagle-2 takes 224×224 images (64 tokens/frame); the authors use intermediate layer-12 VLM features — faster and better downstream than final features.

### Cross-Embodiment Design

- **Shared core** (VLM + DiT): visual, language, and behavioral knowledge across robots.
- **Embodiment-specific interfaces**: state encoder, action encoder, action decoder — translating your platform's dimensions and control semantics to/from the shared features. Cross-robot learning is not just zero-padding: each value's meaning matters.

### The Data Pyramid

| Level | Data | Quantity | Role |
| :--- | :--- | :--- | :--- |
| Bottom | Web data + human videos | Largest | Which actions and tasks exist |
| Middle | Simulation + neural trajectories | Large | How tasks can vary |
| Top | Real robot data | Smallest | How this body executes |

Moving up the pyramid reduces quantity but increases embodiment specificity. **Neural trajectory generation** amplifies scarce real demonstrations into many simulated variations, and experiments show priors + generated data improve few-shot real-robot task learning.

📖 Full details: [NVIDIA GR00T — Adaptation & Fine-Tuning (Seeed Paper Reading)](https://wiki.seeedstudio.com/robotics/paper_reading_gr00t_n1/).

---

## Hardware, Environment, and Data Collection

**Hardware, shopping links, base environment, calibration, teleoperation, cameras, and dataset recording are shared with the [Pi0.5 tutorial](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md)** — record ~50+ episodes with two camera views on your B601-DM / B601-RS / SO-ARM101 exactly as in [Step 4](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-4-record-a-dataset).

GR00T additionally requires:

- An **NVIDIA CUDA GPU** (no CPU path — FlashAttention requires CUDA).
- A compatible PyTorch version — see the next section.

---

## Install the GR00T Stack (Important)

GR00T N1.5 requires **flash-attn** and works only on **CUDA-capable devices**. Recommended order:

1. Set up the base environment (Python, CUDA, drivers) per the [Pi0.5 tutorial](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#environment-setup) — **do not install `lerobot` yet in this step**.

2. Install PyTorch (official version range):

   ```bash
   pip install "torch>=2.2.1,<2.8.0" "torchvision>=0.21.0,<0.23.0"
   ```

   > **RTX 50 series:** use Python 3.10, CUDA 12.8, Torch 2.7.1:
   > ```bash
   > pip install torch==2.7.1 torchvision==0.22.1 torchaudio==2.7.1 --index-url https://download.pytorch.org/whl/cu128
   > ```

3. Install flash-attn:

   ```bash
   pip install ninja "packaging>=24.2,<26.0"
   pip install "flash-attn>=2.5.9,<3.0.0" --no-build-isolation
   python -c "import flash_attn; print(f'Flash Attention {flash_attn.__version__} imported successfully')"
   ```

   > **RTX 50 series:** `pip install flash_attn==2.8.0.post2 torch==2.7.1 --no-build-isolation`

4. Install LeRobot's groot dependency:

   ```bash
   pip install "lerobot[groot]"
   ```

> **If flash-attn installation fails**, it is usually (1) PyTorch/CUDA version mismatch, (2) missing build dependencies, or (3) an environment that is too new or too old — re-check the versions above against the official GR00T documentation.

---

## Train GR00T N1.5 (Fine-Tuning)

The official example is multi-GPU; single-GPU also works in single-process mode. Multi-GPU (replace the variables):

```bash
accelerate launch \
  --multi_gpu \
  --num_processes=$NUM_GPUS \
  $(which lerobot-train) \
  --output_dir=$OUTPUT_DIR \
  --save_checkpoint=true \
  --batch_size=$BATCH_SIZE \
  --steps=$NUM_STEPS \
  --save_freq=$SAVE_FREQ \
  --log_freq=$LOG_FREQ \
  --policy.push_to_hub=true \
  --policy.type=groot \
  --policy.repo_id=$REPO_ID \
  --policy.tune_diffusion_model=false \
  --dataset.repo_id=$DATASET_ID \
  --wandb.enable=true \
  --wandb.disable_artifact=true \
  --job_name=$JOB_NAME
```

Parameters you will actually touch:

- `--dataset.repo_id` — your training dataset (Hub `username/dataset_name` or the local cache `repo_id`).
- `--output_dir` — training output directory (weights/checkpoints land here).
- `--steps`, `--batch_size` — GR00T is a large model; **reduce `batch_size` first** if you run out of VRAM.
- `--policy.repo_id` — the Hub model repository to create if pushing the model.
- `--policy.tune_diffusion_model=false` — keep the DiT frozen by default; enable to fine-tune the action system too.

License: the model follows **Apache 2.0** (consistent with the original GR00T repository).

Official reference: [LeRobot GR00T N1.5 documentation](https://huggingface.co/docs/lerobot/groot).

---

## Evaluate on the Real Arm

After training, evaluate with `lerobot-record` like any other policy — here for a reBot B601-DM single arm:

```bash
lerobot-record \
  --robot.type=seeed_b601_dm_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.can_adapter=damiao \
  --robot.cameras='{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, side: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30} }' \
  --robot.id=follower1 \
  --display_data=true \
  --dataset.repo_id=${HF_USER}/eval_groot_rebot \
  --dataset.num_episodes=10 \
  --dataset.single_task="Grab the black cube and put it in the box" \
  --policy.path=${HF_USER}/groot-rebot \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=10
```

(B601-RS: `--robot.type=seeed_b601_rs_follower --robot.port=can0 --robot.can_adapter=socketcan`; SO-ARM101: `--robot.type=so101_follower`.) Camera keys and `single_task` must match the recording session; keep `eval_`-prefixed dataset names.

---

## FAQ

**`Could not connect on port /dev/ttyUSB0 or /dev/ttyACM0`** — `sudo chmod 666 /dev/ttyUSB* /dev/ttyACM*`.

**flash-attn install fails** — check PyTorch/CUDA match, build deps, and the version pins above; RTX 50 series needs the special pins.

**Out of VRAM** — reduce `--batch_size` first; keep `--policy.tune_diffusion_model=false`.

**No CUDA GPU** — GR00T N1.5 is CUDA-only; use [SmolVLA](../4.How_to_Run_SmolVLA_on_reBot_Arm/README.md) (CPU-capable) or the no-code [SenseCraft Robotics](../2.SenseCraft_Robotics_No_Code_Guide/README.md) instead.

**Which LeRobot repo?** — the verified fork `https://github.com/Seeed-Projects/lerobot.git`.

---

## References

- GR00T N1 paper: [GR00T N1: An Open Foundation Model for Generalist Humanoid Robots](https://arxiv.org/abs/2503.14734) (NVIDIA)
- LeRobot: [repository](https://github.com/huggingface/lerobot) · [GR00T N1.5 docs](https://huggingface.co/docs/lerobot/groot)
- Seeed deep-dive: [NVIDIA GR00T — Adaptation & Fine-Tuning](https://wiki.seeedstudio.com/robotics/paper_reading_gr00t_n1/)
- Source wiki tutorial: [reBot Arm B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/)
- Series: [Pi0.5](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md) · [Pi0](../5.How_to_Run_Pi0_on_reBot_Arm/README.md) · [ACT](../3.How_to_Run_ACT_on_reBot_Arm/README.md) · [SmolVLA](../4.How_to_Run_SmolVLA_on_reBot_Arm/README.md) · [Pi0-FAST](../6.How_to_Run_Pi0_FAST_on_reBot_Arm/README.md)

---

## Tech Support & Product Discussion

<div align="center">
  <a href="https://forum.seeedstudio.com/" target="_blank"><strong>Seeed Forum</strong></a> ·
  <a href="https://www.seeedstudio.com/contacts" target="_blank"><strong>Contact Us</strong></a> ·
  <a href="https://discord.gg/eWkprNDMU7" target="_blank"><strong>Discord</strong></a> ·
  <a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" target="_blank"><strong>GitHub Discussion</strong></a>
</div>
