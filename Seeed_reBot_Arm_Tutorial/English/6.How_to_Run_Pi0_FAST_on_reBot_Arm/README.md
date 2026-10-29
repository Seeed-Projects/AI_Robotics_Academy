# How to Run Pi0-FAST (π0-FAST) on reBot Arm — Theory, Setup, Training & Deployment

<p align="center">
    <img width="600" src="https://files.seeedstudio.com/wiki/robotics/projects/rebot_arm/rebot_dm.png" alt="reBot Arm B601-DM" />
</p>

<p align="center">
    <a href="https://github.com/Seeed-Projects/reBot-DevArm/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT" /></a>
    <img src="https://img.shields.io/badge/Python-3.12+-blue.svg" alt="Python Version" />
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Ubuntu%20%7C%20Jetson%20Orin-orange.svg" alt="Platform" />
    <img src="https://img.shields.io/badge/Policy-pi0fast%20DCT%2BBPE-cyan.svg" alt="Policy: Pi0-FAST" />
</p>

**π0-FAST (Pi0-FAST)** is the tokenized-action variant of π0 from [Physical Intelligence](https://www.physicalintelligence.company/) ([arXiv:24104959](https://arxiv.org/abs/24104959), "FAST: Efficient Action Tokenization for Vision-Language-Action Models"). Instead of generating continuous actions with a flow-matching expert, **π0-FAST compresses action chunks into discrete tokens with the FAST tokenizer — DCT + quantization + BPE — and predicts them autoregressively**, like text.

The headline result: a one-second, 50 Hz, 14-DoF T-shirt-folding chunk shrinks from **700 pointwise tokens to ~53 FAST tokens** (13.2× compression at similar reconstruction error), letting a standard autoregressive VLA learn high-rate dexterous control that pointwise tokenization fails on.

In LeRobot it is the policy type `pi0fast` (pretrained `lerobot/pi0fast_base`). This tutorial runs it on the **reBot Arm B601-DM / B601-RS** and **SO-ARM101**.

Part of the [Seeed reBot Arm VLA tutorial series](../README.md) · [Seeed AI Robotics Academy](../../../README.md). If this guide helps you, please ⭐ star [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm/).

---

## Table of Contents

- [Which Robotic Arms Are Compatible with Pi0-FAST?](#which-robotic-arms-are-compatible-with-pi0-fast)
- [Pi0-FAST Theory Deep-Dive](#pi0-fast-theory-deep-dive)
- [Hardware, Environment, and Data Collection](#hardware-environment-and-data-collection)
- [Train Pi0-FAST](#train-pi0-fast)
- [Evaluate on the Real Arm](#evaluate-on-the-real-arm)
- [Pi0-FAST vs Pi0: When Tokenization Wins](#pi0-fast-vs-pi0-when-tokenization-wins)
- [FAQ](#faq)
- [References](#references)

---

## Which Robotic Arms Are Compatible with Pi0-FAST?

**Any LeRobot-compatible arm with a recorded dataset can fine-tune Pi0-FAST** — same interface as π0/π0.5: the policy adapts to your robot's action space and cameras from the dataset metadata. Verified on Seeed's open hardware:

| Robotic arm | Actuation | Interface | Status | Buy |
| :--- | :--- | :--- | :--- | :--- |
| **reBot Arm B601-DM** (Seeed) | Damiao DM brushless servos | USB-CAN bridge (`/dev/ttyACM0`) | ✅ Verified, this guide | [reBot Arm B601-DM Bundle](https://www.seeedstudio.com/reBot-Arm-B601-DM-Bundle.html) |
| **reBot Arm B601-RS** (Seeed) | RobStride RS actuators | SocketCAN (`can0`) / PCAN | ✅ Verified | [reBot Arm B601-RS Kit](https://www.seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html) |
| **SO-ARM101 / SO-101** (Seeed × LeRobot) | Feetech serial servos | USB serial (`/dev/ttyACM0`) | ✅ Verified | [SO-101 Low-Cost AI Arm Kit Pro](https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html) |

FAST+ (the universal tokenizer) was trained on ~1 million one-second chunks spanning single-arm, bimanual, and mobile platforms with joint and end-effector controls at varied frequencies — LeRobot arms sit squarely in that distribution.

---

## Pi0-FAST Theory Deep-Dive

*Condensed from the Seeed paper-reading deep-dive: [π0-FAST — Efficient Action Tokenization](https://wiki.seeedstudio.com/robotics/paper_reading_pi0_fast/). Paper: [arXiv:24104959](https://arxiv.org/abs/24104959).*

### The Bottleneck: Pointwise Tokenization Fails on High-Rate Data

The naive way to make actions discrete — bin each timestep/dimension independently into 256 bins — produces **H × D tokens per chunk** (50 Hz × 14D × 1 s = 700 tokens). Neighboring values are nearly identical (`[0.200, 0.201, 0.202, 0.203]` often share one bin), so the model can minimize loss by repeating the previous token — **without learning the motion**. In the paper's spline experiment, pointwise error grows toward a constant while DCT error stays low: the failure is in the representation, not the task.

### FAST: Compress First, Predict Second

```text
Continuous chunk → DCT frequency coefficients → quantization → frequency-first flattening → BPE
        → ~n compressed action tokens → autoregressive VLA prediction → decode back to a continuous chunk
```

- **DCT (discrete cosine transform)** expresses the trajectory as cosine components at different frequencies — smooth motions become a few dominant low-frequency coefficients.
- **Frequency-first ordering + BPE** merge common integer subsequences into vocabulary entries, like subwords in language.
- **A token is not a joint angle** — FAST tokens need not correspond to any timestep or dimension; the sequence is generated, then decoded into the full H × D chunk.

Compression results (one-second chunks, similar reconstruction error):

| Dataset / task | Dims | Rate, horizon | Pointwise tokens | FAST tokens | Compression |
| :--- | :--- | :--- | :--- | :--- | :--- |
| BridgeV2 | 7 | 5 Hz, H=5 | 35 | 20 | 1.75× |
| DROID | 7 | 15 Hz, H=15 | 105 | 29 | 3.6× |
| Table bussing | 7 | 20 Hz, H=20 | 140 | 28 | 5.0× |
| T-shirt folding | 14 | 50 Hz, H=50 | 700 | 53 | **13.2×** |

Rule of thumb: **~30 tokens per arm per second**, regardless of control rate — extra control points add little new motion information.

### The Network

```text
2–3 RGB images + language instruction + binned proprioception
        ↓  PaliGemma-3B backbone (image + text tokens form the observation prefix)
FAST action tokens as an autoregressive suffix (n tokens)
        ↓  next-token prediction (cross-entropy), causal mask
generated tokens → reverse BPE → dequantize → inverse DCT → continuous chunk → controller
```

- Current state is **binned into 256 levels and tokenized as text** (state is a condition, not a high-rate output — simple binning suffices); future actions use FAST.
- Training: teacher forcing, next-token loss over the n action tokens; AdamW, lr 5e-5 after warmup, EMA 0.999, all weights fine-tuned.
- No CVAE encoder, no continuous flow decoder — a standard autoregressive VLM.

### FAST vs Pointwise vs π0

| | Pointwise autoregressive | π0 (flow matching) | π0-FAST |
| :--- | :--- | :--- | :--- |
| Representation | Discrete value per timestep/dim | Continuous chunk | Compressed discrete tokens |
| Objective | Next-token | Flow matching | Next-token |
| High-rate dexterity | Fails (repetition collapse) | Works, needs a dedicated expert | Works, ~30 tokens/s |
| Limitation | — | Continuous expert complexity | Lossy quantization; autoregressive inference still sequential |

On smaller datasets (LIBERO, T-shirt folding) π0-FAST and π0 perform comparably; on large table-bussing data FAST reaches strong performance sooner (π0 needs ~3× the steps to catch up).

📖 Full details: [π0-FAST — Efficient Action Tokenization (Seeed Paper Reading)](https://wiki.seeedstudio.com/robotics/paper_reading_pi0_fast/).

---

## Hardware, Environment, and Data Collection

**Hardware, shopping links, environment setup, calibration, teleoperation, cameras, and recording are shared with the [Pi0.5 tutorial](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md).** π0-FAST uses the same `pi` extra:

```bash
cd ~/rebot_lerobot/lerobot
pip install -e ".[pi]"
cd ~
```

**Dataset:** record ~50+ episodes with two camera views, exactly as in the [Pi0.5 tutorial — Step 4](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-4-record-a-dataset). Since FAST's advantage grows with control rate and data scale, keep your recording rate consistent (30 fps cameras, matching control frequency) and collect generously.

---

## Train Pi0-FAST

Follow the same policy-switching pattern as π0/π0.5 — set the policy type to `pi0fast` and fine-tune from the pretrained base:

```bash
lerobot-train \
  --policy.type=pi0fast \
  --dataset.repo_id=${HF_USER}/my_dataset \
  --job_name=pi0fast_training \
  --output_dir=outputs/pi0fast_training \
  --policy.pretrained_path=lerobot/pi0fast_base \
  --policy.repo_id=${HF_USER}/my_pi0fast_policy \
  --policy.dtype=bfloat16 \
  --policy.gradient_checkpointing=true \
  --steps=3000 \
  --policy.device=cuda \
  --batch_size=32 \
  --wandb.enable=false
```

(For a local dataset, use your `repo_id` like `seeed_rebot_b601_dm/test` and add `--policy.push_to_hub=false`.)

Notes:

- The wiki's training-command template (see the [Pi0 tutorial](../5.How_to_Run_Pi0_on_reBot_Arm/README.md#train-pi0)) applies: `--policy.dtype=bfloat16` and `--policy.gradient_checkpointing=true` when VRAM is tight; reduce `--batch_size` first on OOM.
- π0-FAST trains **all policy weights** by default (the paper fine-tunes everything) — on smaller GPUs, prefer reducing batch size over freezing the backbone.
- Autoregressive generation is sequential: expect slower per-chunk inference than π0's parallel denoising; plan evaluation pacing accordingly (see [async inference](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-6-evaluate-on-the-real-arm) to hide the latency).

**Resume from a checkpoint:**

```bash
lerobot-train \
  --config_path=outputs/pi0fast_training/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

Official reference: [LeRobot Pi0-FAST documentation](https://huggingface.co/docs/lerobot/pi0fast).

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
  --dataset.repo_id=${HF_USER}/eval_my_pi0fast_test \
  --dataset.single_task="Put lego brick into the transparent box" \
  --dataset.episode_time_s=50 \
  --dataset.num_episodes=10 \
  --policy.path=outputs/pi0fast_training/checkpoints/last/pretrained_model
```

(B601-RS: `--robot.type=seeed_b601_rs_follower --robot.port=can0 --robot.can_adapter=socketcan`; SO-ARM101: `--robot.type=so101_follower`.)

Checklist: camera keys and `single_task` must match the recording session; `--policy.path` accepts a local checkpoint or a Hub repo; keep `eval_`-prefixed dataset names.

---

## Pi0-FAST vs Pi0: When Tokenization Wins

| Situation | Prefer |
| :--- | :--- |
| Small/medium dataset, moderate dexterity | Either — comparable results |
| Large data, want fast convergence | **π0-FAST** (reaches strong performance sooner) |
| High control rate (50 Hz) with pointwise-style pipeline | **π0-FAST** (pointwise fails; FAST compresses) |
| You need lowest-latency chunk generation | π0 (parallel flow denoising beats sequential token generation) |
| You want subtask hierarchy / open-world generalization | [π0.5](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md) |

Run all three on the same dataset and compare: [π0](../5.How_to_Run_Pi0_on_reBot_Arm/README.md) → **π0-FAST** (this tutorial) → [π0.5](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md).

---

## FAQ

**`Could not connect on port /dev/ttyUSB0 or /dev/ttyACM0`** — `sudo chmod 666 /dev/ttyUSB* /dev/ttyACM*`.

**`No valid stream found in input file`** — `conda install ffmpeg=7.1.1 -c conda-forge`.

**Out of memory** — lower `--batch_size`; enable `--policy.gradient_checkpointing=true`.

**Which LeRobot repo?** — the verified fork `https://github.com/Seeed-Projects/lerobot.git`; π0-FAST shares the `pi` extra with π0/π0.5.

**No local GPU?** — cloud training via the [LeRobot pi0-FAST docs](https://huggingface.co/docs/lerobot/pi0fast), or the no-code [SenseCraft Robotics](../2.SenseCraft_Robotics_No_Code_Guide/README.md) platform.

---

## References

- FAST paper: [FAST: Efficient Action Tokenization for Vision-Language-Action Models](https://arxiv.org/abs/24104959) (Physical Intelligence)
- LeRobot: [repository](https://github.com/huggingface/lerobot) · [Pi0-FAST docs](https://huggingface.co/docs/lerobot/pi0fast) · [pretrained pi0fast_base](https://huggingface.co/lerobot/pi0fast_base)
- Seeed deep-dive: [π0-FAST — Efficient Action Tokenization](https://wiki.seeedstudio.com/robotics/paper_reading_pi0_fast/)
- Source wiki tutorial: [reBot Arm B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/)
- Series: [Pi0.5](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md) · [Pi0](../5.How_to_Run_Pi0_on_reBot_Arm/README.md) · [ACT](../3.How_to_Run_ACT_on_reBot_Arm/README.md) · [SmolVLA](../4.How_to_Run_SmolVLA_on_reBot_Arm/README.md)

---

## Tech Support & Product Discussion

<div align="center">
  <a href="https://forum.seeedstudio.com/" target="_blank"><strong>Seeed Forum</strong></a> ·
  <a href="https://www.seeedstudio.com/contacts" target="_blank"><strong>Contact Us</strong></a> ·
  <a href="https://discord.gg/eWkprNDMU7" target="_blank"><strong>Discord</strong></a> ·
  <a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" target="_blank"><strong>GitHub Discussion</strong></a>
</div>
