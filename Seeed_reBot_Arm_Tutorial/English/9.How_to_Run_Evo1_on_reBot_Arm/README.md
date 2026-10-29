# How to Run Evo-1 on reBot Arm — Theory, Data, and Fine-Tuning Guide

<p align="center">
    <img width="600" src="https://files.seeedstudio.com/wiki/robotics/projects/rebot_arm/rebot_dm.png" alt="reBot Arm B601-DM" />
</p>

<p align="center">
    <a href="https://github.com/Seeed-Projects/reBot-DevArm/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT" /></a>
    <img src="https://img.shields.io/badge/Model-Evo--1%200.77B-brightgreen.svg" alt="Model: Evo-1" />
    <img src="https://img.shields.io/badge/Code%20%26%20weights-open%20source-blue.svg" alt="Open source" />
</p>

**[Evo-1](https://arxiv.org/abs/2511.04555)** ([github.com/MINT-SJTU/Evo-1](https://github.com/MINT-SJTU/Evo-1)) is a **lightweight vision-language-action model (770M parameters) that requires no robot-data pre-training** — it starts from the natively multimodal InternVL3 backbone and uses a **two-stage freeze-then-unfreeze recipe** that preserves the VLM's semantic representations while attaching a flow-matching action expert.

Why it matters for low-cost arms: at 0.77B parameters it reports **16.4 Hz inference in 2.3 GB VRAM** on a consumer GPU (RTX 4090-class) with **78% average real-task success** — outperforming compared baselines including π0 (73%, 17.9 GB) and SmolVLA (50%, 2.0 GB) in the paper's real-world comparison — and it records data in **LeRobot format**, the same pipeline you already use on a reBot Arm.

Part of the [Seeed reBot Arm VLA tutorial series](../README.md) · [Seeed AI Robotics Academy](../../../README.md).

---

## Table of Contents

- [Which Robotic Arms Are Compatible with Evo-1?](#which-robotic-arms-are-compatible-with-evo-1)
- [Evo-1 Theory Deep-Dive](#evo-1-theory-deep-dive)
- [Running the Evo-1 Workflow on reBot Arm](#running-the-evo-1-workflow-on-rebot-arm)
- [FAQ](#faq)
- [References](#references)

---

## Which Robotic Arms Are Compatible with Evo-1?

Evo-1's real-world system: a **six-DoF xArm6 with parallel gripper, two RGB cameras (fixed + wrist), 30 Hz recording, absolute joint-angle state and actions, padded to a 24-dimensional model interface**. That is structurally identical to a reBot/SO-101 leader-follower setup, and its data format is LeRobot:

| Robotic arm | Interface | Compatibility with Evo-1's setup | Buy |
| :--- | :--- | :--- | :--- |
| **reBot Arm B601-DM** | 6-DoF + gripper, USB-CAN, 2× RGB @ 30 fps | ✅ Same DoF/camera/joint-space structure; LeRobot dataset | [reBot Arm B601-DM Bundle](https://www.seeedstudio.com/reBot-Arm-B601-DM-Bundle.html) |
| **reBot Arm B601-RS** | 6-DoF + gripper, SocketCAN | ✅ Same structure | [reBot Arm B601-RS Kit](https://www.seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html) |
| **SO-ARM101 / SO-101** | 6-DoF + gripper, USB serial | ✅ Same structure; SmolVLA-class community arms | [SO-101 Low-Cost AI Arm Kit Pro](https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html) |

The 24-dimensional action interface is padding, not joint count — each platform's effective dimensions are defined by its dataset. Fine-tuning code, data, and weights are released at [github.com/MINT-SJTU/Evo-1](https://github.com/MINT-SJTU/Evo-1).

---

## Evo-1 Theory Deep-Dive

*Condensed from the Seeed paper-reading deep-dive: [Evo-1 — Lightweight VLA](https://wiki.seeedstudio.com/robotics/paper_reading_evo1/).*

### Architecture: Compact Backbone + Persistent Context + Flow-Matching Expert

| Path | Input → Processing |
| :--- | :--- |
| Vision | N RGB views → 448×448 → InternViT-300M features → pixel-unshuffle (¼ tokens) |
| Language | Instruction → first 14 layers of the Qwen2.5-0.5B branch → layer-14 fused image-text features |
| State | Proprioception padded to 24 dims → state encoding → conditioning context |
| Action | Noisy 50×24 chunk → 8-layer Transformer expert → generation updates |

**Integration (the key idea):** every action layer reads the *same* conditioning context (layer-14 image-text features + state concatenated as K/V, noisy actions as Q) via **stacked mid-layer cross-attention** — consistent conditions across layers, chosen over interleaved self-attention, layer-wise, or joint-KV designs in ablation.

**Action generation:** flow matching on a 50×24 chunk — training interpolates `X_τ = τA + (1−τ)ε` with τ sampled from a Beta distribution on [0.02, 0.98]; inference repeatedly predicts direction updates from fresh noise under image-text and state conditioning.

### Two-Stage Training: Adapt New Modules to Existing Features First

| Stage | VLM backbone | Action expert + integration | Purpose |
| :--- | :--- | :--- | :--- |
| **1 — Action-expert alignment** | Fully frozen | Trainable | Learn to use existing perceptual features |
| **2 — Full-model fine-tuning** | Unfrozen | Continue | Jointly adapt downstream control |

No explicit semantic-preservation loss — semantic retention comes from initialization and training order. Training configuration (Meta-World): 10k stage-1 + 65k stage-2 steps, lr 1e-5, batch 16, on 8× A100.

### Results

| Benchmark | Evo-1 | Best comparison |
| :--- | :--- | :--- |
| Meta-World (50 tasks) | **80.6%** | SmolVLA 68.2% (+12.4 pp) |
| RoboTwin 2.0 (4 tasks) | **37.8%** | π0 30.9% (+6.9 pp) |
| LIBERO (40 tasks) | 94.8% | π0 94.2% |
| Real xArm6 (4 tasks, mean) | **78%** | π0 73%, SmolVLA 50% |
| Inference (RTX 4090-class) | **16.4 Hz / 2.3 GB** | SmolVLA 12.7 Hz / 2.0 GB; π0 11.5 Hz / 17.9 GB |

Real-world detail: pick-and-place 95%, pouring 70%, handover 95%, stacking 50% — generalization tests (distractors, ±30 mm target shifts) stay at 70–100%. **Boundaries:** averages hide uneven categories (LIBERO Spatial trails π0; RoboTwin hard settings remain weak); 16.4 Hz is GPU inference frequency, not full control-loop rate; baselines were not all retrained uniformly.

📖 Full deep-dive: [Evo-1 — Lightweight VLA (Seeed Paper Reading)](https://wiki.seeedstudio.com/robotics/paper_reading_evo1/).

---

## Running the Evo-1 Workflow on reBot Arm

Evo-1 consumes **LeRobot-format demonstrations** — exactly what the reBot pipeline produces:

1. **Set up and calibrate** your B601-DM / B601-RS / SO-ARM101 — [Pi0.5 tutorial, Steps 0–2](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-by-step-practice).
2. **Two RGB views at 30 fps** (fixed + wrist or front + side) — matching Evo-1's real-system setup; the [Camera Mount](https://www.seeedstudio.com/Robot-Arm-Camera-Mount-for-LeRobot-SO-10&reBot-B601-p-7003.html) holds them stable.
3. **Record demonstrations** with `lerobot-record` ([Step 4](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-4-record-a-dataset)); absolute joint-angle state/actions are the LeRobot default. The paper's real-world scale: **100 teleoperated episodes per task, 20 evaluation trials per task**.
4. **Fine-tune with the official code** — [github.com/MINT-SJTU/Evo-1](https://github.com/MINT-SJTU/Evo-1) releases code, data, and weights; follow its two-stage configuration (freeze-then-unfreeze). Scale the stage steps to your dataset size as the paper does per benchmark.
5. **Deploy** the 50-step action chunks through your platform's control interface — the same `lerobot-record`-style evaluation loop used for [every policy in this series](../README.md).

**Compute note:** 2.3 GB inference VRAM means a reBot Arm + Evo-1 can run on a single consumer-GPU machine; training in the paper used 8× A100 — reduce scale for desktop fine-tuning.

---

## FAQ

**How is Evo-1 different from SmolVLA?** Both are sub-1B VLAs, but SmolVLA pre-trains on community robot data, while Evo-1 skips robot pre-training entirely (natively multimodal InternVL3 backbone + two-stage training), and uses stacked cross-attention integration rather than SmolVLA's half-backbone + alternating attention.

**Can I run Evo-1 today on a reBot Arm?** Yes — code, data, and weights are open ([github.com/MINT-SJTU/Evo-1](https://github.com/MINT-SJTU/Evo-1)), and its data format is LeRobot. For a one-flag LeRobot experience instead, use the built-in policies: [SmolVLA](../4.How_to_Run_SmolVLA_on_reBot_Arm/README.md) / [Pi0.5](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md).

**How much data?** The paper's real-robot tasks used 100 demonstrations per task; simulation benchmarks used 50 per task.

---

## References

- Evo-1 paper: [Evo-1: Lightweight Vision-Language-Action Model with Preserved Semantic Alignment](https://arxiv.org/abs/2511.04555) · [code & weights](https://github.com/MINT-SJTU/Evo-1)
- Seeed deep-dive: [Evo-1 — Lightweight VLA](https://wiki.seeedstudio.com/robotics/paper_reading_evo1/)
- Data pipeline: [reBot Arm B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/) (source wiki tutorial)
- Series: [Pi0.5](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md) · [ACT](../3.How_to_Run_ACT_on_reBot_Arm/README.md) · [SmolVLA](../4.How_to_Run_SmolVLA_on_reBot_Arm/README.md) · [Pi0](../5.How_to_Run_Pi0_on_reBot_Arm/README.md) · [Pi0-FAST](../6.How_to_Run_Pi0_FAST_on_reBot_Arm/README.md) · [GR00T N1.5](../7.How_to_Run_GR00T_N1.5_on_reBot_Arm/README.md) · [EO-1](../8.How_to_Run_EO1_on_reBot_Arm/README.md)

---

## Tech Support & Product Discussion

<div align="center">
  <a href="https://forum.seeedstudio.com/" target="_blank"><strong>Seeed Forum</strong></a> ·
  <a href="https://www.seeedstudio.com/contacts" target="_blank"><strong>Contact Us</strong></a> ·
  <a href="https://discord.gg/eWkprNDMU7" target="_blank"><strong>Discord</strong></a> ·
  <a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" target="_blank"><strong>GitHub Discussion</strong></a>
</div>
