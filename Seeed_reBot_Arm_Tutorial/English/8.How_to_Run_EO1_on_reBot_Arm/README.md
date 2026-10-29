# How to Run EO-1 on reBot Arm — Theory, Data, and Fine-Tuning Guide

<p align="center">
    <img width="600" src="https://files.seeedstudio.com/wiki/robotics/projects/rebot_arm/rebot_dm.png" alt="reBot Arm B601-DM" />
</p>

<p align="center">
    <a href="https://github.com/Seeed-Projects/reBot-DevArm/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT" /></a>
    <img src="https://img.shields.io/badge/Model-EO--1%20unified%20embodied-orange.svg" alt="Model: EO-1" />
    <img src="https://img.shields.io/badge/Data-EO--Data1.5M-yellow.svg" alt="Data: EO-Data1.5M" />
</p>

**EO-1** ([arXiv:2508.21112](https://arxiv.org/abs/2508.21112), [project page](https://eo-robotics.ai/eo-1)) is an **open unified embodied foundation model for general robot control** from EO-Robotics. Unlike a pure control policy, EO-1 learns **embodied reasoning and control together** through **interleaved vision-text-action pre-training** on **EO-Data1.5M** — it can act, look again, judge whether the action succeeded, and re-plan, all in one decoder-only backbone.

Why this matters for your arm: a VLA that can answer "is it finished?" and "what next?" *before and after acting* closes the loop that pure-chunk policies leave open. This tutorial covers EO-1's architecture and training recipe, what its experiments actually demonstrate, and how to prepare **reBot Arm** data in the right shape for unified embodied models.

> **Integration status:** EO-1 is a research foundation model with an open project page — it is not (yet) a one-flag LeRobot policy type like [`pi05`](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md) or [`act`](../3.How_to_Run_ACT_on_reBot_Arm/README.md). The workflow here is: understand the model → record your reBot demonstrations → fine-tune following the official release.

> 📚 **Want the full tutorial and more resources?** This GitHub guide is a companion to the complete documentation on the **[Seeed Studio Wiki](https://wiki.seeedstudio.com/robotics_page/)**, which is continuously updated and includes video walkthroughs, hardware assembly, and troubleshooting:
> - **EO-1 theory deep-dive:** [EO-1 — Interleaved Multimodal Pre-Training](https://wiki.seeedstudio.com/robotics/paper_reading_eo1/)
> - **Data-collection pipeline (wiki):** [reBot Arm B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/) · [reBot Arm B601-RS with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_rs_lerobot/) · [SO-ARM101 with LeRobot](https://wiki.seeedstudio.com/lerobot_so100m_new/)
> - **Hardware setup:** [reBot Arm B601-DM Getting Started](https://wiki.seeedstudio.com/rebot_b601_dm_getting_started/) · [reBot Arm B601-RS Getting Started](https://wiki.seeedstudio.com/rebot_b601_rs_getting_started/)

Part of the [Seeed reBot Arm VLA tutorial series](../README.md) · [Seeed AI Robotics Academy](../../../README.md).

---

## Table of Contents

- [Which Robots Does EO-1 Support?](#which-robots-does-eo-1-support)
- [EO-1 Theory Deep-Dive](#eo-1-theory-deep-dive)
- [Preparing reBot Arm Data for Unified Embodied Models](#preparing-rebot-arm-data-for-unified-embodied-models)
- [FAQ](#faq)
- [References](#references)

---

## Which Robots Does EO-1 Support?

EO-1's evaluation spans **28 real-robot task settings** across four embodiment families:

| Platform | Role in the paper |
| :--- | :--- |
| Franka Panda | 7 pick-and-place task settings (IPEC-Franka third-person + wrist views, 10 FPS) |
| WidowX 250 | 13 "out-of-the-box" task settings (DROID left/right/wrist views, 15 FPS) |
| AgiBot G-1 | 4 long-horizon dexterous tasks (ego/left/right views, 30 FPS) |
| **SO100 / SO-ARM class arms** | Training-data configurations with third-person + wrist cameras at 30 FPS |

**SO-ARM-class hardware with 2 RGB views at ~30 Hz is squarely inside EO-1's training distribution** — exactly the configuration of the [reBot Arm B601-DM / B601-RS](https://www.seeedstudio.com/reBot-Arm-B601-DM-Bundle.html) and [SO-ARM101](https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html) kits (two cameras, 30 fps, leader-follower teleoperation). The fine-tuning recipe used for long-horizon tasks: **150 demonstrations per task**, 25 epochs, subtask and full-task data mixed 9:1.

---

## EO-1 Theory Deep-Dive

*Condensed from the Seeed paper-reading deep-dive: [EO-1 — Interleaved Multimodal Pre-Training](https://wiki.seeedstudio.com/robotics/paper_reading_eo1/).*

### One Backbone, Two Outputs

A decoder-only Transformer initialized from **Qwen2.5-VL** shares processing between reasoning and control:

| Raw input | Encoding | Representation |
| :--- | :--- | :--- |
| Multi-view images / video | Pre-trained visual encoder | Visual token sequence |
| Instructions, reasoning QA | Tokenizer + embedding | Text token sequence |
| Robot state `q_t` | State projection | State tokens |
| Noisy chunk `X_τ` + progress `τ` | Action-input projection | Action tokens |

Two output branches: an **LM head** (text tokens, autoregressive) and a **flow head** (continuous action updates). The final chunk is **16 × d_a** (16 control points, platform-specific action dimension).

### Text Sequentially, Actions as a Chunk

- **Text** — next-token prediction; the model can reason about what it sees and what it just did.
- **Actions** — flow matching: train to predict the update field that denoises `X_τ = τA + (1−τ)Z` (real action A, Gaussian noise Z) toward the demonstrated chunk; at inference, ten updates from pure noise produce the executable chunk. Both loss terms have weight one.

### Interleaved Sequences: Reason Again After Acting

```text
Standard VLA sample:
  Image → Task text → State → Action chunk

EO-1 interleaved sample:
  Current image → Planning QA → State → Action chunk
  → New image → Completion QA → Next-step planning → Next chunk
```

After attempting to bag a tomato, a new image may show it still in the gripper — the model judges the placement incomplete and generates more actions. Each modality has boundary markers (`[BOI]/[EOI]`, `[BOR]/[EOR]`, `[BOA]/[EOA]`), and causal attention conditions only on the past.

### Interleaved Rectifying Sampling

Training the current action requires noise, but later text should condition on *completed* actions. EO-1 splits a sequence with N action segments into **N+1 training prefixes at action boundaries** — historical actions stay clean, only the action currently being generated is noisy.

### EO-Data1.5M

Over 1.5 million embodied-reasoning samples overall (≈122,000 are full interleaved vision-text-action examples), built by preserving diverse scenes and then adding reasoning annotations. Three data groups connect question answering and actions, emphasizing *relevant* interleaved data.

### What the Experiments Show

| Evaluation | EO-1 | Comparison |
| :--- | :--- | :--- |
| LIBERO mean success | **98.2 ± 0.3%** | π0: 94.2%, OpenVLA-OFT: 97.1% |
| SimplerEnv (WidowX/Google VM/VA) | 72.7 / 76.5 / 63.0% | π0: 69.2 / 71.4 / 54.7% |
| ERQA accuracy | 45.5% | Qwen2.5-VL 3B: 35.3%; Gemini 1.5 Flash: 46.3% |
| EO-Bench spatial/temporal | 36.4 / 38.9 | Qwen2.5-VL 3B: 20.0 / 22.6 |
| Tic-tac-toe reasoning-control | 0.76 | π0: 0.24; GR00T-N1.5: 0.36 |

Ablation insights: continuous flow generation beats fully-autoregressive FAST actions in this setting (LIBERO 98.2% vs 88.0%), and **relevant** interleaved data lifts WidowX completion from 0.71 to 0.80 while *generic* image-text data (0.34) hurts — task alignment matters.

**Boundaries:** metrics are not interchangeable (QA scores ≠ robot capability); long-horizon scores average subtask-instruction completion; gains are bounded by the demonstrated tasks and data.

📖 Full deep-dive: [EO-1 — Interleaved Multimodal Pre-Training (Seeed Paper Reading)](https://wiki.seeedstudio.com/robotics/paper_reading_eo1/).

---

## Preparing reBot Arm Data for Unified Embodied Models

The demonstrations that fine-tune a unified model like EO-1 come from the same leader-follower pipeline as every other tutorial in this series:

1. **Set up the arm** — [environment, calibration, teleoperation](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-by-step-practice) for B601-DM / B601-RS / SO-ARM101.
2. **Two camera views at 30 fps** (`front` + `side` or top + wrist) — matches the SO100-style configuration in EO-1's training data. [Camera Mount](https://www.seeedstudio.com/Robot-Arm-Camera-Mount-for-LeRobot-SO-10&reBot-B601-p-7003.html) keeps positions fixed.
3. **Record with `lerobot-record`** — [Step 4 of the Pi0.5 tutorial](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-4-record-a-dataset); aim for the paper's fine-tuning scale (~150 episodes per task) with consistent task descriptions.
4. **Subtask structure** — long-horizon fine-tuning mixes subtask and full-task demonstrations (9:1 in the paper): record some episodes labeled by subtask ("pick up the cube"), others by full task ("sort all cubes into the box").
5. **Fine-tune following the official release** — [project page](https://eo-robotics.ai/eo-1) / [arXiv:2508.21112](https://arxiv.org/abs/2508.21112); the interleaved format (image → QA → state → action) is what distinguishes EO-1 data preparation.

---

## FAQ

**How is EO-1 different from π0.5?** π0.5 also selects subtasks within one model, but EO-1 trains *interleaved reasoning and acting* (planning QA, completion QA around action chunks) on its EO-Data1.5M corpus, and reports reasoning benchmarks alongside control scores.

**Can I run EO-1 today on a reBot Arm?** The architecture, data construction, and training methods are published as a reference for embodied foundation models; follow the [project page](https://eo-robotics.ai/eo-1) for the open release. For a turnkey fine-tune today, use the LeRobot policies in this series ([Pi0.5](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md), [Pi0](../5.How_to_Run_Pi0_on_reBot_Arm/README.md), [SmolVLA](../4.How_to_Run_SmolVLA_on_reBot_Arm/README.md), [ACT](../3.How_to_Run_ACT_on_reBot_Arm/README.md)).

**How much data?** The paper's long-horizon fine-tuning used 150 demonstrations per task for 25 epochs — more than the ~50 episodes typical for LeRobot policy fine-tuning, because unified models learn the reasoning structure as well as the motion.

---

## References

- EO-1 paper: [EO-1: An Open Unified Embodied Foundation Model for General Robot Control](https://arxiv.org/abs/2508.21112) · [project page](https://eo-robotics.ai/eo-1)
- Seeed deep-dive: [EO-1 — Interleaved Multimodal Pre-Training](https://wiki.seeedstudio.com/robotics/paper_reading_eo1/)
- Data pipeline: [reBot Arm B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/) (source wiki tutorial)
- Series: [Pi0.5](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md) · [ACT](../3.How_to_Run_ACT_on_reBot_Arm/README.md) · [SmolVLA](../4.How_to_Run_SmolVLA_on_reBot_Arm/README.md) · [Pi0](../5.How_to_Run_Pi0_on_reBot_Arm/README.md) · [Pi0-FAST](../6.How_to_Run_Pi0_FAST_on_reBot_Arm/README.md) · [GR00T N1.5](../7.How_to_Run_GR00T_N1.5_on_reBot_Arm/README.md)

---

## Tech Support & Product Discussion

<div align="center">
  <a href="https://forum.seeedstudio.com/" target="_blank"><strong>Seeed Forum</strong></a> ·
  <a href="https://www.seeedstudio.com/contacts" target="_blank"><strong>Contact Us</strong></a> ·
  <a href="https://discord.gg/eWkprNDMU7" target="_blank"><strong>Discord</strong></a> ·
  <a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" target="_blank"><strong>GitHub Discussion</strong></a>
</div>
