# How to Run WALL-OSS on reBot Arm — Theory, Data, and Fine-Tuning Guide

<p align="center">
    <img width="600" src="https://files.seeedstudio.com/wiki/robotics/projects/rebot_arm/rebot_dm.png" alt="reBot Arm B601-DM" />
</p>

<p align="center">
    <a href="https://github.com/Seeed-Projects/reBot-DevArm/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT" /></a>
    <img src="https://img.shields.io/badge/Model-WALL--OSS%20Uni--CoT-9cf.svg" alt="Model: WALL-OSS" />
    <img src="https://img.shields.io/badge/Backbone-Qwen2.5--VL--3B-6e7bff.svg" alt="Backbone" />
</p>

**WALL-OSS** ("[Igniting VLMs toward the Embodied Space](https://arxiv.org/abs/2509.11766)") is an **end-to-end embodied foundation model** that takes a Qwen2.5-VL-3B backbone and adds: **(1)** vision-language understanding sensitive to embodied features, **(2)** strong language–action association, and **(3)** robust manipulation — unified by **Uni-CoT**, one differentiable framework that links instruction reasoning, subgoal decomposition, and fine-grained action synthesis.

Its signature capability is **long-horizon manipulation with reasoning**: table setting and bedroom tidying that exceed five stages and run 3–5+ minutes, where baselines repeat utensils, miss the second place setting, or loop when objects leave view — while WALL-OSS generates subtasks before acting and tracks progress.

> **Integration status:** WALL-OSS is a research foundation model (the paper announces open training code and checkpoints; [arXiv:2509.11766](https://arxiv.org/abs/2509.11766)). It is not a one-flag LeRobot policy type like [`pi05`](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md). The workflow here: understand the model → record reBot demonstrations → fine-tune following the official release.

Part of the [Seeed reBot Arm VLA tutorial series](../README.md) · [Seeed AI Robotics Academy](../../../README.md).

---

## Table of Contents

- [Which Robots Does WALL-OSS Support?](#which-robots-does-wall-oss-support)
- [WALL-OSS Theory Deep-Dive](#wall-oss-theory-deep-dive)
- [Preparing reBot Arm Data for WALL-OSS-Style Long-Horizon Tasks](#preparing-rebot-arm-data-for-wall-oss-style-long-horizon-tasks)
- [FAQ](#faq)
- [References](#references)

---

## Which Robots Does WALL-OSS Support?

The training corpus spans **tabletop arms, mobile mounts, wheeled bimanual, and humanoid robots** with ego/external/arm views — cross-platform alignment uses meters for positions, radians for angles, and a maximum-DoF template with masks for missing joints:

| Your arm | Fit with WALL-OSS's setup |
| :--- | :--- |
| **[reBot Arm B601-DM](https://www.seeedstudio.com/reBot-Arm-B601-DM-Bundle.html)** / **[B601-RS](https://www.seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html)** | Tabletop arm, 6-DoF + gripper, 2 RGB views — matches the tabletop-arm platform family |
| **[SO-ARM101 / SO-101](https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html)** | Same tabletop structure; open robot-action data (DROID, BC-Z, BridgeData V2) covers comparable low-cost arms |

The paper's fine-tuning demonstration scale for long-horizon tasks: **500–1,600 episodes per task** (Set-Table 1,500; Tidy-Bedroom 1,000; Block-Spell 1,600; Collect-Waste 900–1,000).

---

## WALL-OSS Theory Deep-Dive

*Condensed from the Seeed paper-reading deep-dive: [WALL-OSS — Embodied Reasoning & Robot Control](https://wiki.seeedstudio.com/robotics/paper_reading_wall_oss/).*

### Three Gaps from VLM to Embodied

1. **Modality/data-scale gap** — actions are continuous spatiotemporal signals; instructions omit subtasks.
2. **Distribution gap** — ego views, fisheye, self-occlusion differ from internet imagery.
3. **Objective gap** — text is discrete next-token; actions suit diffusion/flow — naively joining objectives weakly couples semantics and control.

### Architecture: Shared Attention, Separate Feature Processing

A **statically routed mixture-of-experts** design on the Qwen2.5-VL-3B backbone: self-attention exchanges information across modalities, while **visual-language features go through a VL FFN and action features through a separate action FFN** — tight coupling without full parameter sharing. Outputs branch into an LM head (reasoning/subtask text), discrete FAST action tokens (during the discrete stage), and a flow head producing a T × d velocity field that integrates into the executed trajectory.

### Two Training Stages

| Stage | Supervision | Purpose |
| :--- | :--- | :--- |
| **Inspiration** | Embodied VQA, text/subtasks/CoT, **FAST discrete action tokens** | Robot-scene understanding and action semantics as sequence prediction |
| **Integration phase 1** | Continuous **flow matching** (VLM frozen) | Let the new action branch use established priors |
| **Integration phase 2** | Flow matching + joint optimization (VLM unfrozen) | Align vision, language, and actions |

FAST tokenization here follows the [DCT → quantize → BPE](../6.How_to_Run_Pi0_FAST_on_reBot_Arm/README.md) recipe — trajectories compressed into discrete tokens to build priors; deployment then uses continuous generation.

### Uni-CoT: Reason When Needed, Act Directly When Not

One model learns **cross-level mappings** (instruction → reasoning → subgoals → actions) with **path-drop**: some training samples include intermediate reasoning, others go straight from observation to action — so inference can use full CoT for hard tasks or direct execution for simple ones. Long-horizon fine-tuning generates subtasks before actions.

### Data

| Source | Share | Content |
| :--- | :--- | :--- |
| Internal robot actions | 57.5% | Everyday manipulation, complex trajectories, process labels (kitchen, clothing/storage, mobile pick-and-place, assembly) |
| Open robot actions | 33.1% | DROID, BC-Z, BridgeData V2, and others |
| Multimodal VQA | 9.4% | General QA + embodied QA (space, time, progress, affordances) |

Total reported as over 10,000 hours of robot data (with some internal inconsistency in the paper's totals).

### What the Experiments Show

- **Embodied VQA:** object grounding 46.1% → **91.6%** (+45.5 pp over Qwen2.5-VL-3B); scene captioning +29.9 pp; action planning +9.2 pp.
- **Zero-shot instruction following:** 85% progress (seen objects/containers), 61% (new objects) — *progress scores, not full-success rates*.
- **Long-horizon:** Set-Table / Tidy-Bedroom exceed five stages with 3–5+ minute durations; WALL-OSS beats π0 and Diffusion Policy flat/GPT-subtask baselines, which fail by repeating utensils, missing the second setting, or looping when clothes leave view.
- **Evaluation:** blind third-party scoring with environment/starting-state documents; trial counts and error bars not fully published.

**Boundaries:** fine-tuned tasks are not zero-shot (unseen in pre-training ≠ unseen in fine-tuning); the combined loss establishes shared multi-objective learning, not gradients through every generated word; v1 omits many training hyperparameters.

📖 Full deep-dive: [WALL-OSS — Embodied Reasoning & Robot Control (Seeed Paper Reading)](https://wiki.seeedstudio.com/robotics/paper_reading_wall_oss/).

---

## Preparing reBot Arm Data for WALL-OSS-Style Long-Horizon Tasks

1. **Set up and calibrate** the arm — [Pi0.5 tutorial, Steps 0–3](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-by-step-practice) (B601-DM / B601-RS / SO-ARM101).
2. **Two+ RGB views** — WALL-OSS uses ego/external/arm views; the reBot standard front + side (or top + wrist) layout with the [Camera Mount](https://www.seeedstudio.com/Robot-Arm-Camera-Mount-for-LeRobot-SO-10&reBot-B601-p-7003.html) is the tabletop-arm equivalent.
3. **Record demonstrations** with `lerobot-record` ([Step 4](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-4-record-a-dataset)).
4. **Long-horizon structure** — the distinguishing part: record multi-stage tasks (e.g. "set the table for two" = plate → knife → fork × 2) with:
   - **Process labels / subtask annotations** for some episodes (WALL-OSS's internal data mixes explicit-instruction precision tasks and implicit-step long tasks).
   - Task descriptions at both levels — overall goal ("tidy the bedroom") and subtasks ("collect the clothes").
   - Scale: 500–1,600 episodes per long-horizon task in the paper; start smaller and iterate.
5. **Fine-tune following the official release** — [arXiv:2509.11766](https://arxiv.org/abs/2509.11766); the announced open training code and checkpoints carry the two-stage Inspiration/Integration recipe.

---

## FAQ

**How is WALL-OSS different from π0.5 and EO-1?** All three unify reasoning and acting in one model, but WALL-OSS's route is **discrete-first** (FAST action tokens learned in the language space during Inspiration) then flow-matching integration, with Uni-CoT path-drop; π0.5 uses subtask selection; EO-1 uses interleaved QA-action training with rectifying sampling.

**Can I run WALL-OSS today on a reBot Arm?** The paper announces open training code and checkpoints — follow the [arXiv page](https://arxiv.org/abs/2509.11766) for the release. For turnkey LeRobot fine-tuning today, use [Pi0.5](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md) or [SmolVLA](../4.How_to_Run_SmolVLA_on_reBot_Arm/README.md).

**How much data?** 500–1,600 episodes per long-horizon task (far more than single-stage policies) — but note only 1% of the paper's long-task training data was subtask-labeled, so heavy annotation is not required.

---

## References

- WALL-OSS paper: [Igniting VLMs toward the Embodied Space](https://arxiv.org/abs/2509.11766)
- Seeed deep-dive: [WALL-OSS — Embodied Reasoning & Robot Control](https://wiki.seeedstudio.com/robotics/paper_reading_wall_oss/)
- Data pipeline: [reBot Arm B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/) (source wiki tutorial)
- Related in series: [FAST tokenization (π0-FAST)](../6.How_to_Run_Pi0_FAST_on_reBot_Arm/README.md) · [Pi0.5](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md) · [EO-1](../8.How_to_Run_EO1_on_reBot_Arm/README.md) · [Evo-1](../9.How_to_Run_Evo1_on_reBot_Arm/README.md)

---

## Tech Support & Product Discussion

<div align="center">
  <a href="https://forum.seeedstudio.com/" target="_blank"><strong>Seeed Forum</strong></a> ·
  <a href="https://www.seeedstudio.com/contacts" target="_blank"><strong>Contact Us</strong></a> ·
  <a href="https://discord.gg/eWkprNDMU7" target="_blank"><strong>Discord</strong></a> ·
  <a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" target="_blank"><strong>GitHub Discussion</strong></a>
</div>
