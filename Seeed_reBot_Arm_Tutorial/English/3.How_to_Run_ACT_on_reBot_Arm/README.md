# How to Run ACT on reBot Arm — Theory, Setup, Training & Deployment

<p align="center">
    <img width="600" src="https://files.seeedstudio.com/wiki/robotics/projects/rebot_arm/rebot_dm.png" alt="reBot Arm B601-DM" />
</p>

<p align="center">
    <a href="https://github.com/Seeed-Projects/reBot-DevArm/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT" /></a>
    <img src="https://img.shields.io/badge/Python-3.12+-blue.svg" alt="Python Version" />
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Ubuntu%20%7C%20Jetson%20Orin-orange.svg" alt="Platform" />
    <img src="https://img.shields.io/badge/Policy-ACT%20action%20chunking-teal.svg" alt="Policy: ACT" />
</p>

**ACT (Action Chunking with Transformers)** is the imitation-learning algorithm from [Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware](https://arxiv.org/abs/2304.13705) (ALOHA, Stanford/Berkeley/Meta, 2023). With just ~10 minutes (≈50 trajectories) of human demonstrations, an ALOHA system trained with ACT performs six fine-manipulation tasks — opening condiment cups, inserting batteries — at **80–90% success**.

ACT is the **best starting point for learning robot imitation learning on real hardware**: it is lightweight (trainable on an RTX 3060), needs no language model, and its LeRobot policy type `act` adapts automatically to your robot. This tutorial runs it on the **reBot Arm B601-DM / B601-RS** and **SO-ARM101** leader-follower kits.

> 📚 **Want the full tutorial and more resources?** This GitHub guide is a companion to the complete documentation on the **[Seeed Studio Wiki](https://wiki.seeedstudio.com/robotics_page/)**, which is continuously updated and includes video walkthroughs, hardware assembly, and troubleshooting:
> - **Full wiki tutorials:** [reBot Arm B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/) · [reBot Arm B601-RS with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_rs_lerobot/) · [SO-ARM101 with LeRobot](https://wiki.seeedstudio.com/lerobot_so100m_new/)
> - **ACT theory deep-dive:** [ACT — Low-Cost Bimanual Manipulation](https://wiki.seeedstudio.com/robotics/paper_reading_act/)
> - **Hands-on course:** [ACT Model Training](https://wiki.seeedstudio.com/robotics/hands_on_act_training/) · [ACT On-Robot Deployment](https://wiki.seeedstudio.com/robotics/hands_on_act_deployment/)
> - **Hardware setup:** [reBot Arm B601-DM Getting Started](https://wiki.seeedstudio.com/rebot_b601_dm_getting_started/) · [reBot Arm B601-RS Getting Started](https://wiki.seeedstudio.com/rebot_b601_rs_getting_started/)

Part of the [Seeed reBot Arm VLA tutorial series](../README.md) · [Seeed AI Robotics Academy](../../../README.md). If this guide helps you, please ⭐ star [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm/).

---

## Table of Contents

- [Which Robotic Arms Are Compatible with ACT?](#which-robotic-arms-are-compatible-with-act)
- [ACT Theory Deep-Dive](#act-theory-deep-dive)
- [Hardware, Environment, and Data Collection](#hardware-environment-and-data-collection)
- [Train ACT](#train-act)
- [Evaluate on the Real Arm](#evaluate-on-the-real-arm)
- [ACT vs Pi0.5: Which Should You Start With?](#act-vs-pi05-which-should-you-start-with)
- [FAQ](#faq)
- [References](#references)

---

## Which Robotic Arms Are Compatible with ACT?

**ACT works with any robotic arm that has a [LeRobot](https://github.com/huggingface/lerobot) robot class and a recorded demonstration dataset** — it is a pure imitation-learning policy (images + joint state in, joint targets out), so it has no language backbone and no pretrained checkpoint to match dimensions against. Verified on Seeed's open hardware:

| Robotic arm | Actuation | Interface | Status | Buy |
| :--- | :--- | :--- | :--- | :--- |
| **reBot Arm B601-DM** (Seeed) | Damiao DM brushless servos | USB-CAN bridge (`/dev/ttyACM0`) | ✅ Verified, this guide | [reBot Arm B601-DM Bundle](https://www.seeedstudio.com/reBot-Arm-B601-DM-Bundle.html) |
| **reBot Arm B601-RS** (Seeed) | RobStride RS actuators | SocketCAN (`can0`) / PCAN | ✅ Verified | [reBot Arm B601-RS Kit](https://www.seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html) |
| **SO-ARM101 / SO-101** (Seeed × LeRobot) | Feetech serial servos | USB serial (`/dev/ttyACM0`) | ✅ Verified | [SO-101 Low-Cost AI Arm Kit Pro](https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html) |

The original paper's platform was **ALOHA** — two ViperX 6-DoF arms (14 controlled dimensions), four RGB cameras. Any LeRobot-supported arm (SO-100/SO-101, Koch v1.1, ALOHA-style bimanual rigs, mobile manipulators) can train and run ACT; the policy auto-adapts to your robot's motor states, actions, and camera count from the dataset metadata.

> 💡 ACT is also the **default policy in [SenseCraft Robotics](../2.SenseCraft_Robotics_No_Code_Guide/README.md)**, Seeed's no-code platform for these arms (Windows / macOS / Linux).

---

## ACT Theory Deep-Dive

*Condensed from the Seeed paper-reading deep-dive: [ACT — Low-Cost Bimanual Manipulation](https://wiki.seeedstudio.com/robotics/paper_reading_act/). Paper: [arXiv:2304.13705](https://arxiv.org/abs/2304.13705).*

### The Problem: Compounding Errors in Imitation Learning

Imitation learning is simple in principle: demonstrate the task, record camera observations and actions, then train a model that acts from images alone. But the conventional loop — **observe → predict one action → execute → observe again** — has a flaw: each small prediction error moves the arm into a state slightly outside the training distribution, making later predictions less reliable. Errors **compound**, showing up on the robot as jitter, hesitation, and drift.

### Fix 1: Action Chunking — Predict a Sequence at Once

ACT's core idea: instead of `πθ(at|st)` (one action per observation), predict `πθ(at:t+k|st)` — **k future actions in one shot**:

```text
Single-step:  t=0→a0, t=1→a1, t=2→a2, t=3→a3   (4 decisions)
ACT k=4:      t=0→[a0, a1, a2, a3]              (1 decision, whole chunk)
```

Benefits:

- **k× shorter effective horizon** — errors have fewer decision boundaries to compound across.
- **Lower inference-rate requirement** — one prediction covers a short interval, so even modest hardware yields smooth motion.
- **Temporal structure** — the model learns "approach → align → insert" as coherent motion, like a person does, instead of a list of one-millimeter corrections.

### Fix 2: Temporal Ensembling — Blend Overlapping Predictions

Executing a whole chunk blind is risky if the environment changes. ACT instead **observes and predicts a new chunk at every timestep** and blends the overlapping predictions for the current instant with exponential weights (`w_i = exp(-m·i)`):

```text
t=0 → [a0, a1, a2, a3]
t=1 →      [a1', a2', a3', a4']
t=2 →           [a2'', a3'', ...]
```

At `t=2`, the executed action is a weighted combination of `a2`, `a1'→a2'`, `a2''` — smooth, self-correcting motion without abrupt jumps.

### Fix 3: CVAE — Model Multi-Modal Demonstrations

Different demonstrations of the same task take different valid trajectories (approach from the left or the right, faster or slower). A model forced to output "the one correct action" averages across strategies — which can land mid-obstacle. ACT's **conditional VAE** learns *which reasonable action sequences a person might use*:

- **Encoder** (training only): takes robot state + the demonstrated action chunk, outputs Gaussian parameters μ, σ; sampling a latent style variable z represents demonstration variety.
- **Policy** (always): a Transformer that takes camera images + joint positions + z and predicts the whole chunk.
- **Inference**: the encoder is dropped and **z = 0**; images and joints alone drive prediction.

### The Network

1. **ResNet18** embeds each camera image into a 300×512 feature sequence (ALOHA's 4 cameras → 1200×512 visual tokens).
2. Joint positions and latent z are projected to 512 dims and concatenated with visual features.
3. A **Transformer encoder** fuses what the robot sees, its configuration, and the demonstration style.
4. A **Transformer decoder** with k positional queries predicts all k future actions at once (ALOHA: chunk_size=100 → 100 timesteps × 14 joint dimensions).
5. Loss: **L1 reconstruction loss + CVAE KL regularization**.

```text
Training:  demonstration → CVAE learns z → images + joints + z → Transformer → action chunk
Inference: images + joints + z=0 → Transformer → action chunk → temporal ensembling → robot
```

> One-line summary: **action chunking decides how many future actions are predicted together; temporal ensembling smooths execution; the CVAE models that several demonstrations can be valid for the same task.**

📖 Full derivation with figures: [ACT — Low-Cost Bimanual Manipulation (Seeed Paper Reading)](https://wiki.seeedstudio.com/robotics/paper_reading_act/).

---

## Hardware, Environment, and Data Collection

**Hardware, shopping links, environment setup, calibration, teleoperation, cameras, and dataset recording are identical to the [Pi0.5 tutorial](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md)** — the arms, dataset format, and workflow stages are the same; only the training policy changes.

Quick recap (details and all command variants for B601-DM / B601-RS / SO-ARM101 in the [Pi0.5 tutorial](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-by-step-practice)):

1. **Environment** — Ubuntu 22.04/24.04 or Jetson Orin (JetPack 6.0/6.1), Python 3.12; Miniforge → clone [Seeed-Projects/lerobot](https://github.com/Seeed-Projects/lerobot) → `conda create -n lerobot python=3.12` → `pip install -e ./lerobot` + `lerobot-teleoperator-rebot-arm-102` + `lerobot-robot-seeed-b601` + `motorbridge` → `conda install ffmpeg -c conda-forge`. (ACT needs no extra policy dependency — no `.[pi]` step.)
2. **Calibrate** follower + leader (`lerobot-calibrate`, per-arm commands in the Pi0.5 tutorial).
3. **Teleoperate** to verify the leader-follower pairing.
4. **Cameras** — two views (`front` + `side`), 640×480 @ 30 fps, MJPG.
5. **Record a dataset** — with `lerobot-record`, `--dataset.single_task="Grab the black cube"`, etc. For ACT, **~50 episodes** of a consistent tabletop task is the classic starting scale (the ALOHA paper's 50 trajectories / ~10 minutes of demos).

B601-DM record command for reference:

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
    --dataset.num_episodes=50 \
    --dataset.single_task="Grab the black cube" \
    --dataset.push_to_hub=false \
    --dataset.episode_time_s=30 \
    --dataset.reset_time_s=30
```

---

## Train ACT

**With a local dataset** (recorded with `--dataset.push_to_hub=false`):

```bash
lerobot-train \
  --dataset.repo_id=seeed_rebot_b601_dm/test \
  --policy.type=act \
  --output_dir=outputs/train/act_rebot_test \
  --job_name=act_rebot_test \
  --policy.device=cuda \
  --wandb.enable=false \
  --policy.push_to_hub=false \
  --steps=300000
```

**With a Hub dataset:**

```bash
lerobot-train \
  --dataset.repo_id=${HF_USER}/rebot_test \
  --policy.type=act \
  --output_dir=outputs/train/act_rebot_test \
  --job_name=act_rebot_test \
  --policy.device=cuda \
  --wandb.enable=false \
  --steps=300000
```

Parameters that matter:

- **`--policy.type=act`** selects the ACT policy (loads `configuration_act.py`). The policy automatically adapts to your robot's motor states, motor actions, and number of cameras — this information is already stored in the dataset.
- **`--steps=300000`** — the algorithm defaults to 800,000; adjust to task difficulty. Set it higher if unsure: checkpoints are saved during training and evaluation can resume from any checkpoint.
- **`--policy.device=cuda`** for NVIDIA GPUs (`mps` for Apple Silicon, `cpu` works but is slow).
- **`--wandb.enable=true`** to visualize training charts with [Weights & Biases](https://docs.wandb.ai/quickstart) (optional; `wandb login` first).

**RTX 50-series GPUs:** add `--dataset.video_backend=pyav` to bypass missing APIs in the preview torchvision:

```bash
lerobot-train \
  --dataset.repo_id=seeed_rebot_b601_dm/test \
  --dataset.video_backend=pyav \
  --policy.type=act \
  --output_dir=outputs/train/act_rebot_test \
  --policy.device=cuda \
  --wandb.enable=false \
  --policy.push_to_hub=false \
  --steps=300000
```

**Resume from a checkpoint:**

```bash
lerobot-train \
  --config_path=outputs/train/act_rebot_test/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

**Training time:** ACT on 50 episodes takes ~6 hours on an RTX 3060 (8GB) laptop, ~2–3 hours on RTX 4090 / A100.

Official reference: [LeRobot ACT documentation](https://huggingface.co/docs/lerobot/act).

---

## Evaluate on the Real Arm

Run the trained checkpoint as an autonomous policy — the arm executes the task while the system records evaluation episodes:

```bash
lerobot-record \
  --robot.type=seeed_b601_dm_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.can_adapter=damiao \
  --robot.cameras='{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: "MJPG"}, side: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30, fourcc: "MJPG"} }' \
  --robot.id=follower1 \
  --display_data=false \
  --dataset.repo_id=seeed/eval_test123 \
  --dataset.single_task="Put lego brick into the transparent box" \
  --policy.path=outputs/train/act_rebot_test/checkpoints/last/pretrained_model
```

Checklist:

- `--policy.path` points at the trained weights — e.g. `outputs/train/act_rebot_test/checkpoints/last/pretrained_model` — or a Hub repo (e.g. `${HF_USER}/act_rebot_test`).
- Name evaluation datasets with an `eval_` prefix; they record video + data separately from training data.
- `File exists: '...eval_xxxx'` → delete the old `eval_` folder and re-run.
- `mean is infinity` error → the camera keys in `--robot.cameras` (`front`, `side`) must match the collection session exactly.
- Vary initial conditions (object position, lighting) over N trials and measure the success rate — **low training loss measures imitation accuracy, not task completion**.

**Closing the loop:** failures become input for the next round — add demonstrations for the situations that fail, retrain, evaluate again.

---

## ACT vs Pi0.5: Which Should You Start With?

| | **ACT** | **Pi0.5 (π0.5)** |
| :--- | :--- | :--- |
| Type | Pure imitation learning (CNN + Transformer + CVAE) | Vision-Language-Action foundation model |
| Pretraining | None — trains from scratch | Starts from `lerobot/pi05_base` web-scale pretraining |
| Data scale | ~50 episodes of one task | 50–100 episodes; benefits more from variation |
| Language | No task text | Task instruction in the prompt |
| Generalization | Narrow — your table, your cameras, your task | Better open-world behavior from co-training |
| VRAM / time | Trains on RTX 3060 8GB, hours | Larger; use `bfloat16` + `gradient_checkpointing`, or cloud/no-code |
| Best for | Learning the full pipeline, fast iteration, deterministic tabletop tasks | Language-conditioned tasks, more robust generalization |

**Recommended path:** run ACT end-to-end first (this tutorial), then fine-tune Pi0.5 on the same dataset and compare: [How to Run Pi0.5 on reBot Arm](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md).

---

## FAQ

**`Could not connect on port /dev/ttyUSB0 or /dev/ttyACM0`** — grant serial permissions: `sudo chmod 666 /dev/ttyUSB* /dev/ttyACM*`.

**`No valid stream found in input file`** — `conda install ffmpeg=7.1.1 -c conda-forge`.

**The arm fails to grasp after training** — camera positions/angles/lighting must stay stable across collection; keep the object visible in both views; record ≥50 episodes; start with a simple, consistent task before adding variation.

**Keyboard shortcuts unresponsive during recording** — `pip install pynput==1.6.8`; verify `$DISPLAY` is set.

**Camera image errors** — connect USB cameras directly, not through a hub.

**Which LeRobot repo?** — this tutorial targets the verified fork `https://github.com/Seeed-Projects/lerobot.git`.

**No local GPU?** — train in a Colab notebook (see [LeRobot ACT docs](https://huggingface.co/docs/lerobot/act)), or use the no-code [SenseCraft Robotics](../2.SenseCraft_Robotics_No_Code_Guide/README.md) cloud training — its default policy is ACT.

---

## References

- ACT paper: [Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware](https://arxiv.org/abs/2304.13705) (Zhao, Kumar, Levine, Finn)
- LeRobot: [repository](https://github.com/huggingface/lerobot) · [ACT docs](https://huggingface.co/docs/lerobot/act) · [ACT implementation](https://github.com/huggingface/lerobot/blob/main/src/lerobot/policies/act/modeling_act.py)
- Seeed deep-dive: [ACT — Low-Cost Bimanual Manipulation](https://wiki.seeedstudio.com/robotics/paper_reading_act/) · hands-on: [ACT Training](https://wiki.seeedstudio.com/robotics/hands_on_act_training/) / [ACT Deployment](https://wiki.seeedstudio.com/robotics/hands_on_act_deployment/)
- Source wiki tutorial: [reBot Arm B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/)
- ALOHA project: [tonyzhaozh.github.io/aloha](https://tonyzhaozh.github.io/aloha/)
- Next in this series: [How to Run Pi0.5 on reBot Arm](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md)

---

## Tech Support & Product Discussion

<div align="center">
  <a href="https://forum.seeedstudio.com/" target="_blank"><strong>Seeed Forum</strong></a> ·
  <a href="https://www.seeedstudio.com/contacts" target="_blank"><strong>Contact Us</strong></a> ·
  <a href="https://discord.gg/eWkprNDMU7" target="_blank"><strong>Discord</strong></a> ·
  <a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" target="_blank"><strong>GitHub Discussion</strong></a>
</div>
