# How to Run SmolVLA on reBot Arm — Theory, Setup, Training & Deployment

<p align="center">
    <img width="600" src="https://files.seeedstudio.com/wiki/robotics/projects/rebot_arm/rebot_dm.png" alt="reBot Arm B601-DM" />
</p>

<p align="center">
    <a href="https://github.com/Seeed-Projects/reBot-DevArm/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT" /></a>
    <img src="https://img.shields.io/badge/Python-3.12+-blue.svg" alt="Python Version" />
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Ubuntu%20%7C%20Jetson%20Orin-orange.svg" alt="Platform" />
    <img src="https://img.shields.io/badge/Policy-smolvla%20450M-green.svg" alt="Policy: SmolVLA" />
</p>

**[SmolVLA](https://arxiv.org/abs/2506.01844)** is Hugging Face's **lightweight vision-language-action foundation model** — small enough to fine-tune on one GPU and to run on consumer GPUs or even a CPU, yet competitive with VLAs roughly ten times larger. It is pre-trained on carefully cleaned **community robot data** (including LeRobot datasets from low-cost arms), which makes it a natural match for the open-source **reBot Arm** and **SO-ARM101**.

In LeRobot it is the policy `smolvla`, fine-tuned from the pretrained checkpoint `lerobot/smolvla_base`. This tutorial covers the theory, setup, and the full pipeline: calibrate → teleoperate → record demonstrations → fine-tune SmolVLA → run it on the real arm.

> 📚 **Want the full tutorial and more resources?** This GitHub guide is a companion to the complete documentation on the **[Seeed Studio Wiki](https://wiki.seeedstudio.com/robotics_page/)**, which is continuously updated and includes video walkthroughs, hardware assembly, and troubleshooting:
> - **Full wiki tutorials:** [reBot Arm B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/) · [reBot Arm B601-RS with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_rs_lerobot/) · [SO-ARM101 with LeRobot](https://wiki.seeedstudio.com/lerobot_so100m_new/)
> - **SmolVLA theory deep-dive:** [SmolVLA — Affordable Vision-Language-Action Models](https://wiki.seeedstudio.com/robotics/paper_reading_smolvla/)
> - **Hardware setup:** [reBot Arm B601-DM Getting Started](https://wiki.seeedstudio.com/rebot_b601_dm_getting_started/) · [reBot Arm B601-RS Getting Started](https://wiki.seeedstudio.com/rebot_b601_rs_getting_started/)

Part of the [Seeed reBot Arm VLA tutorial series](../README.md) · [Seeed AI Robotics Academy](../../../README.md). If this guide helps you, please ⭐ star [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm/).

---

## Table of Contents

- [Which Robotic Arms Are Compatible with SmolVLA?](#which-robotic-arms-are-compatible-with-smolvla)
- [SmolVLA Theory Deep-Dive](#smolvla-theory-deep-dive)
- [Hardware, Environment, and Data Collection](#hardware-environment-and-data-collection)
- [Train SmolVLA](#train-smolvla)
- [Evaluate on the Real Arm](#evaluate-on-the-real-arm)
- [SmolVLA vs ACT vs Pi0.5](#smolvla-vs-act-vs-pi05)
- [FAQ](#faq)
- [References](#references)

---

## Which Robotic Arms Are Compatible with SmolVLA?

**SmolVLA works with any LeRobot-compatible arm with a recorded demonstration dataset.** Fine-tuning adapts the action expert to your robot's state/action dimensions and cameras automatically. Verified on Seeed's open hardware:

| Robotic arm | Actuation | Interface | Status | Buy |
| :--- | :--- | :--- | :--- | :--- |
| **reBot Arm B601-DM** (Seeed) | Damiao DM brushless servos | USB-CAN bridge (`/dev/ttyACM0`) | ✅ Verified, this guide | [reBot Arm B601-DM Bundle](https://www.seeedstudio.com/reBot-Arm-B601-DM-Bundle.html) |
| **reBot Arm B601-RS** (Seeed) | RobStride RS actuators | SocketCAN (`can0`) / PCAN | ✅ Verified | [reBot Arm B601-RS Kit](https://www.seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html) |
| **SO-ARM101 / SO-101** (Seeed × LeRobot) | Feetech serial servos | USB serial (`/dev/ttyACM0`) | ✅ Verified | [SO-101 Low-Cost AI Arm Kit Pro](https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html) |

SmolVLA's pre-training data comes largely from the **community of SO-ARM-style low-cost arms**, so SO-101-class hardware is the closest match to its training distribution. Other LeRobot-supported arms (SO-100, Koch v1.1, ALOHA-style bimanual) work through the same fine-tuning path.

---

## SmolVLA Theory Deep-Dive

*Condensed from the Seeed paper-reading deep-dive: [SmolVLA — Affordable Vision-Language-Action Models](https://wiki.seeedstudio.com/robotics/paper_reading_smolvla/). Paper: [arXiv:2506.01844](https://arxiv.org/abs/2506.01844) (Hugging Face).*

### Why a Small VLA?

Most VLAs have billions of parameters, are expensive to train, and are hard to deploy on real robots. SmolVLA is **450M parameters total (~100M in the action expert)**: trainable on one GPU (~30K GPU hours for the full project), deployable on consumer GPUs or CPU. On LIBERO simulation, the 0.45B model outperforms the 3.3B π0 and 7B OpenVLA on average success rate.

### Architecture

```text
Multi-view RGB images + language instruction + robot state
        ↓  VLM (SmolVLM-2 backbone)
Scene-conditioning features
        ↓  Action Expert + noisy actions, flow matching
Action chunk
```

Three deliberate efficiency choices:

- **64 visual tokens per frame** — tiling disabled, pixel shuffle keeps computation low (at the cost of some fine detail).
- **Only the first half of the VLM's layers** (16 of the language-model layers) — features are extracted at layer N = L/2; keeping the larger model's width beats swapping in a smaller VLM.
- **A ~100M action expert** alternating **cross-attention** (action tokens read what is visible, what the instruction requests, and the robot configuration) with **causal self-attention** (keeps the future sequence coherent, "approach → close gripper → lift → transport → place"). Causal masking outperforms bidirectional attention in ablations.

**Training:** the VLM is frozen; the action expert is trained with **flow matching** (chunk size 50, 10 flow steps) over noisy action chunks.

### Community-Data Pre-Training

Robot data forms "islands" — different robots, control frequencies, joint definitions, cameras, and labels. SmolVLA's recipe:

1. **Select** community datasets by robot type, episode count, quality, and frame coverage (an order of magnitude less data than large VLAs use).
2. **Clean task language** — vague labels like `Hold` / `Up` are rewritten into complete action-verb instructions using Qwen2.5-VL-3B.
3. **Standardize cameras** — map views to a fixed **Top → Wrist → Side** ordering (`OBS_IMAGE_1/2/3`), so the same input slot always means the same viewpoint.

This is why community-recorded demonstrations (like yours, on a reBot/SO-101 arm) are exactly the distribution SmolVLA was built to learn from.

### Asynchronous Inference

A synchronous chunked policy either executes a whole chunk open-loop or pauses waiting for the next inference. SmolVLA's async stack splits the roles:

| Component | Responsibility |
| :--- | :--- |
| **RobotClient** | Capture images, read state, keep the action queue, execute |
| **PolicyServer** | Receive observations, run the model, return the next chunk |

A threshold **g ∈ [0,1]** triggers the next request when the remaining queue fraction `|A_t|/n` drops below it — `g=0` approximates synchronous (pauses), `g=1` requests almost every cycle (max responsiveness, max compute), `g≈0.7` is the paper's illustration. Near-duplicate observations (joint-space distance below ε) are skipped to save compute. In LeRobot this is available for any chunked policy (including SmolVLA, ACT, Pi0): see the [async inference section of the Pi0.5 tutorial](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-6-evaluate-on-the-real-arm).

📖 Full details: [SmolVLA — Affordable Vision-Language-Action Models (Seeed Paper Reading)](https://wiki.seeedstudio.com/robotics/paper_reading_smolvla/).

---

## Hardware, Environment, and Data Collection

**Hardware, shopping links, environment setup, calibration, teleoperation, cameras, and recording are shared with the [Pi0.5 tutorial](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md).** One extra dependency for SmolVLA:

```bash
cd ~/rebot_lerobot/lerobot
pip install -e ".[smolvla]"
cd ~
```

**Dataset advice for SmolVLA specifically:**

- Start with **~50 episodes** of your task (too few leads to poor learning/generalization).
- Cover the task's variations: 5 object positions × 10 episodes each = 50 episodes.
- Experience from the field: recording only 25 episodes is often insufficient — **data quality and quantity both matter**.

B601-DM record command (RS / SO-ARM variants in the [Pi0.5 tutorial](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-4-record-a-dataset)):

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

## Train SmolVLA

Fine-tune from the pretrained 450M base (`lerobot/smolvla_base`) on your dataset:

```bash
lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=${HF_USER}/mydataset \
  --batch_size=64 \
  --steps=20000 \
  --output_dir=outputs/train/my_smolvla \
  --job_name=my_smolvla_training \
  --policy.device=cuda \
  --wandb.enable=true
```

- For a **local dataset**, use your collection `repo_id` (e.g. `seeed_rebot_b601_dm/test`) and add `--policy.push_to_hub=false`.
- The official example trains **20k steps — about 4 hours on a single A100** (reference only; actual time varies by hardware).
- **Out of memory?** Reduce `--batch_size` first; once it runs, gradually increase it.
- All available parameters: `lerobot-train --help`.
- No GPU? Train via a Colab notebook — see the [official SmolVLA tutorial](https://huggingface.co/docs/lerobot/smolvla).

**Resume from a checkpoint:**

```bash
lerobot-train \
  --config_path=outputs/train/my_smolvla/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

Official reference: [LeRobot SmolVLA documentation](https://huggingface.co/docs/lerobot/smolvla).

---

## Evaluate on the Real Arm

Load the fine-tuned model and let the arm perform the task autonomously while the system records the run:

```bash
lerobot-record \
  --robot.type=seeed_b601_dm_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.can_adapter=damiao \
  --robot.id=follower1 \
  --robot.cameras='{ front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30, fourcc: "MJPG"}, side: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30, fourcc: "MJPG"} }' \
  --dataset.single_task="Grasp a lego block and put it in the bin." \
  --dataset.repo_id=${HF_USER}/eval_DATASET_NAME_test \
  --dataset.episode_time_s=50 \
  --dataset.num_episodes=10 \
  --policy.path=${HF_USER}/FINETUNE_MODEL_NAME
```

How to fill the parameters:

- `--robot.port` — the serial port on your machine (`/dev/ttyACM0` or `/dev/ttyUSB0`; `can0` + `--robot.can_adapter=socketcan` for B601-RS; SO-ARM101 uses `/dev/ttyACM0` + `so101_follower`).
- `--robot.cameras` — your real `index_or_path`; camera keys (`front`, `side`) must **exactly match** the recording session.
- `--dataset.single_task` — match the description used when recording.
- `--policy.path` — local weights (`outputs/train/my_smolvla/checkpoints/last/pretrained_model`) or a Hub repo (`${HF_USER}/my_smolvla`).

**Optional — manual correction between evaluation episodes:** add the teleoperator block so you can re-position the arm by hand between episodes:

```bash
  --teleop.type=rebot_arm_102_leader \
  --teleop.port=/dev/ttyUSB0 \
  --teleop.id=rebot_arm_102_leader
```

**Smoother motion:** pair SmolVLA with [async inference](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#step-6-evaluate-on-the-real-arm) (RobotClient + PolicyServer) — the model's native deployment mode from the paper.

---

## SmolVLA vs ACT vs Pi0.5

| | **ACT** | **SmolVLA** | **Pi0.5 (π0.5)** |
| :--- | :--- | :--- | :--- |
| Type | Imitation learning | Lightweight VLA | VLA with open-world generalization |
| Parameters | ~small CNN+Transformer | **450M** | ~3B backbone + 300M expert |
| Pretraining | None | Community robot data | Multi-robot + web data, subtask hierarchy |
| Language | No | Task instruction | Task instruction + subtasks |
| Fine-tune cost | Hours on RTX 3060 | ~4h on A100 for 20k steps | Heavier; bf16 + gradient checkpointing |
| Runs on | Any GPU | Consumer GPU or even CPU | GPU recommended |
| Best for | First steps, deterministic tasks | Budget VLA with language + async inference | Strongest generalization |

**Suggested order:** [ACT](../3.How_to_Run_ACT_on_reBot_Arm/README.md) → **SmolVLA** (this tutorial) → [Pi0.5](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md), reusing the same dataset each time.

---

## FAQ

**`Could not connect on port /dev/ttyUSB0 or /dev/ttyACM0`** — `sudo chmod 666 /dev/ttyUSB* /dev/ttyACM*`.

**`No valid stream found in input file`** — `conda install ffmpeg=7.1.1 -c conda-forge`.

**Out of memory during training** — lower `--batch_size` first; then re-raise it gradually.

**`mean is infinity` / normalization errors at evaluation** — camera keys and `single_task` must match the recording session exactly.

**Camera image errors** — connect USB cameras directly, not through a hub.

**No local GPU?** — Colab (official docs) or the no-code [SenseCraft Robotics](../2.SenseCraft_Robotics_No_Code_Guide/README.md) cloud training.

**Which LeRobot repo?** — the verified fork `https://github.com/Seeed-Projects/lerobot.git`.

---

## References

- SmolVLA paper: [SmolVLA: A vision-language-action model for affordable and efficient robotics](https://arxiv.org/abs/2506.01844) (Hugging Face)
- LeRobot: [repository](https://github.com/huggingface/lerobot) · [SmolVLA docs](https://huggingface.co/docs/lerobot/smolvla) · [pretrained smolvla_base](https://huggingface.co/lerobot/smolvla_base)
- Seeed deep-dive: [SmolVLA — Affordable Vision-Language-Action Models](https://wiki.seeedstudio.com/robotics/paper_reading_smolvla/)
- Source wiki tutorial: [reBot Arm B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/)
- Async inference: [HF blog — async robot inference](https://huggingface.co/blog/async-robot-inference)
- Series: [Pi0.5 tutorial](../1.How_to_Run_Pi0.5_on_reBot_Arm/README.md) · [ACT tutorial](../3.How_to_Run_ACT_on_reBot_Arm/README.md)

---

## Tech Support & Product Discussion

<div align="center">
  <a href="https://forum.seeedstudio.com/" target="_blank"><strong>Seeed Forum</strong></a> ·
  <a href="https://www.seeedstudio.com/contacts" target="_blank"><strong>Contact Us</strong></a> ·
  <a href="https://discord.gg/eWkprNDMU7" target="_blank"><strong>Discord</strong></a> ·
  <a href="https://github.com/Seeed-Studio/wiki-documents/discussions/69" target="_blank"><strong>GitHub Discussion</strong></a>
</div>
