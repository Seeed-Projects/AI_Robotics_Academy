# reBot Arm VLA Tutorial Series — Run SOTA Policies on an Open-Source Robotic Arm

**Language / 语言: [English](./English/) | [简体中文](https://wiki.seeedstudio.com/robotics_page/) (wiki)**

A growing series of algorithm-specific tutorials for running state-of-the-art Vision-Language-Action and imitation-learning policies on the **100% open-source reBot Arm** (B601-DM / B601-RS) and **SO-ARM101** leader-follower kits, through the open-source [LeRobot](https://github.com/huggingface/lerobot) framework.

Each tutorial is self-contained: theory deep-dive, hardware list with shopping links, environment setup, and the full hands-on pipeline — calibrate → teleoperate → record a dataset → fine-tune → deploy on the real arm.

| # | Tutorial | Policy | Status |
| :--- | :--- | :--- | :--- |
| 1 | [How to Run Pi0.5 (π0.5) on reBot Arm](./English/1.How_to_Run_Pi0.5_on_reBot_Arm/README.md) | `pi05` · [arXiv:2504.16054](https://arxiv.org/abs/2504.16054) | ✅ Published |
| 2 | [SenseCraft Robotics: No-Code Guide (Windows / macOS / Linux)](./English/2.SenseCraft_Robotics_No_Code_Guide/README.md) | GUI platform | ✅ Published |
| 3 | How to Run ACT on reBot Arm | `act` | 🚧 Coming soon |
| 4 | How to Run SmolVLA on reBot Arm | `smolvla` | 🚧 Coming soon |
| 5 | How to Run Pi0 (π0) on reBot Arm | `pi0` | 🚧 Coming soon |
| 6 | How to Run Pi0-FAST (π0-FAST) on reBot Arm | `pi0fast` | 🚧 Coming soon |
| 7 | How to Run GR00T N1.5 on reBot Arm | `groot` | 🚧 Coming soon |

## Which arms are supported?

The tutorials are verified on **reBot Arm B601-DM**, **reBot Arm B601-RS**, and **SO-ARM101**, and the same workflow applies to any LeRobot-compatible arm. See the compatibility section of each tutorial (for Pi0.5: [Which Robotic Arms Are Compatible with Pi0.5?](./English/1.How_to_Run_Pi0.5_on_reBot_Arm/README.md#which-robotic-arms-are-compatible-with-pi05)).

## Hardware

- [reBot Arm B601-DM Bundle](https://www.seeedstudio.com/reBot-Arm-B601-DM-Bundle.html) (follower + reBot Arm 102 leader)
- [reBot Arm B601-RS Assembled Kit with Gripper](https://www.seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html)
- [SO-101 Low-Cost AI Arm Kit Pro](https://www.seeedstudio.com/SO-101-Low-Cost-AI-Arm-Kit-Pro-p-6427.html)
- [Robot Arm Camera Mount for SO-10x / reBot B601](https://www.seeedstudio.com/Robot-Arm-Camera-Mount-for-LeRobot-SO-10&reBot-B601-p-7003.html)
- Open-source arm project: [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm/)

## Related resources

- Full paper deep-dives (theory): [Seeed Robotics Paper Reading course](https://wiki.seeedstudio.com/robotics/paper_reading_getting_started/)
- Source wiki tutorials: [B601-DM with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/) · [B601-RS with LeRobot](https://wiki.seeedstudio.com/rebot_arm_b601_rs_lerobot/)
- LeRobot: [GitHub](https://github.com/huggingface/lerobot) · [Pi0.5 docs](https://huggingface.co/docs/lerobot/pi05)
- Seeed verified fork: [Seeed-Projects/lerobot](https://github.com/Seeed-Projects/lerobot)

## License

MIT — same as the [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm/) open-source hardware project.
