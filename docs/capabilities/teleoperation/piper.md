---
title: "Piper Teleoperation"
description: "Operate a Piper arm with Quest VR or a keyboard."
---

DimOS supports direct Piper teleoperation through Quest VR and keyboard input.
These paths operate the arm only; they do not record episodes or create
datasets.

## Quest

Start the Quest blueprint on the robot:

```bash
dimos run teleop-quest-piper
```

Open the robot's LAN address from the Quest browser. The Quest web endpoint
binds to all LAN interfaces (HTTPS, port `8443` by default), so the headset
and robot must be on the same network. The left controller drives the Piper
arm; the blueprint routes its command stream to the Piper coordinator.

Use `--simulation` to run the same composition against the supported simulator.

## Keyboard

Start keyboard teleoperation from the robot's terminal:

```bash
dimos run keyboard-teleop-piper
```

Use the keyboard controls shown by the teleop module for Cartesian arm motion.
The configured gripper keys open and close the Piper gripper while leaving arm
motion controls unchanged. Both teleop paths apply the Piper motion safety
limits before commands reach the coordinator.
