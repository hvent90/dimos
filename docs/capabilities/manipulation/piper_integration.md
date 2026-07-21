---
title: "Piper Integration"
description: "Connect and run a Piper arm with DimOS manipulation and teleoperation blueprints."
---

DimOS integrates the Piper arm through its CAN-based `piper_sdk` adapter and the
standard manipulation stack. The Piper hardware configuration uses the CAN port
from `GlobalConfig.can_port`; when it is not set, teleoperation blueprints use
the mock adapter where supported.

The manipulation extra provides the verified software prerequisites: Drake and
`piper-sdk`. Piper is a six-degree-of-freedom arm in the DimOS hardware model.

For real hardware, set `GlobalConfig.can_port` to the CAN interface used by the
arm. The default hardware address is `can0`; the standard Piper configuration
uses the mock adapter when no CAN port is configured. No separate DimOS hardware
bring-up command is documented here.

## Run a Piper blueprint

Use the coordinator for the basic manipulation composition:

```bash
dimos run coordinator-piper
```

For keyboard Cartesian teleoperation, use:

```bash
dimos run keyboard-teleop-piper
```

The Quest teleoperation composition is available as:

```bash
dimos run teleop-quest-piper
```

Add `--simulation` to run the supported MuJoCo composition instead of real
hardware.

## Integration points

The Piper blueprints are defined in:

- [`basic.py`](/dimos/robot/manipulators/piper/blueprints/basic.py) — coordinator
- [`teleop.py`](/dimos/robot/manipulators/piper/blueprints/teleop.py) — keyboard,
  Cartesian, and Quest teleoperation compositions
- [`config.py`](/dimos/robot/manipulators/piper/config.py) — hardware and model
  configuration

The manipulation requirements include Drake and `piper-sdk`. Follow the
hardware vendor's documentation for connecting and powering the arm; DimOS
provides the integration and blueprint compositions, not a separate hardware
bring-up procedure.
