# Manipulator Adapters

Hardware IO for manipulator arms. Each arm gets an adapter that wraps its vendor SDK and exposes a standard Protocol; the `ControlCoordinator` drives every arm through that Protocol.

To integrate a new arm, follow **[Adding a Custom Arm](../../../docs/capabilities/manipulation/adding_a_custom_arm.md)**. This README is a quick reference for what lives here.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│              ControlCoordinator (100Hz tick loop)            │
│  - Reads state from every adapter                           │
│  - Runs tasks (trajectory, servo, velocity, eef_twist, ...) │
│  - Arbitrates per-joint conflicts by priority               │
└─────────────────────┬───────────────────────────────────────┘
                      │ calls Protocol methods
┌─────────────────────▼───────────────────────────────────────┐
│              Adapter (implements the Protocol)               │
│  - Wraps the vendor SDK                                      │
│  - Converts vendor units to SI                               │
│  - Swappable: XArmAdapter, PiperAdapter, MockAdapter, ...    │
└─────────────────────────────────────────────────────────────┘
```

Adapters hold no threading and no ports. The coordinator owns the control loop and calls into the adapter each tick. Blueprints and robot configs live under `dimos/robot/manipulators/<arm>/`, not here.

## Directory structure

```
manipulators/
├── spec.py       # ManipulatorAdapter Protocol + shared types
├── registry.py   # Lazy adapter registry (discovers _registry.py manifests)
├── a750/         # A-750 6-DOF (serial)
├── base/         # Shared adapter helpers
├── mock/         # MockAdapter — no hardware, reference implementation
├── openarm/      # OpenArm bimanual (raw SocketCAN, no vendor SDK)
├── piper/        # Piper (CAN)
├── sim/          # ShmMujocoAdapter — MuJoCo simulation
└── xarm/         # xArm 6/7 (TCP/IP)
```

Every arm directory holds `adapter.py` plus a `_registry.py` manifest. There are no `__init__.py` files: these are PEP 420 namespace packages, so import submodules directly.

## Registered adapters

| `adapter_type` | Class | Transport |
|---|---|---|
| `a750` | `A750Adapter` | Serial |
| `mock` | `MockAdapter` | None |
| `openarm` | `OpenArmAdapter` | SocketCAN |
| `piper` | `PiperAdapter` | CAN |
| `sim_mujoco` | `ShmMujocoAdapter` | Shared memory |
| `xarm` | `XArmAdapter` | TCP/IP |

```python
from dimos.hardware.manipulators.registry import adapter_registry

adapter_registry.available()                              # list registered names
adapter_registry.create("xarm", address="192.168.1.185", dof=6)
```

Discovery is lazy. The registry loads each `_registry.py` manifest (stdlib imports only) to learn the names, and imports the adapter module itself only on `create()`. A missing vendor SDK therefore fails loudly at `create()`.

## ManipulatorAdapter Protocol

Duck-typed, defined in `spec.py`. No inheritance: match the signatures.

| Category | Methods |
|----------|---------|
| Connection | `connect`, `disconnect`, `is_connected` |
| Lifecycle | `activate`, `deactivate` |
| Info | `get_info`, `get_dof`, `get_limits` |
| Mode | `set_control_mode`, `get_control_mode` |
| State | `read_joint_positions`, `read_joint_velocities`, `read_joint_efforts`, `read_state`, `read_error` |
| Motion | `write_joint_positions`, `write_joint_velocities`, `write_stop` |
| Servo | `write_enable`, `read_enabled`, `write_clear_errors` |

Optional, return `None` or `False` when unsupported and never raise:
`read_cartesian_position`, `write_cartesian_position`, `read_gripper_position`, `write_gripper_position`, `read_force_torque`.

`mock/adapter.py` implements all of them and is the reference to copy.

## Unit conventions

Everything crossing the adapter boundary is SI.

| Quantity | Unit |
|----------|------|
| Angles | radians |
| Angular velocity | rad/s |
| Torque | Nm |
| Position | meters |
| Force | Newtons |

## Testing without hardware

Set `adapter_type="mock"` on the `HardwareComponent` in a blueprint. The whole coordinator and planner path runs unchanged. Most arm configs expose a `mock_without_address=True` flag that does this automatically when no address is set.
