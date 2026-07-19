# Galaxea A1Z + G1Z

The A1Z integration uses native Linux SocketCAN, the vendor's 250 Hz MIT
position-control loop, the G1Z URDF for gravity compensation, and the vendor
G1Z gripper implementation.

## Vendor SDK

The G1Z requires the vendor SDK's `gripper` branch; vendor `main` does not
accept `with_gripper` and cannot actuate CAN motor 7.

Run the one-command host setup as your normal user:

```bash
./dimos/robot/manipulators/galaxea_a1z/scripts/setup_a1z.sh
```

The wrapper verifies G1Z support and, when needed, installs a known-working
commit from the vendor's `gripper` branch into the DimOS virtual environment.
It requests `sudo` only when invoking the Linux SocketCAN setup. Use
`--sdk-only` to install or verify the Python SDK without touching CAN.

For a manual SDK-only installation, use the same pinned source:

```bash
uv pip install \
  "a1z @ git+https://github.com/userguide-galaxea/GALAXEA-A1Z.git@e931ecd0e25ad35df251097ba42921b3d2fa7224"
```

DimOS deliberately has no Linux userspace-CAN fallback. After boot or
reconnecting the HHS adapter, the one-command setup can be rerun, or the CAN
portion can be invoked directly to bind the adapter to the kernel driver,
configure the stable `a1zcan` SocketCAN interface, and verify transmission:

```bash
sudo ./dimos/robot/manipulators/galaxea_a1z/scripts/setup_a1z_can.sh
```

Do not start DimOS unless the script prints `A1Z CAN setup passed`. Galaxea's
HHS USB-CANFD adapter is incompatible with `gs_usb` in some Linux kernels. An
affected kernel still creates a normal-looking, UP CAN interface but drops
every transmission. The setup script detects that misleading state and prints
the supported-kernel and exact-kernel patch options. Galaxea recommends kernel
6.8.0-124 or newer; Jetsons and other pinned-kernel hosts require a persistent
driver patch built for their exact kernel.

The A1Z has no brakes. Support the arm and keep the workspace clear before
starting a hardware blueprint. Enabling the G1Z also initializes the gripper.

## Camera, teach, replay, and LeRobot export

The teach command uses a standard Linux UVC camera through DimOS's generic
`Webcam` and `CameraModule`. The default camera is `/dev/video0`; select another
video device with `--camera-index N`. Each saved episode contains 640x480 RGB
images at 15 Hz plus the measured six arm joints and gripper position.

After the CAN setup check passes, record one or more episodes:

```bash
uv run dimos a1z teach --task "pick up the object"
```

The command prints the Memory2 `.db` path. Replay a saved episode by passing
that path (the latest saved episode is selected by default):

```bash
uv run dimos a1z replay ~/.local/state/dimos/recordings/a1z_teach_<timestamp>.db
```

Convert the same recording into a LeRobot v3 dataset with synchronized video,
seven-element observation state, and seven-element action:

```bash
uv run dimos dataprep build \
  --source ~/.local/state/dimos/recordings/a1z_teach_<timestamp>.db \
  --output ./a1z_lerobot_dataset \
  --format lerobot \
  --config dimos/learning/dataprep/galaxea_a1z_state_config.json

uv run dimos dataprep inspect ./a1z_lerobot_dataset
```

The LeRobot output stores images as
`observation.images.image`, the measured arm and gripper state as
`observation.state`, and the next measured state as `action`.
