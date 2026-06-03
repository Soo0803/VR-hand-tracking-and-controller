# Hand Tracking Streamer - ManiSkill Bridge

Meta Quest hand/controller telemetry for teleoperating a Franka arm in ManiSkill.

This repository adapts the original Hand Tracking Streamer Unity app into a robotics
teleoperation pipeline. The Quest headset streams hand, wrist, controller, and optional
head pose data to a host machine. The Python bridge scripts then convert Unity
left-handed coordinate data into the right-handed, Z-up frame expected by the ManiSkill
Franka teleoperation stack and forward compact binary UDP packets to the simulator.

The current branch is intended for ManiSkill simulation. A real Franka/FR3 hardware
bridge uses a different robot-frame mapping and should be kept separate from this
simulation-oriented bridge.

## Two teleoperation option: hand tracking teleoperation, and controller tracking teleoperation

There are two host-side ManiSkill bridge scripts. Choose the script that matches the
tracking mode selected inside the Meta Quest app:

| Quest app tracking mode | Python bridge to run | Default output port | Use this for |
| --- | --- | --- | --- |
| Hand tracking / hand landmarks | `scripts/hand_bridge.py` | `9877` | Wrist pose plus 21 hand landmarks converted into wrist pose and finger curl values |
| Controller tracking / Quest controllers | `scripts/controller_bridge.py` | `9876` | Left/right Quest controller pose and optional A/primary-button grasp signal |

If the Quest app is set to a hand mode such as `Both Hands`, `Left Hand Only`, or
`Right Hand Only`, run `hand_bridge.py`. 
If the Quest app is set to `Controllers Only`, or you are using controller pose teleoperation, run `controller_bridge.py`.

## What This Project Is For

- Use a Meta Quest headset as a low-latency teleoperation input device.
- Stream OpenXR hand landmarks and wrist poses over TCP or UDP.
- Convert hand landmark motion into per-finger curl values for gripper or dexterous
  control experiments.
- Forward Quest controller poses as Franka end-effector commands for ManiSkill.
- Prototype virtual Franka teleoperation before moving to real robot hardware.

## Steps to start the teleoperation and simulation

Terminal 1, start the bridge:
Before running the bash command below, make sure that you have allow the cable connection to allow data connection from the VR to the PC by enabling it on the Meta Quest VR headset screen

Terminal 2, start ManiSkill teleoperation so it listens on the expected UDP port.
 
```bash
adb reverse tcp:8000 tcp:8000
python3 scripts/hand_bridge.py --in-protocol tcp --in-port 8000 --out-port 9877 --verbose
```

In the Quest app:

1. Select `TCP Wired`. (Make sure the cable have been connected between the VR headset and the PC)
2. Set IP to `127.0.0.1`.
3. Set port to `8000`.
4. Select `Both Hands` or `Controller` mode on the option.
5. Press Start.

If packets are flowing, the bridge logs tracked hands and curl values, and ManiSkill
receives binary UDP packets on `127.0.0.1:9877` at the log output on the bridge.py terminal

## System Overview

```text
Meta Quest / Unity app
        |
        | UTF-8 CSV over TCP or UDP
        v
Host Python bridge in scripts/
        |
        | packed float32 UDP packet
        v
ManiSkill teleoperation process
```

The Unity app is in `hand_tracking_streamer/`. The host-side bridge scripts are in
`scripts/`.

## Repository Layout

```text
.
|-- hand_tracking_streamer/          # Unity project for the Quest app
|   `-- Assets/Scripts/
|       |-- AppManager.cs            # in-headset menu, protocol, mode, stream state
|       |-- HandLandmarkStreamer.cs  # wrist + 21 hand landmarks
|       |-- ControllerPoseStreamer.cs # Quest controller pose stream
|       |-- HeadPoseStreamer.cs      # optional head pose stream
|       `-- FistTracking.cs          # left-hand open/closed fallback signal
|-- scripts/
|   |-- hand_bridge.py               # hand landmarks -> ManiSkill hand packet
|   |-- controller_bridge.py         # Quest controller pose -> ManiSkill controller packet
|   |-- sockets.py                   # raw TCP/UDP listener for debugging
|   |-- visualizer.py                # stream visualizer/debug tool
|   `-- test_bridge_logic.py         # bridge packet contract checks
|-- hand_tracking_streamer.apk       # prebuilt Quest APK, if you do not rebuild Unity
|-- CONNECTIONS.md                   # upstream HTS stream format notes
`-- pyproject.toml                   # Python script dependencies
```

## Requirements

- Meta Quest headset with developer mode enabled.
- ADB for wired TCP streaming and APK installation.
- Unity 6000.0.65f1 if rebuilding the Quest app from source.
- Python 3.13 or newer for the host scripts.
- ManiSkill teleoperation code that listens for the bridge UDP packet.

Install Python dependencies:

```bash
uv sync
```

If you are not using `uv`:

```bash
python3 -m pip install numpy matplotlib
```

## ManiSkill Hand Teleoperation, Hand Bridge

Use `scripts/hand_bridge.py` when driving ManiSkill from hand tracking data.

```bash
python3 scripts/hand_bridge.py \
  --in-protocol tcp \
  --in-port 8000 \
  --out-host 127.0.0.1 \
  --out-port 9877
```

Add `--verbose` to print parsed tracking and curl values:

```bash
python3 scripts/hand_bridge.py --in-protocol tcp --in-port 8000 --out-port 9877 --verbose
```

The bridge:

- Receives `Left/Right wrist` and `Left/Right landmarks` CSV lines from the Quest.
- Converts Unity coordinates from `(x right, y up, z forward)` to ManiSkill
  `(x forward, y left, z up)` using `(z, -x, y)`.
- Converts quaternions using `(-qz, qx, -qy, qw)`.
- Computes thumb, index, middle, ring, and pinky curl values from 21 streamed hand
  landmark positions.
- Sends a 104-byte little-endian packet at 60 Hz to the ManiSkill process.

Hand bridge output packet:

```text
<26f, little-endian, 104 bytes>

right_px, right_py, right_pz,
right_qx, right_qy, right_qz, right_qw,
right_tracked,
right_thumb_curl, right_index_curl, right_middle_curl, right_ring_curl, right_pinky_curl,
left_px, left_py, left_pz,
left_qx, left_qy, left_qz, left_qw,
left_tracked,
left_thumb_curl, left_index_curl, left_middle_curl, left_ring_curl, left_pinky_curl
```

Curl values are normalized to `[0.0, 1.0]`, where `0.0` is open/straight and `1.0`
is closed/bent.

## ManiSkill Controller Teleoperation, Controller Bridge

Use `scripts/controller_bridge.py` when the Quest app is tracking the Meta Quest
controllers instead of hands. It forwards a 72-byte packet:

```text
<18f, little-endian, 72 bytes>

right_px, right_py, right_pz,
right_qx, right_qy, right_qz, right_qw,
right_grasp,
right_tracked,
left_px, left_py, left_pz,
left_qx, left_qy, left_qz, left_qw,
left_grasp,
left_tracked
```

Run it with:

```bash
python3 scripts/controller_bridge.py \
  --in-protocol tcp \
  --in-port 8000 \
  --out-host 127.0.0.1 \
  --out-port 9876
```

By default, gripper forwarding is disabled so a Quest A/primary-button press does
not command a hard close during arm teleop testing. Enable it explicitly only when
the downstream ManiSkill controller expects the grasp signal:

```bash
python3 scripts/controller_bridge.py --in-protocol tcp --in-port 8000 --out-port 9876 --enable-gripper
```

`ControllerPoseStreamer.cs` sends the final controller CSV field as a binary
A/primary-button grasp value: `1.0` when pressed, `0.0` when released.

## Debugging Raw Streams

Before connecting ManiSkill, verify that Quest telemetry reaches the host.

TCP:

```bash
python3 scripts/sockets.py --protocol tcp --host localhost --port 8000
```

UDP:

```bash
python3 scripts/sockets.py --protocol udp --host 0.0.0.0 --port 9000
```

Count messages without printing every packet:

```bash
python3 scripts/sockets.py --protocol tcp --host localhost --port 8000 --tally
```

For visual inspection of hand landmarks:

```bash
python3 scripts/visualizer.py --protocol tcp --host localhost --port 8000 --show-fingers
```

## Stream Formats

The Unity app sends UTF-8 CSV lines. Hand tracking uses OpenXR hand joints and streams
the wrist plus 21 landmark positions per hand.

Example hand messages:

```text
Right wrist:, px, py, pz, qx, qy, qz, qw
Right landmarks:, x0, y0, z0, x1, y1, z1, ... x20, y20, z20
```

Controller messages are emitted by `ControllerPoseStreamer.cs` when the app is in
`Hands + Controllers` or `Controllers Only` mode.

Optional debug headers add a frame id and monotonic send timestamp:

```text
Right wrist | f = 123 | t = 123456789012345:, ...
```

See `CONNECTIONS.md` for the original HTS stream notes and OpenXR joint ordering.

## Safety Notes

- This branch is for ManiSkill simulation. Do not reuse the ManiSkill coordinate
  mapping directly for a physical Franka arm without validating the robot frame,
  workspace limits, gripper behavior, and emergency stop path.
- Keep gripper forwarding disabled until the downstream controller has been tested.
- Start with small simulated motions and verify axis directions before using the
  bridge for real robot teleoperation.

## Troubleshooting

- `TCP Error: Connection refused`: start the Python bridge before pressing Start in
  the Quest app.
- Wired TCP receives nothing: run `adb reverse tcp:8000 tcp:8000` and confirm the
  headset appears in `adb devices`.
- Wireless TCP cannot connect: use the host PC's LAN IPv4 address, keep Quest and PC
  on the same network, and allow the port through the firewall.
- UDP has latency spikes: use TCP wired for the most consistent stream timing.
- ManiSkill does not move: confirm the bridge output port matches the ManiSkill
  listener port (`9877` for `hand_bridge.py`, `9876` for `controller_bridge.py` by default).

## Attribution and License

This project is based on the original Hand Tracking Streamer project by Zhengyang K.
Weng and keeps the Apache-2.0 license. See `LICENSE` for details.
