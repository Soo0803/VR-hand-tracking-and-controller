# Hand Tracking Streamer - ManiSkill Bridge

Quest-side telemetry bridge for ManiSkill teleoperation.

This repo contains the Unity application on the headset plus the Python bridge scripts that convert Quest tracking into the packet formats consumed by the simulator in `SoftHandGrasping`.

## Current Working Branch

```text
maniskill-shadowhand
```

## What This Repo Does

- Streams hand landmarks, wrist pose, and controller pose from the Quest.
- Converts Unity coordinates into the ManiSkill-friendly frame used by the simulator.
- Sends compact UDP packets to `vr_teleop.py` in the simulation repo.
- Supports both hand-tracking teleoperation and controller teleoperation.

## How The Pieces Fit Together

```text
Quest Unity app
  -> scripts/hand_bridge.py or scripts/controller_bridge.py
  -> UDP 127.0.0.1:9877 or 127.0.0.1:9876
  -> SoftHandGrasping/vr_teleop.py
  -> ManiSkill simulation
```

## Repository Layout

```text
hand_tracking_streamer/           # Unity project for the headset app
scripts/
  hand_bridge.py                  # hand landmarks -> Shadow hand packet
  controller_bridge.py            # controller pose -> controller packet
  sockets.py                      # raw packet listener
  visualizer.py                   # stream visualizer
  test_bridge_logic.py            # packet contract checks
hand_tracking_streamer.apk        # prebuilt Quest APK
CONNECTIONS.md                    # stream format notes
```

## Working Steps

1. Start the Quest app on the headset.
2. Choose hand mode or controller mode.
3. Run the matching bridge script on the host.
4. Start `vr_teleop.py` in `SoftHandGrasping`.
5. Use the bridge logs first, then the teleop logs, to debug packet flow.

## Setup

```bash
uv sync
```

If not using `uv`:

```bash
python3 -m pip install numpy matplotlib
```

## Hand Tracking Bridge

Use this when the Quest app is in hand tracking mode:

```bash
adb reverse tcp:8000 tcp:8000

python3 scripts/hand_bridge.py \
  --in-protocol tcp \
  --in-port 8000 \
  --out-host 127.0.0.1 \
  --out-port 9877 \
  --shadow-joint-packet \
  --thumb-debug \
  --finger-debug
```

What it sends:

- Wrist pose for left and right hand.
- 21 hand landmarks per hand.
- Finger curl values.
- Optional Shadow joint packet with detailed per-joint retargeting features.

## Controller Bridge

Use this when the Quest app is in controller mode:

```bash
python3 scripts/controller_bridge.py \
  --in-protocol tcp \
  --in-port 8000 \
  --out-host 127.0.0.1 \
  --out-port 9876
```

## Debug Commands

Print raw TCP traffic:

```bash
python3 scripts/sockets.py --protocol tcp --host localhost --port 8000
```

Print raw UDP traffic:

```bash
python3 scripts/sockets.py --protocol udp --host 0.0.0.0 --port 9000
```

Visualize hand landmarks:

```bash
python3 scripts/visualizer.py --protocol tcp --host localhost --port 8000 --show-fingers
```

## Stream Format Notes

- Unity sends UTF-8 CSV lines.
- Hand tracking uses wrist plus 21 landmarks.
- Controller streaming uses controller pose plus grasp state.
- The Python bridges convert those lines into binary UDP packets for ManiSkill.

## What To Read First

For a new engineer or agent, the useful order is:

1. `README.md`
2. `scripts/hand_bridge.py`
3. `scripts/controller_bridge.py`
4. `CONNECTIONS.md`
5. `SoftHandGrasping/vr_teleop.py`

## Troubleshooting

- If packets do not arrive, check `adb reverse` and the Quest IP/port settings.
- If hand motion is wrong, inspect the bridge debug logs first.
- If the simulator receives packets but the motion is wrong, inspect `vr_teleop.py`.
- If you need to retune Shadow joint mappings, keep the bridge and simulator README in sync.

## Attribution And License

This project is based on the original Hand Tracking Streamer project by Zhengyang K. Weng and keeps the Apache-2.0 license. See `LICENSE` for details.
