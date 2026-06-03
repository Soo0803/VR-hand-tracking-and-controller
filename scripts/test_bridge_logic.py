#!/usr/bin/env python3
"""Lightweight checks for controller_bridge.py packet parsing.

Run from the scripts directory:
    python3 test_bridge_logic.py
"""

import struct

import controller_bridge as bridge


def assert_close_tuple(actual, expected, eps=1e-6):
    assert len(actual) == len(expected)
    for got, want in zip(actual, expected):
        assert abs(got - want) <= eps, f"got {actual}, expected {expected}"


def pack_receiver(receiver):
    return struct.pack(
        bridge._PACK_FMT,
        receiver.right.px,
        receiver.right.py,
        receiver.right.pz,
        receiver.right.qx,
        receiver.right.qy,
        receiver.right.qz,
        receiver.right.qw,
        receiver.right.grasp,
        float(receiver.right.tracked),
        receiver.left.px,
        receiver.left.py,
        receiver.left.pz,
        receiver.left.qx,
        receiver.left.qy,
        receiver.left.qz,
        receiver.left.qw,
        receiver.left.grasp,
        float(receiver.left.tracked),
    )


def test_controller_parse_gripper_disabled():
    receiver = bridge.Receiver("tcp", "127.0.0.1", 8000, enable_gripper=False)
    line = (
        "Right controller:, "
        "1, "  # tracked
        "0.1, 0.2, 0.3, "  # position x,y,z
        "0.4, 0.5, 0.6, 0.7, "  # quaternion x,y,z,w
        "0, 0, 0, "  # forward
        "0, 0, 0, "  # up
        "0, 0, 0, "  # right
        "1"  # A/primary button pressed
    )

    parsed = bridge._parse_line(line, receiver)

    assert parsed == "right"
    assert receiver.right.tracked is True
    assert_close_tuple(
        (receiver.right.px, receiver.right.py, receiver.right.pz),
        (0.3, -0.1, 0.2),
    )
    # Rotation mapping follows the ManiSkill bridge packet convention.
    assert_close_tuple(
        (receiver.right.qx, receiver.right.qy, receiver.right.qz, receiver.right.qw),
        (0.6, -0.4, 0.5, 0.7),
    )
    # Default safety: pressing the Quest button does not close the gripper.
    assert receiver.right.grasp == 0.0


def test_controller_parse_gripper_enabled():
    receiver = bridge.Receiver("tcp", "127.0.0.1", 8000, enable_gripper=True)
    line = "Right controller:, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1"

    parsed = bridge._parse_line(line, receiver)

    assert parsed == "right"
    assert receiver.right.grasp == 1.0


def test_packet_contract():
    receiver = bridge.Receiver("tcp", "127.0.0.1", 8000, enable_gripper=True)
    bridge._parse_line(
        "Right controller:, 1, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1",
        receiver,
    )

    packet = pack_receiver(receiver)
    vals = struct.unpack(bridge._PACK_FMT, packet)

    assert bridge._PACK_SIZE == 72
    assert len(packet) == 72
    assert len(vals) == 18
    assert_close_tuple(vals[:9], (0.3, -0.1, 0.2, 0.6, -0.4, 0.5, 0.7, 1.0, 1.0))
    assert_close_tuple(vals[9:], (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0))


def test_controller_parse_current_unity_pose_first_format():
    receiver = bridge.Receiver("tcp", "127.0.0.1", 8000, enable_gripper=True)
    line = (
        "Right controller:, "
        "0.1, 0.2, 0.3, "  # position x,y,z
        "0.4, 0.5, 0.6, 0.7, "  # quaternion x,y,z,w
        "0, 0, 0, 0, 0, 0, 0, 0, 0, "
        "0.8"  # A/primary button field
    )

    parsed = bridge._parse_line(line, receiver)

    assert parsed == "right"
    assert receiver.right.tracked is True
    assert_close_tuple((receiver.right.px, receiver.right.py, receiver.right.pz), (0.3, -0.1, 0.2))
    assert_close_tuple((receiver.right.qx, receiver.right.qy, receiver.right.qz, receiver.right.qw), (0.6, -0.4, 0.5, 0.7))
    assert receiver.right.grasp == 1.0


def test_controller_primary_button_grasp_threshold_binary_output():
    receiver = bridge.Receiver("tcp", "127.0.0.1", 8000, enable_gripper=True, grasp_threshold=0.5)
    bridge._parse_line(
        "Right controller:, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1",
        receiver,
    )
    assert receiver.right.grasp == 0.0

    bridge._parse_line(
        "Right controller:, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.5",
        receiver,
    )
    assert receiver.right.grasp == 1.0


def main():
    test_controller_parse_gripper_disabled()
    test_controller_parse_gripper_enabled()
    test_packet_contract()
    test_controller_parse_current_unity_pose_first_format()
    test_controller_primary_button_grasp_threshold_binary_output()
    print("controller_bridge.py parse and packet contract tests passed")


if __name__ == "__main__":
    main()
