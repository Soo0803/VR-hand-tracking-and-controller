#!/usr/bin/env python3
"""Hand Tracking Bridge: Quest hand-tracking stream -> finger curl packet -> Franka simulation.

Receives hand-tracking data (wrist pose + finger landmarks) from Hand Tracking
Streamer (HTS) over TCP or UDP, computes per-finger curl values from the 21
landmark positions, and forwards a packed binary float32 array to a local UDP
port consumed by vr_teleop.py in hand-tracking mode.

Output packet layout (little-endian float32):
    Byte offset  Field
    ──────────── ──────────────────────────
     0           right_px        (meters, RH frame)
     4           right_py
     8           right_pz
    12           right_qx        (palm quaternion, RH frame)
    16           right_qy
    20           right_qz
    24           right_qw
    28           right_tracked   (1.0 or 0.0)
    32           right_thumb_curl    (0.0=open, 1.0=closed)
    36           right_index_curl
    40           right_middle_curl
    44           right_ring_curl
    48           right_pinky_curl
    52           left_px
    56           left_py
    60           left_pz
    64           left_qx
    68           left_qy
    72           left_qz
    76           left_qw
    80           left_tracked
    84           left_thumb_curl
    88           left_index_curl
    92           left_middle_curl
    96           left_ring_curl
   100           left_pinky_curl
    ──────────── ──────────────────────────
    Total: 26 × float32 = 104 bytes

Usage:
    # Receive on TCP 8000 (USB / adb reverse), forward to UDP 9877
    python hand_bridge.py --in-protocol tcp --in-port 8000 --out-port 9877

    # Print parsed values to terminal for debugging
    python hand_bridge.py --in-protocol tcp --in-port 8000 --out-port 9877 --verbose
"""

from __future__ import annotations

import argparse
import logging
import math
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

# 26 floats = 104 bytes. Retarget mode sends 32 floats = 128 bytes.
# Shadow joint mode sends 80 floats = 320 bytes:
#   16-float retarget block + 24 normalized Shadow joint controls per hand.
_PACK_FMT = "<26f"
_RETARGET_PACK_FMT = "<32f"
_SHADOW_JOINT_PACK_FMT = "<80f"
_PACK_SIZE = struct.calcsize(_PACK_FMT)
_RETARGET_PACK_SIZE = struct.calcsize(_RETARGET_PACK_FMT)
_SHADOW_JOINT_PACK_SIZE = struct.calcsize(_SHADOW_JOINT_PACK_FMT)

SHADOW_JOINT_NAMES = (
    "WRJ1", "WRJ2",
    "FFJ1", "FFJ2", "FFJ3", "FFJ4",
    "MFJ1", "MFJ2", "MFJ3", "MFJ4",
    "RFJ1", "RFJ2", "RFJ3", "RFJ4",
    "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5",
    "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
)


@dataclass
class HandPose:
    """State for a single hand (wrist + finger curls)."""
    tracked: bool = False
    wrist_px: float = 0.0
    wrist_py: float = 0.0
    wrist_pz: float = 0.0
    wrist_qx: float = 0.0
    wrist_qy: float = 0.0
    wrist_qz: float = 0.0
    wrist_qw: float = 1.0
    px: float = 0.0
    py: float = 0.0
    pz: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    # Per-finger curl: 0.0 = fully open, 1.0 = fully closed
    thumb_curl: float = 0.0
    index_curl: float = 0.0
    middle_curl: float = 0.0
    ring_curl: float = 0.0
    pinky_curl: float = 0.0
    # Extra landmark-derived features for Shadow Hand retargeting.
    thumb_opposition: float = 0.0
    finger_spread: float = 0.0
    palm_cup: float = 0.0
    shadow_joint_targets: np.ndarray = field(default_factory=lambda: np.zeros(len(SHADOW_JOINT_NAMES), dtype=np.float32))
    thumb_joint_angles: np.ndarray = field(default_factory=lambda: np.full(3, np.nan, dtype=np.float32))
    finger_joint_angles: np.ndarray = field(default_factory=lambda: np.full((4, 3), np.nan, dtype=np.float32))
    # Raw landmark positions (21 joints × 3 coords = 63 floats)
    landmarks: Optional[np.ndarray] = None


def _convert_position(ux: float, uy: float, uz: float) -> Tuple[float, float, float]:
    """Unity LH (x-right, y-up, z-forward) -> RH (x-front, y-left, z-up).
    
    NOTE: This mapping is for the ManiSkill SIMULATION.
    """
    return (uz, -ux, uy)


def _convert_quaternion(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float, float]:
    """Unity LH Quat -> RH Quat (for ManiSkill simulation)."""
    return (-qz, qx, -qy, qw)


def _convert_landmark_position(ux: float, uy: float, uz: float) -> Tuple[float, float, float]:
    """Convert a landmark position from Unity LH to RH frame (same as wrist)."""
    return (uz, -ux, uy)


def _normalize_vector(v: np.ndarray) -> Optional[np.ndarray]:
    norm = np.linalg.norm(v)
    if norm < 1e-8:
        return None
    return v / norm


def _rotation_matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2, 1] - matrix[1, 2]) / s
        y = (matrix[0, 2] - matrix[2, 0]) / s
        z = (matrix[1, 0] - matrix[0, 1]) / s
    else:
        diag = np.diag(matrix)
        idx = int(np.argmax(diag))
        if idx == 0:
            s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / s
            x = 0.25 * s
            y = (matrix[0, 1] + matrix[1, 0]) / s
            z = (matrix[0, 2] + matrix[2, 0]) / s
        elif idx == 1:
            s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / s
            x = (matrix[0, 1] + matrix[1, 0]) / s
            y = 0.25 * s
            z = (matrix[1, 2] + matrix[2, 1]) / s
        else:
            s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / s
            x = (matrix[0, 2] + matrix[2, 0]) / s
            y = (matrix[1, 2] + matrix[2, 1]) / s
            z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / max(np.linalg.norm(q), 1e-8)


def _quaternion_wxyz_to_matrix(quaternion_wxyz: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion_wxyz, dtype=np.float64).reshape(4)
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        return np.eye(3, dtype=np.float64)
    w, x, y, z = q / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _compose_quaternion_wxyz(parent_wxyz: np.ndarray, child_wxyz: np.ndarray) -> np.ndarray:
    parent_matrix = _quaternion_wxyz_to_matrix(parent_wxyz)
    child_matrix = _quaternion_wxyz_to_matrix(child_wxyz)
    return _rotation_matrix_to_quaternion(parent_matrix @ child_matrix)


def _rotate_vector_wxyz(quaternion_wxyz: np.ndarray, vector: np.ndarray) -> np.ndarray:
    rotation = _quaternion_wxyz_to_matrix(quaternion_wxyz)
    return rotation @ np.asarray(vector, dtype=np.float64).reshape(3)


def _shadow_palm_alignment_wxyz() -> np.ndarray:
    # Human landmark frame:
    #   x = lateral, y = distal, z = normal.
    # Shadow palm_link frame:
    #   x = lateral, y = normal, z = distal.
    half_pi = 0.5 * math.pi
    return _rotation_matrix_to_quaternion(
        np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, math.cos(half_pi), -math.sin(half_pi)],
                [0.0, math.sin(half_pi), math.cos(half_pi)],
            ],
            dtype=np.float64,
        )
    )


def compute_palm_pose_from_landmarks(
    landmarks: np.ndarray,
    handedness: str = "right",
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    if landmarks is None or len(landmarks) < 21:
        return None

    wrist = landmarks[0]
    index_mcp = landmarks[5]
    middle_mcp = landmarks[9]
    pinky_mcp = landmarks[17]

    finger_center = np.mean(landmarks[[5, 9, 13, 17]], axis=0)
    distal_axis = _normalize_vector(finger_center - wrist)
    lateral_axis = _normalize_vector(index_mcp - pinky_mcp)
    if distal_axis is None or lateral_axis is None:
        return None

    if handedness.lower() == "left":
        palm_normal = _normalize_vector(np.cross(distal_axis, lateral_axis))
    else:
        palm_normal = _normalize_vector(np.cross(lateral_axis, distal_axis))
    if palm_normal is None:
        return None

    lateral_axis = _normalize_vector(np.cross(distal_axis, palm_normal))
    distal_axis = _normalize_vector(np.cross(palm_normal, lateral_axis))
    if lateral_axis is None or distal_axis is None:
        return None

    rotation = np.column_stack([lateral_axis, distal_axis, palm_normal])
    if not np.all(np.isfinite(rotation)):
        return None

    position = np.median(landmarks[[0, 5, 9, 13, 17]], axis=0)
    quaternion_wxyz = _rotation_matrix_to_quaternion(rotation)
    return position, quaternion_wxyz


def _angle_between_vectors(v1: np.ndarray, v2: np.ndarray) -> float:
    """Compute the angle (in radians) between two 3D vectors."""
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-8 or n2 < 1e-8:
        return 0.0
    cos_angle = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return float(np.arccos(cos_angle))


def _joint_bend_amount(
    p0: np.ndarray,
    p1: np.ndarray,
    p2: np.ndarray,
    open_angle: float = 0.0,
    closed_angle: float = math.pi / 2.0,
) -> float:
    angle = _angle_between_vectors(p1 - p0, p2 - p1)
    return _distance_to_unit_interval(angle, open_angle, closed_angle)


def _spread_amount(
    palm_lateral: np.ndarray,
    palm_distal: np.ndarray,
    finger_mcp: np.ndarray,
    finger_tip: np.ndarray,
    open_angle: float,
    closed_angle: float,
) -> float:
    direction = _safe_unit(finger_tip - finger_mcp)
    if direction is None:
        return 0.0
    x = float(np.dot(direction, palm_lateral))
    y = float(np.dot(direction, palm_distal))
    angle = math.atan2(x, y)
    return _distance_to_unit_interval(angle, open_angle, closed_angle)


def _mcp_flex_amount(
    palm_distal: np.ndarray,
    palm_normal: np.ndarray,
    finger_mcp: np.ndarray,
    finger_pip: np.ndarray,
    open_angle: float = 0.06,
    closed_angle: float = 0.82,
) -> float:
    """Estimate MCP/base flexion from the proximal phalanx in the palm frame.

    The previous wrist-MCP-PIP angle is weak for MCP flexion because wrist->MCP
    mostly lies inside the palm and does not represent a true metacarpal axis.
    This measures how much MCP->PIP leaves the palm distal direction and bends
    into/out of the palm normal direction.
    """
    direction = _safe_unit(finger_pip - finger_mcp)
    if direction is None:
        return 0.0
    distal_component = max(float(np.dot(direction, palm_distal)), 1e-6)
    normal_component = abs(float(np.dot(direction, palm_normal)))
    angle = math.atan2(normal_component, distal_component)
    return _distance_to_unit_interval(angle, open_angle, closed_angle)


def _finger_mcp_curl_amount(
    palm_distal: np.ndarray,
    finger_mcp: np.ndarray,
    finger_tip: np.ndarray,
    open_angle: float = 0.05,
    closed_angle: float = 0.92,
) -> float:
    """Estimate whole-finger MCP flexion from the MCP-to-tip direction."""
    direction = _safe_unit(finger_tip - finger_mcp)
    if direction is None:
        return 0.0
    distal_component = float(np.dot(direction, palm_distal))
    angle = math.acos(float(np.clip(distal_component, -1.0, 1.0)))
    return _distance_to_unit_interval(angle, open_angle, closed_angle)


def _point_to_palm_amount(
    point: np.ndarray,
    palm_center: np.ndarray,
    palm_width: float,
    open_distance: float,
    closed_distance: float,
) -> float:
    if palm_width < 1e-8:
        return 0.0
    distance = float(np.linalg.norm(point - palm_center) / palm_width)
    return _distance_to_unit_interval(distance, open_distance, closed_distance)


def _thumb_across_palm_amount(
    palm_lateral: np.ndarray,
    thumb_base: np.ndarray,
    thumb_tip: np.ndarray,
    palm_width: float,
) -> float:
    """Return 0 for an open/lateral thumb and 1 when it crosses into the palm."""
    if palm_width < 1e-8:
        return 0.0
    across = -float(np.dot(thumb_tip - thumb_base, palm_lateral)) / palm_width
    return _distance_to_unit_interval(across, -0.55, 0.18)


def compute_shadow_joint_targets_from_landmarks(landmarks: np.ndarray, handedness: str = "right") -> np.ndarray:
    """Compute normalized Shadow joint controls from 21 hand landmarks.

    Outputs are in SHADOW_JOINT_NAMES order and normalized to [0, 1].  They are
    later mapped to actual Shadow joint ranges by vr_teleop.py.
    """
    targets = np.zeros(len(SHADOW_JOINT_NAMES), dtype=np.float32)
    if landmarks is None or len(landmarks) < 21:
        return targets

    wrist = landmarks[0]
    index_mcp = landmarks[5]
    pinky_mcp = landmarks[17]
    finger_center = np.mean(landmarks[[5, 9, 13, 17]], axis=0)
    palm_distal = _safe_unit(finger_center - wrist)
    palm_lateral = _safe_unit(index_mcp - pinky_mcp)
    if palm_distal is None or palm_lateral is None:
        return targets
    if handedness.lower() == "left":
        palm_normal = _safe_unit(np.cross(palm_distal, palm_lateral))
    else:
        palm_normal = _safe_unit(np.cross(palm_lateral, palm_distal))
    if palm_normal is None:
        return targets

    palm_width = float(np.linalg.norm(index_mcp - pinky_mcp))
    if palm_width < 1e-8:
        return targets
    palm_center = np.mean(landmarks[[0, 5, 9, 13, 17]], axis=0)

    name_to_idx = {name: idx for idx, name in enumerate(SHADOW_JOINT_NAMES)}

    finger_specs = {
        "FF": (5, 6, 7, 8, -0.18, 0.16),
        "MF": (9, 10, 11, 12, -0.05, 0.05),
        "RF": (13, 14, 15, 16, 0.08, -0.08),
        "LF": (17, 18, 19, 20, 0.18, 0.30),
    }
    for prefix, (mcp_i, pip_i, dip_i, tip_i, open_spread, closed_spread) in finger_specs.items():
        mcp, pip, dip, tip = landmarks[mcp_i], landmarks[pip_i], landmarks[dip_i], landmarks[tip_i]
        proximal_flex = _mcp_flex_amount(
            palm_distal,
            palm_normal,
            mcp,
            pip,
            open_angle=0.04,
            closed_angle=0.72,
        )
        whole_finger_flex = _finger_mcp_curl_amount(palm_distal, mcp, tip)
        mcp_flex = max(proximal_flex, whole_finger_flex)
        if prefix == "LF":
            mcp_flex = min(1.0, 1.18 * mcp_flex)
        targets[name_to_idx[f"{prefix}J3"]] = mcp_flex
        targets[name_to_idx[f"{prefix}J2"]] = _joint_bend_amount(mcp, pip, dip)
        targets[name_to_idx[f"{prefix}J1"]] = _joint_bend_amount(pip, dip, tip)
        if prefix != "MF":
            spread = _spread_amount(
                palm_lateral,
                palm_distal,
                mcp,
                tip,
                open_spread,
                closed_spread,
            )
            if prefix == "LF":
                spread = max(spread, 0.65 * mcp_flex)
            targets[name_to_idx[f"{prefix}J4"]] = spread

    thumb_base = landmarks[1]
    thumb_mcp = landmarks[2]
    thumb_ip = landmarks[3]
    thumb_tip = landmarks[4]
    thumb_across_palm = _thumb_across_palm_amount(
        palm_lateral,
        thumb_base,
        thumb_tip,
        palm_width,
    )
    thumb_ip_to_palm = _point_to_palm_amount(
        thumb_ip,
        palm_center,
        palm_width,
        open_distance=1.05,
        closed_distance=0.48,
    )
    thumb_mcp_to_palm = _point_to_palm_amount(
        thumb_mcp,
        palm_center,
        palm_width,
        open_distance=0.78,
        closed_distance=0.36,
    )
    thumb_base_flex = max(thumb_ip_to_palm, 0.65 * thumb_across_palm, thumb_mcp_to_palm)
    targets[name_to_idx["THJ3"]] = thumb_base_flex
    targets[name_to_idx["THJ2"]] = _joint_bend_amount(
        thumb_base,
        thumb_mcp,
        thumb_ip,
        closed_angle=0.80,
    )
    targets[name_to_idx["THJ1"]] = _joint_bend_amount(
        thumb_mcp,
        thumb_ip,
        thumb_tip,
        closed_angle=0.55,
    )

    pinch_target = 0.5 * (landmarks[8] + landmarks[12])
    thumb_distance = np.linalg.norm(thumb_tip - pinch_target) / palm_width
    pinch_opposition = 1.0 - _distance_to_unit_interval(thumb_distance, 0.45, 1.5)
    thumb_to_palm = np.linalg.norm(thumb_tip - palm_center) / palm_width
    palm_opposition = 1.0 - _distance_to_unit_interval(thumb_to_palm, 0.55, 1.35)
    thumb_opposition = max(pinch_opposition, palm_opposition, thumb_across_palm)
    targets[name_to_idx["THJ4"]] = thumb_opposition
    targets[name_to_idx["THJ5"]] = thumb_opposition

    pinky_to_palm = np.linalg.norm(landmarks[20] - palm_center) / palm_width
    pinky_cup = 1.0 - _distance_to_unit_interval(pinky_to_palm, 0.45, 1.25)
    targets[name_to_idx["LFJ5"]] = 0.45 * pinky_cup

    return np.clip(targets, 0.0, 1.0).astype(np.float32)


def compute_thumb_joint_angles_from_landmarks(landmarks: np.ndarray) -> np.ndarray:
    """Return thumb IP, MCP, and base bend angles in radians for debugging."""
    if landmarks is None or len(landmarks) < 21:
        return np.full(3, np.nan, dtype=np.float32)
    thumb_base = landmarks[1]
    thumb_mcp = landmarks[2]
    thumb_ip = landmarks[3]
    thumb_tip = landmarks[4]
    return np.asarray(
        [
            _angle_between_vectors(thumb_ip - thumb_mcp, thumb_tip - thumb_ip),
            _angle_between_vectors(thumb_mcp - thumb_base, thumb_ip - thumb_mcp),
            _angle_between_vectors(thumb_base - landmarks[0], thumb_mcp - thumb_base),
        ],
        dtype=np.float32,
    )


def compute_finger_joint_angles_from_landmarks(landmarks: np.ndarray) -> np.ndarray:
    """Return DIP, PIP, MCP bend angles in radians for index/middle/ring/little."""
    values = np.full((4, 3), np.nan, dtype=np.float32)
    if landmarks is None or len(landmarks) < 21:
        return values
    finger_indices = (
        (5, 6, 7, 8),
        (9, 10, 11, 12),
        (13, 14, 15, 16),
        (17, 18, 19, 20),
    )
    wrist = landmarks[0]
    for row, (mcp_i, pip_i, dip_i, tip_i) in enumerate(finger_indices):
        mcp = landmarks[mcp_i]
        pip = landmarks[pip_i]
        dip = landmarks[dip_i]
        tip = landmarks[tip_i]
        values[row] = np.asarray(
            [
                _angle_between_vectors(dip - pip, tip - dip),
                _angle_between_vectors(pip - mcp, dip - pip),
                _angle_between_vectors(mcp - wrist, pip - mcp),
            ],
            dtype=np.float32,
        )
    return values


def compute_finger_curls(landmarks: np.ndarray) -> Tuple[float, float, float, float, float]:
    """Compute per-finger curl values from the 21 landmark positions.

    Landmark layout (21 joints, each 3D position relative to wrist):
        [0-4]:   Thumb  (base, metacarpal, proximal, distal, tip)
        [5-8]:   Index  (metacarpal, proximal, intermediate, tip)
        [9-12]:  Middle (metacarpal, proximal, intermediate, tip)
        [13-16]: Ring   (metacarpal, proximal, intermediate, tip)
        [17-20]: Pinky  (metacarpal, proximal, intermediate, tip)

    Curl is computed as the average bend angle between consecutive bone
    segments, normalized to [0, 1] where 0=straight and 1=90° bend.
    """
    if landmarks is None or len(landmarks) < 21:
        return (0.0, 0.0, 0.0, 0.0, 0.0)

    def finger_curl(joint_indices: list) -> float:
        """Compute curl for a finger given its joint indices in the landmarks array."""
        if len(joint_indices) < 3:
            return 0.0
        
        angles = []
        for i in range(len(joint_indices) - 2):
            p0 = landmarks[joint_indices[i]]
            p1 = landmarks[joint_indices[i + 1]]
            p2 = landmarks[joint_indices[i + 2]]
            v1 = p1 - p0
            v2 = p2 - p1
            angle = _angle_between_vectors(v1, v2)
            angles.append(angle)
        
        if not angles:
            return 0.0
        
        avg_angle = sum(angles) / len(angles)
        # Normalize: 0 rad = straight (curl 0), pi/2 rad = fully bent (curl 1)
        curl = float(np.clip(avg_angle / (math.pi / 2.0), 0.0, 1.0))
        return curl

    thumb_curl = finger_curl([0, 1, 2, 3, 4])
    index_curl = finger_curl([5, 6, 7, 8])
    middle_curl = finger_curl([9, 10, 11, 12])
    ring_curl = finger_curl([13, 14, 15, 16])
    pinky_curl = finger_curl([17, 18, 19, 20])

    return (thumb_curl, index_curl, middle_curl, ring_curl, pinky_curl)


def _safe_unit(v: np.ndarray) -> Optional[np.ndarray]:
    norm = np.linalg.norm(v)
    if norm < 1e-8:
        return None
    return v / norm


def _distance_to_unit_interval(value: float, open_value: float, closed_value: float) -> float:
    denom = closed_value - open_value
    if abs(denom) < 1e-8:
        return 0.0
    return float(np.clip((value - open_value) / denom, 0.0, 1.0))


def compute_shadow_retarget_features(landmarks: np.ndarray) -> Tuple[float, float, float]:
    """Compute three coarse retargeting controls from 21 hand landmarks.

    The outputs are normalized to [0, 1]:
      thumb_opposition: thumb tip moving toward the index/middle fingertip region
      finger_spread: lateral distance between index and pinky fingertips
      palm_cup: pinky metacarpal/tip folding toward the palm
    """
    if landmarks is None or len(landmarks) < 21:
        return (0.0, 0.0, 0.0)

    wrist = landmarks[0]
    index_mcp = landmarks[5]
    pinky_mcp = landmarks[17]
    thumb_base = landmarks[1]
    thumb_tip = landmarks[4]
    index_tip = landmarks[8]
    middle_tip = landmarks[12]
    pinky_tip = landmarks[20]

    palm_width = np.linalg.norm(index_mcp - pinky_mcp)
    if palm_width < 1e-8:
        return (0.0, 0.0, 0.0)
    finger_center = np.mean(landmarks[[5, 9, 13, 17]], axis=0)
    palm_distal = _safe_unit(finger_center - wrist)
    palm_lateral = _safe_unit(index_mcp - pinky_mcp)
    if palm_distal is None or palm_lateral is None:
        return (0.0, 0.0, 0.0)

    pinch_target = 0.5 * (index_tip + middle_tip)
    thumb_distance = np.linalg.norm(thumb_tip - pinch_target) / palm_width
    thumb_opposition = 1.0 - _distance_to_unit_interval(thumb_distance, 0.45, 1.5)
    palm_center = np.mean(landmarks[[0, 5, 9, 13, 17]], axis=0)
    thumb_to_palm = np.linalg.norm(thumb_tip - palm_center) / palm_width
    palm_opposition = 1.0 - _distance_to_unit_interval(thumb_to_palm, 0.55, 1.35)
    thumb_across_palm = _thumb_across_palm_amount(
        palm_lateral,
        thumb_base,
        thumb_tip,
        float(palm_width),
    )
    thumb_opposition = max(thumb_opposition, palm_opposition, thumb_across_palm)

    tip_spread = np.linalg.norm(index_tip - pinky_tip) / palm_width
    finger_spread = _distance_to_unit_interval(tip_spread, 0.7, 1.9)

    pinky_to_palm = np.linalg.norm(pinky_tip - palm_center) / palm_width
    palm_cup = 0.45 * (1.0 - _distance_to_unit_interval(pinky_to_palm, 0.45, 1.25))

    return (
        float(np.clip(thumb_opposition, 0.0, 1.0)),
        float(np.clip(finger_spread, 0.0, 1.0)),
        float(np.clip(palm_cup, 0.0, 1.0)),
    )


def _parse_line(line: str, receiver: "Receiver") -> Optional[str]:
    """Parse a CSV line from HTS and update the corresponding hand."""
    parts = [p.strip() for p in line.split(",")]
    if not parts:
        return None

    header = parts[0].lower()

    # Handle Fist State messages (fallback grasp detection)
    if "fist:" in header:
        state = parts[0].split(":")[1].strip().lower()
        receiver.fist_state = 1.0 if state == "closed" else 0.0
        return "fist"

    # Determine which hand
    side = "right" if "right" in header else "left" if "left" in header else None
    if side is None:
        return None

    target = receiver.right if side == "right" else receiver.left

    try:
        raw_vals = []
        for p in parts[1:]:
            try:
                raw_vals.append(float(p))
            except ValueError:
                continue

        # Wrist pose message
        # HTS sends: "Right wrist:, px, py, pz, qx, qy, qz, qw" (7 floats, NO tracked flag)
        if "wrist" in header:
            if len(raw_vals) < 7:
                return None
            target.tracked = True  # If we receive wrist data, the hand is tracked
            target.wrist_px, target.wrist_py, target.wrist_pz = _convert_position(
                raw_vals[0], raw_vals[1], raw_vals[2]
            )
            target.wrist_qx, target.wrist_qy, target.wrist_qz, target.wrist_qw = _convert_quaternion(
                raw_vals[3], raw_vals[4], raw_vals[5], raw_vals[6]
            )
            # Fallback until landmarks arrive: stream wrist pose.
            target.px, target.py, target.pz = target.wrist_px, target.wrist_py, target.wrist_pz
            wrist_quaternion_wxyz = np.array(
                [target.wrist_qw, target.wrist_qx, target.wrist_qy, target.wrist_qz],
                dtype=np.float64,
            )
            wrist_quaternion_wxyz = _compose_quaternion_wxyz(
                wrist_quaternion_wxyz,
                _shadow_palm_alignment_wxyz(),
            )
            target.qw, target.qx, target.qy, target.qz = (
                float(wrist_quaternion_wxyz[0]),
                float(wrist_quaternion_wxyz[1]),
                float(wrist_quaternion_wxyz[2]),
                float(wrist_quaternion_wxyz[3]),
            )
            return side

        # Landmark message
        if "landmarks" in header or "landmark" in header:
            # Expect 21 joints × 3 coords = 63 float values
            if len(raw_vals) < 63:
                return None
            
            # Parse into an array of 21 positions
            positions = []
            for i in range(21):
                ux, uy, uz = raw_vals[i * 3], raw_vals[i * 3 + 1], raw_vals[i * 3 + 2]
                rx, ry, rz = _convert_landmark_position(ux, uy, uz)
                positions.append(np.array([rx, ry, rz]))
            
            target.landmarks = np.array(positions)
            
            # Compute finger curls from the landmark positions
            curls = compute_finger_curls(target.landmarks)
            retarget_features = compute_shadow_retarget_features(target.landmarks)
            shadow_joint_targets = compute_shadow_joint_targets_from_landmarks(target.landmarks, handedness=side)
            thumb_joint_angles = compute_thumb_joint_angles_from_landmarks(target.landmarks)
            finger_joint_angles = compute_finger_joint_angles_from_landmarks(target.landmarks)
            palm_pose = compute_palm_pose_from_landmarks(target.landmarks, handedness=side)
            if palm_pose is not None:
                palm_position, palm_quaternion_wxyz = palm_pose
                wrist_position = np.array(
                    [target.wrist_px, target.wrist_py, target.wrist_pz],
                    dtype=np.float64,
                )
                wrist_quaternion_wxyz = np.array(
                    [target.wrist_qw, target.wrist_qx, target.wrist_qy, target.wrist_qz],
                    dtype=np.float64,
                )
                palm_position = wrist_position + _rotate_vector_wxyz(
                    wrist_quaternion_wxyz,
                    palm_position,
                )
                palm_quaternion_wxyz = _compose_quaternion_wxyz(
                    wrist_quaternion_wxyz,
                    palm_quaternion_wxyz,
                )
                palm_quaternion_wxyz = _compose_quaternion_wxyz(
                    palm_quaternion_wxyz,
                    _shadow_palm_alignment_wxyz(),
                )
                target.px, target.py, target.pz = (
                    float(palm_position[0]),
                    float(palm_position[1]),
                    float(palm_position[2]),
                )
                target.qw, target.qx, target.qy, target.qz = (
                    float(palm_quaternion_wxyz[0]),
                    float(palm_quaternion_wxyz[1]),
                    float(palm_quaternion_wxyz[2]),
                    float(palm_quaternion_wxyz[3]),
                )
            fallback_curl = receiver.fist_state
            target.thumb_curl = max(curls[0], fallback_curl)
            target.index_curl = max(curls[1], fallback_curl)
            target.middle_curl = max(curls[2], fallback_curl)
            target.ring_curl = max(curls[3], fallback_curl)
            target.pinky_curl = max(curls[4], fallback_curl)
            target.thumb_opposition = retarget_features[0]
            target.finger_spread = retarget_features[1]
            target.palm_cup = retarget_features[2]
            target.shadow_joint_targets = shadow_joint_targets
            target.thumb_joint_angles = thumb_joint_angles
            target.finger_joint_angles = finger_joint_angles
            return f"{side}_landmarks"

    except Exception as e:
        logging.error("Parse error: %s", e)
    
    return None


class Receiver:
    """Manages incoming socket and data state."""
    def __init__(self, protocol: str, host: str, port: int):
        self.protocol = protocol
        self.host = host
        self.port = port
        self.right = HandPose()
        self.left = HandPose()
        self.fist_state = 0.0
        self._stop = threading.Event()
        self._lock = threading.Lock()

    def run(self):
        if self.protocol == "tcp":
            self._run_tcp()
        else:
            self._run_udp()

    def _run_udp(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.host, self.port))
        sock.settimeout(0.5)
        logging.info("UDP Hand Bridge Listening on %s:%d", self.host, self.port)
        while not self._stop.is_set():
            try:
                data, _ = sock.recvfrom(65536)
                lines = data.decode("utf-8").splitlines()
                with self._lock:
                    for line in lines:
                        _parse_line(line, self)
            except socket.timeout:
                continue
            except Exception as e:
                logging.error("UDP error: %s", e)

    def _handle_tcp_conn(self, conn, addr):
        """Worker thread to handle a single TCP client."""
        logging.info("Connected to %s", addr)
        try:
            with conn:
                conn.settimeout(5.0)
                buffer = ""
                while not self._stop.is_set():
                    try:
                        data = conn.recv(8192)
                        if not data:
                            break
                        buffer += data.decode("utf-8", errors="ignore")
                        while "\n" in buffer:
                            line, buffer = buffer.split("\n", 1)
                            with self._lock:
                                _parse_line(line, self)
                    except socket.timeout:
                        continue
        except Exception as e:
            logging.error("Connection handler error (%s): %s", addr, e)
        finally:
            logging.info("Disconnected from %s", addr)

    def _run_tcp(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(5)
        server.settimeout(0.5)
        logging.info("TCP Hand Bridge Listening on %s:%d (Multi-Threaded)", self.host, self.port)

        while not self._stop.is_set():
            try:
                conn, addr = server.accept()
                t = threading.Thread(
                    target=self._handle_tcp_conn,
                    args=(conn, addr),
                    daemon=True,
                )
                t.start()
            except socket.timeout:
                continue
            except Exception as e:
                logging.error("TCP Server error: %s", e)
        server.close()


def run_bridge(
    in_protocol: str,
    in_port: int,
    out_port: int,
    out_host: str,
    verbose: bool = False,
    shadow_retarget_packet: bool = False,
    shadow_joint_packet: bool = False,
    thumb_debug: bool = False,
    thumb_debug_interval: float = 0.25,
    finger_debug: bool = False,
    finger_debug_interval: float = 0.25,
):
    receiver = Receiver(in_protocol, "0.0.0.0", in_port)
    t = threading.Thread(target=receiver.run, daemon=True)
    t.start()

    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out_addr = (out_host, out_port)

    logging.info("Hand Bridge started. Forwarding to UDP %s:%d", out_host, out_port)
    last_thumb_debug_time = 0.0
    last_finger_debug_time = 0.0

    try:
        while True:
            with receiver._lock:
                if shadow_joint_packet:
                    right_shadow = np.asarray(receiver.right.shadow_joint_targets, dtype=np.float32)
                    left_shadow = np.asarray(receiver.left.shadow_joint_targets, dtype=np.float32)
                    packet = struct.pack(
                        _SHADOW_JOINT_PACK_FMT,
                        # Right Hand (40 floats)
                        receiver.right.px, receiver.right.py, receiver.right.pz,
                        receiver.right.qx, receiver.right.qy, receiver.right.qz, receiver.right.qw,
                        float(receiver.right.tracked),
                        receiver.right.thumb_curl,
                        receiver.right.index_curl,
                        receiver.right.middle_curl,
                        receiver.right.ring_curl,
                        receiver.right.pinky_curl,
                        receiver.right.thumb_opposition,
                        receiver.right.finger_spread,
                        receiver.right.palm_cup,
                        *right_shadow.tolist(),
                        # Left Hand (40 floats)
                        receiver.left.px, receiver.left.py, receiver.left.pz,
                        receiver.left.qx, receiver.left.qy, receiver.left.qz, receiver.left.qw,
                        float(receiver.left.tracked),
                        receiver.left.thumb_curl,
                        receiver.left.index_curl,
                        receiver.left.middle_curl,
                        receiver.left.ring_curl,
                        receiver.left.pinky_curl,
                        receiver.left.thumb_opposition,
                        receiver.left.finger_spread,
                        receiver.left.palm_cup,
                        *left_shadow.tolist(),
                    )
                elif shadow_retarget_packet:
                    # Pack data: 32 floats = 128 bytes.
                    # Per hand: pos(3), quat(4), tracked, curls(5), retarget features(3)
                    packet = struct.pack(
                        _RETARGET_PACK_FMT,
                        # Right Hand (16 floats)
                        receiver.right.px, receiver.right.py, receiver.right.pz,
                        receiver.right.qx, receiver.right.qy, receiver.right.qz, receiver.right.qw,
                        float(receiver.right.tracked),
                        receiver.right.thumb_curl,
                        receiver.right.index_curl,
                        receiver.right.middle_curl,
                        receiver.right.ring_curl,
                        receiver.right.pinky_curl,
                        receiver.right.thumb_opposition,
                        receiver.right.finger_spread,
                        receiver.right.palm_cup,
                        # Left Hand (16 floats)
                        receiver.left.px, receiver.left.py, receiver.left.pz,
                        receiver.left.qx, receiver.left.qy, receiver.left.qz, receiver.left.qw,
                        float(receiver.left.tracked),
                        receiver.left.thumb_curl,
                        receiver.left.index_curl,
                        receiver.left.middle_curl,
                        receiver.left.ring_curl,
                        receiver.left.pinky_curl,
                        receiver.left.thumb_opposition,
                        receiver.left.finger_spread,
                        receiver.left.palm_cup,
                    )
                else:
                    # Pack data: 26 floats = 104 bytes
                    # Order: R_pos(3), R_quat(4), R_tracked, R_curls(5),
                    #         L_pos(3), L_quat(4), L_tracked, L_curls(5)
                    packet = struct.pack(
                        _PACK_FMT,
                        # Right Hand (13 floats)
                        receiver.right.px, receiver.right.py, receiver.right.pz,
                        receiver.right.qx, receiver.right.qy, receiver.right.qz, receiver.right.qw,
                        float(receiver.right.tracked),
                        receiver.right.thumb_curl,
                        receiver.right.index_curl,
                        receiver.right.middle_curl,
                        receiver.right.ring_curl,
                        receiver.right.pinky_curl,
                        # Left Hand (13 floats)
                        receiver.left.px, receiver.left.py, receiver.left.pz,
                        receiver.left.qx, receiver.left.qy, receiver.left.qz, receiver.left.qw,
                        float(receiver.left.tracked),
                        receiver.left.thumb_curl,
                        receiver.left.index_curl,
                        receiver.left.middle_curl,
                        receiver.left.ring_curl,
                        receiver.left.pinky_curl,
                    )

                now = time.time()
                if thumb_debug and now - last_thumb_debug_time >= max(float(thumb_debug_interval), 1e-3):
                    last_thumb_debug_time = now
                    right_shadow = np.asarray(receiver.right.shadow_joint_targets, dtype=np.float32)
                    left_shadow = np.asarray(receiver.left.shadow_joint_targets, dtype=np.float32)
                    right_angles = np.asarray(receiver.right.thumb_joint_angles, dtype=np.float32)
                    left_angles = np.asarray(receiver.left.thumb_joint_angles, dtype=np.float32)
                    right_thumb = right_shadow[19:24] if right_shadow.size >= 24 else np.zeros(5, dtype=np.float32)
                    left_thumb = left_shadow[19:24] if left_shadow.size >= 24 else np.zeros(5, dtype=np.float32)
                    logging.info(
                        "ThumbDebug Bridge | R tracked=%d curl=%.3f opp=%.3f "
                        "ang[IP,MCP,base]=[% .3f % .3f % .3f] THJ[1:5]=[% .3f % .3f % .3f % .3f % .3f] | "
                        "L tracked=%d curl=%.3f opp=%.3f "
                        "ang[IP,MCP,base]=[% .3f % .3f % .3f] THJ[1:5]=[% .3f % .3f % .3f % .3f % .3f]",
                        receiver.right.tracked,
                        receiver.right.thumb_curl,
                        receiver.right.thumb_opposition,
                        right_angles[0], right_angles[1], right_angles[2],
                        right_thumb[0], right_thumb[1], right_thumb[2], right_thumb[3], right_thumb[4],
                        receiver.left.tracked,
                        receiver.left.thumb_curl,
                        receiver.left.thumb_opposition,
                        left_angles[0], left_angles[1], left_angles[2],
                        left_thumb[0], left_thumb[1], left_thumb[2], left_thumb[3], left_thumb[4],
                    )

                if finger_debug and now - last_finger_debug_time >= max(float(finger_debug_interval), 1e-3):
                    last_finger_debug_time = now
                    right_shadow = np.asarray(receiver.right.shadow_joint_targets, dtype=np.float32)
                    left_shadow = np.asarray(receiver.left.shadow_joint_targets, dtype=np.float32)
                    right_angles = np.asarray(receiver.right.finger_joint_angles, dtype=np.float32)
                    left_angles = np.asarray(receiver.left.finger_joint_angles, dtype=np.float32)
                    finger_slices = (("FF", 2), ("MF", 6), ("RF", 10), ("LF", 14))
                    right_parts = []
                    left_parts = []
                    for row, (name, start) in enumerate(finger_slices):
                        right_vals = right_shadow[start:start + 3] if right_shadow.size >= start + 3 else np.full(3, np.nan)
                        left_vals = left_shadow[start:start + 3] if left_shadow.size >= start + 3 else np.full(3, np.nan)
                        right_parts.append(
                            f"{name} ang[DIP,PIP,MCP]={np.array2string(right_angles[row], precision=3, suppress_small=True)} "
                            f"J[1:3]={np.array2string(right_vals, precision=3, suppress_small=True)}"
                        )
                        left_parts.append(
                            f"{name} ang[DIP,PIP,MCP]={np.array2string(left_angles[row], precision=3, suppress_small=True)} "
                            f"J[1:3]={np.array2string(left_vals, precision=3, suppress_small=True)}"
                        )
                    logging.info(
                        "FingerDebug Bridge | R %s | L %s",
                        " | ".join(right_parts),
                        " | ".join(left_parts),
                    )

                if verbose and (receiver.right.tracked or receiver.left.tracked):
                    logging.info(
                        "R:%d Curls[T:%.2f I:%.2f M:%.2f R:%.2f P:%.2f] "
                        "Retarget[Opp:%.2f Spread:%.2f Cup:%.2f] | "
                        "L:%d Curls[T:%.2f I:%.2f M:%.2f R:%.2f P:%.2f] "
                        "Retarget[Opp:%.2f Spread:%.2f Cup:%.2f]",
                        receiver.right.tracked,
                        receiver.right.thumb_curl, receiver.right.index_curl,
                        receiver.right.middle_curl, receiver.right.ring_curl,
                        receiver.right.pinky_curl,
                        receiver.right.thumb_opposition, receiver.right.finger_spread,
                        receiver.right.palm_cup,
                        receiver.left.tracked,
                        receiver.left.thumb_curl, receiver.left.index_curl,
                        receiver.left.middle_curl, receiver.left.ring_curl,
                        receiver.left.pinky_curl,
                        receiver.left.thumb_opposition, receiver.left.finger_spread,
                        receiver.left.palm_cup,
                    )

            out_sock.sendto(packet, out_addr)
            time.sleep(1.0 / 60.0)  # 60Hz
    except KeyboardInterrupt:
        logging.info("Stopping hand bridge...")


def main():
    parser = argparse.ArgumentParser(description="Quest Hand Tracking to ManiSkill Bridge")
    parser.add_argument("--in-protocol", choices=["tcp", "udp"], default="tcp")
    parser.add_argument("--in-port", type=int, default=8000)
    parser.add_argument("--out-host", type=str, default="127.0.0.1")
    parser.add_argument("--out-port", type=int, default=9877)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument(
        "--shadow-retarget-packet",
        action="store_true",
        help="Send the 128-byte packet with extra Shadow Hand retargeting features.",
    )
    parser.add_argument(
        "--shadow-joint-packet",
        action="store_true",
        help="Send the 320-byte packet with per-joint Shadow Hand retargeting targets.",
    )
    parser.add_argument("--thumb-debug", action="store_true", help="Log raw per-thumb Shadow joint targets.")
    parser.add_argument("--thumb-debug-interval", type=float, default=0.25)
    parser.add_argument("--finger-debug", action="store_true", help="Log raw per-finger Shadow joint targets.")
    parser.add_argument("--finger-debug-interval", type=float, default=0.25)
    args = parser.parse_args()

    # Default to the Shadow joint packet so vr_teleop.py can drive the thumb
    # from per-joint landmark retargeting instead of the coarse curl heuristic.
    if not args.shadow_retarget_packet and not args.shadow_joint_packet:
        args.shadow_joint_packet = True
        print(
            "[HandBridge] No packet mode selected; defaulting to --shadow-joint-packet "
            "for more accurate Shadow thumb teleoperation.",
            flush=True,
        )

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    run_bridge(
        args.in_protocol,
        args.in_port,
        args.out_port,
        args.out_host,
        args.verbose,
        args.shadow_retarget_packet,
        args.shadow_joint_packet,
        args.thumb_debug,
        args.thumb_debug_interval,
        args.finger_debug,
        args.finger_debug_interval,
    )


if __name__ == "__main__":
    main()
