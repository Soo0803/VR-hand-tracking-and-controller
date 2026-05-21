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
_PACK_FMT = "<26f"
_RETARGET_PACK_FMT = "<32f"
_PACK_SIZE = struct.calcsize(_PACK_FMT)
_RETARGET_PACK_SIZE = struct.calcsize(_RETARGET_PACK_FMT)


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
    middle_mcp = landmarks[9]
    pinky_mcp = landmarks[17]
    thumb_tip = landmarks[4]
    index_tip = landmarks[8]
    middle_tip = landmarks[12]
    pinky_tip = landmarks[20]

    palm_width = np.linalg.norm(index_mcp - pinky_mcp)
    if palm_width < 1e-8:
        return (0.0, 0.0, 0.0)

    pinch_target = 0.5 * (index_tip + middle_tip)
    thumb_distance = np.linalg.norm(thumb_tip - pinch_target) / palm_width
    thumb_opposition = 1.0 - _distance_to_unit_interval(thumb_distance, 0.45, 1.5)

    tip_spread = np.linalg.norm(index_tip - pinky_tip) / palm_width
    finger_spread = _distance_to_unit_interval(tip_spread, 0.7, 1.9)

    palm_center = np.mean(landmarks[[0, 5, 9, 13, 17]], axis=0)
    pinky_to_palm = np.linalg.norm(pinky_tip - palm_center) / palm_width
    palm_cup = 1.0 - _distance_to_unit_interval(pinky_to_palm, 0.65, 1.6)

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
):
    receiver = Receiver(in_protocol, "0.0.0.0", in_port)
    t = threading.Thread(target=receiver.run, daemon=True)
    t.start()

    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out_addr = (out_host, out_port)

    logging.info("Hand Bridge started. Forwarding to UDP %s:%d", out_host, out_port)

    try:
        while True:
            with receiver._lock:
                if shadow_retarget_packet:
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
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    run_bridge(
        args.in_protocol,
        args.in_port,
        args.out_port,
        args.out_host,
        args.verbose,
        args.shadow_retarget_packet,
    )


if __name__ == "__main__":
    main()
