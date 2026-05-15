#!/usr/bin/env python3
"""Hand Tracking Bridge: Quest hand-tracking stream → finger curl packet → Franka simulation.

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
    12           right_qx        (quaternion, RH frame)
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

# 26 floats = 104 bytes
_PACK_FMT = "<26f"
_PACK_SIZE = struct.calcsize(_PACK_FMT)


@dataclass
class HandPose:
    """State for a single hand (wrist + finger curls)."""
    tracked: bool = False
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
            target.px, target.py, target.pz = _convert_position(
                raw_vals[0], raw_vals[1], raw_vals[2]
            )
            target.qx, target.qy, target.qz, target.qw = _convert_quaternion(
                raw_vals[3], raw_vals[4], raw_vals[5], raw_vals[6]
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
            fallback_curl = receiver.fist_state
            target.thumb_curl = max(curls[0], fallback_curl)
            target.index_curl = max(curls[1], fallback_curl)
            target.middle_curl = max(curls[2], fallback_curl)
            target.ring_curl = max(curls[3], fallback_curl)
            target.pinky_curl = max(curls[4], fallback_curl)
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


def run_bridge(in_protocol: str, in_port: int, out_port: int, out_host: str,
               verbose: bool = False):
    receiver = Receiver(in_protocol, "0.0.0.0", in_port)
    t = threading.Thread(target=receiver.run, daemon=True)
    t.start()

    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out_addr = (out_host, out_port)

    logging.info("Hand Bridge started. Forwarding to UDP %s:%d", out_host, out_port)

    try:
        while True:
            with receiver._lock:
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
                        "R:%d Curls[T:%.2f I:%.2f M:%.2f R:%.2f P:%.2f] | "
                        "L:%d Curls[T:%.2f I:%.2f M:%.2f R:%.2f P:%.2f]",
                        receiver.right.tracked,
                        receiver.right.thumb_curl, receiver.right.index_curl,
                        receiver.right.middle_curl, receiver.right.ring_curl,
                        receiver.right.pinky_curl,
                        receiver.left.tracked,
                        receiver.left.thumb_curl, receiver.left.index_curl,
                        receiver.left.middle_curl, receiver.left.ring_curl,
                        receiver.left.pinky_curl,
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
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    run_bridge(args.in_protocol, args.in_port, args.out_port, args.out_host, args.verbose)


if __name__ == "__main__":
    main()
