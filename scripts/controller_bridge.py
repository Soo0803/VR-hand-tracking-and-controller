#!/usr/bin/env python3
"""Controller Bridge: Quest controller stream -> raw float array -> Franka simulation.

Receives controller pose data from Hand Tracking Streamer (HTS) over TCP or
UDP, converts from Unity left-handed coordinates to a right-handed (Z-up)
frame, and forwards the result as a packed binary float32 array to a local
UDP port consumed by the robotics simulation.

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
    28           right_grasp     (1.0=Grasp)
    32           right_tracked   (1.0 or 0.0)
    36           left_px
    40           left_py
    44           left_pz
    48           left_qx
    52           left_qy
    56           left_qz
    60           left_qw
    64           left_grasp
    68           left_tracked
    ──────────── ──────────────────────────
    Total: 18 × float32 = 72 bytes

Usage:
    # Receive on TCP 8000 (USB / adb reverse), forward to UDP 9876
    python controller_bridge.py --in-protocol tcp --in-port 8000 --out-port 9876
"""

from __future__ import annotations

import argparse
import logging
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

# 18 floats = 72 bytes
_PACK_FMT = "<18f"
_PACK_SIZE = struct.calcsize(_PACK_FMT)

@dataclass
class ControllerPose:
    """State for a single controller."""
    tracked: bool = False
    px: float = 0.0
    py: float = 0.0
    pz: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    grasp: float = 0.0

def _convert_position(ux: float, uy: float, uz: float) -> Tuple[float, float, float]:
    """Unity LH (x-right, y-up, z-forward) -> RH (x-front, y-left, z-up).
    
    NOTE: This mapping is for the ManiSkill SIMULATION. The real Franka
    bridge (hand-tracking-streamer-current) uses a different mapping
    tuned for the FR3 hardware: (uz, uy, -ux).
    """
    # Standard Unity LH -> RH Z-up: [[0,0,1],[-1,0,0],[0,1,0]]
    return (uz, -ux, uy)

def _convert_quaternion(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float, float]:
    """Unity LH Quat -> RH Quat (for ManiSkill simulation).
    
    NOTE: This mapping is for the ManiSkill SIMULATION. The real Franka
    bridge uses (-qz, qx, -qy, qw) which accounts for the FR3's frame.
    """
    # Matches the standard position axis mapping (z, -x, y).
    return (qz, -qx, qy, qw)

def _parse_line(line: str, receiver: "Receiver") -> Optional[str]:
    """Parse a CSV line from HTS and update the corresponding side."""
    parts = [p.strip() for p in line.split(",")]
    if not parts:
        return None
    
    header = parts[0].lower()
    
    # Handle Hand Tracking Fist State messages
    if "fist:" in header:
        state = parts[0].split(":")[1].strip().lower()
        receiver.fist_state = 1.0 if state == "closed" else 0.0
        return "fist"

    # Handle Controller, Hand Tracking (Wrist), or Head Updates
    if "controller" not in header and "wrist" not in header and "head" not in header:
        return None

    side = "right" if "right" in header else "left" if "left" in header else None
    if side is None:
        return None

    target = receiver.right if side == "right" else receiver.left

    try:
        # Extract numeric values (skipping header)
        raw_vals = []
        for p in parts[1:]:
            try:
                raw_vals.append(float(p))
            except ValueError:
                continue

        if len(raw_vals) < 7:
            return None

        # Current Unity ControllerPoseStreamer sends pose-first:
        #   px, py, pz, qx, qy, qz, qw, padding..., grasp
        # Older bridge captures include a tracked flag first:
        #   tracked, px, py, pz, qx, qy, qz, qw, padding..., grasp
        if len(raw_vals) >= 18:
            tracked = raw_vals[0] > 0.5
            pos_offset = 1
            quat_offset = 4
        else:
            tracked = True
            pos_offset = 0
            quat_offset = 3

        target.px, target.py, target.pz = _convert_position(
            raw_vals[pos_offset],
            raw_vals[pos_offset + 1],
            raw_vals[pos_offset + 2],
        )
        
        target.qx, target.qy, target.qz, target.qw = _convert_quaternion(
            raw_vals[quat_offset],
            raw_vals[quat_offset + 1],
            raw_vals[quat_offset + 2],
            raw_vals[quat_offset + 3],
        )
        
        # Grasp signal. ControllerPoseStreamer sends a binary primary face
        # button value here: right A, and the matching primary button on left.
        # Keep disabled by default while testing arm teleop so a button press
        # cannot accidentally command a hard close.
        if not receiver.enable_gripper:
            target.grasp = 0.0
        elif "wrist" in header:
            # For hands, use the global fist state tracked by FistTracking.cs
            target.grasp = receiver.fist_state
        else:
            raw_grasp = float(max(0.0, min(1.0, raw_vals[-1])))
            target.grasp = 1.0 if raw_grasp >= receiver.grasp_threshold else 0.0
        
        target.tracked = tracked
        return side
    except Exception as e:
        logging.error("Parse error: %s", e)
        return None

class Receiver:
    """Manages incoming socket and data state."""
    def __init__(
        self,
        protocol: str,
        host: str,
        port: int,
        enable_gripper: bool = False,
        grasp_threshold: float = 0.5,
    ):
        self.protocol = protocol
        self.host = host
        self.port = port
        self.enable_gripper = enable_gripper
        self.grasp_threshold = grasp_threshold
        self.right = ControllerPose()
        self.left = ControllerPose()
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
        logging.info("UDP Bridge Listening on %s:%d", self.host, self.port)
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
        logging.info("TCP Bridge Listening on %s:%d", self.host, self.port)
        
        while not self._stop.is_set():
            try:
                conn, addr = server.accept()
                t = threading.Thread(
                    target=self._handle_tcp_conn, 
                    args=(conn, addr), 
                    daemon=True
                )
                t.start()
            except socket.timeout:
                continue
            except Exception as e:
                logging.error("TCP Server error: %s", e)
        server.close()

def run_bridge(
    in_protocol,
    in_port,
    out_host,
    out_port,
    verbose=False,
    enable_gripper=False,
    grasp_threshold=0.5,
):
    receiver = Receiver(
        in_protocol,
        "0.0.0.0",
        in_port,
        enable_gripper=enable_gripper,
        grasp_threshold=grasp_threshold,
    )
    t = threading.Thread(target=receiver.run, daemon=True)
    t.start()

    out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    out_addr = (out_host, out_port)

    logging.info(
        "Bridge started. Forwarding to UDP %s:%d. Gripper forwarding: %s. Grasp threshold: %.2f",
        out_host,
        out_port,
        "enabled" if enable_gripper else "disabled",
        grasp_threshold,
    )
    
    try:
        while True:
            with receiver._lock:
                # Pack data: 18 floats = 72 bytes
                # Order: R_pos(3), R_quat(4), R_grasp, R_tracked, L_pos(3), L_quat(4), L_grasp, L_tracked
                packet = struct.pack(_PACK_FMT,
                    # Right Hand (9 floats)
                    receiver.right.px, receiver.right.py, receiver.right.pz,
                    receiver.right.qx, receiver.right.qy, receiver.right.qz, receiver.right.qw,
                    receiver.right.grasp,
                    float(receiver.right.tracked),
                    # Left Hand (9 floats)
                    receiver.left.px, receiver.left.py, receiver.left.pz,
                    receiver.left.qx, receiver.left.qy, receiver.left.qz, receiver.left.qw,
                    receiver.left.grasp,
                    float(receiver.left.tracked)
                )
                
                if verbose and (receiver.right.tracked or receiver.left.tracked):
                    logging.info("R:%d G:%.1f | L:%d G:%.1f", 
                                 receiver.right.tracked, receiver.right.grasp,
                                 receiver.left.tracked, receiver.left.grasp)

            out_sock.sendto(packet, out_addr)
            time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        logging.info("Stopping bridge...")

def main():
    parser = argparse.ArgumentParser(description="Quest to ManiSkill Bridge")
    parser.add_argument("--in-protocol", choices=["tcp", "udp"], default="tcp")
    parser.add_argument("--in-port", type=int, default=8000)
    parser.add_argument("--out-host", type=str, default="127.0.0.1", help="Target IP address for UDP output")
    parser.add_argument("--out-port", type=int, default=9876)
    parser.add_argument(
        "--enable-gripper",
        action="store_true",
        help="Forward Quest button/fist state as gripper close commands. Default keeps gripper open for arm teleop testing.",
    )
    parser.add_argument(
        "--grasp-threshold",
        type=float,
        default=0.5,
        help="Controller A/primary-button value at or above this threshold is sent as binary grasp=1.0.",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    run_bridge(
        args.in_protocol,
        args.in_port,
        args.out_host,
        args.out_port,
        args.verbose,
        args.enable_gripper,
        args.grasp_threshold,
    )

if __name__ == "__main__":
    main()
