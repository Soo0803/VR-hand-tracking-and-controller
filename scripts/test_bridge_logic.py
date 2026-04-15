import sys
import os

# Mock objects to simulate the bridge environment
class ControllerPose:
    def __init__(self):
        self.px, self.py, self.pz = 0, 0, 0
        self.qx, self.qy, self.qz, self.qw = 0, 0, 0, 1
        self.grasp = 0
        self.tracked = False

class Receiver:
    def __init__(self):
        self.right = ControllerPose()
        self.left = ControllerPose()
        self.fist_state = 0

def _convert_position(x, y, z): return (z, -x, y)
def _convert_quaternion(x, y, z, w): return (z, -x, y, w)

def test_parse():
    # Example line matching new format:
    # Header, isTracked, px, py, pz, qx, qy, qz, qw, fwdX, fwdY, fwdZ, upX, upY, upZ, rtX, rtY, rtZ, button
    # Indices: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18 (in parts)
    # raw_vals starts from parts[1:]
    test_line = "Right controller:, 1, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1"
    
    parts = [p.strip() for p in test_line.split(",")]
    raw_vals = [float(p) for p in parts[1:]]
    
    print(f"Header: {parts[0]}")
    print(f"Raw vals length: {len(raw_vals)}")
    print(f"isTracked (raw_vals[0]): {raw_vals[0]}")
    print(f"Button/Grasp (raw_vals[17]): {raw_vals[17]}")
    
    # Matching the logic in bridge.py
    px, py, pz = _convert_position(raw_vals[1], raw_vals[2], raw_vals[3])
    qx, qy, qz, qw = _convert_quaternion(raw_vals[4], raw_vals[5], raw_vals[6], raw_vals[7])
    grasp = raw_vals[17]
    tracked = raw_vals[0] > 0.5
    
    # Expected: Convert (0.1, 0.2, 0.3) -> (0.3, -0.1, 0.2)
    print(f"Converted Pos: {px}, {py}, {pz}")
    print(f"Grasp detected: {grasp}")
    print(f"Tracked detected: {tracked}")
    
    assert grasp == 1.0, "Grasp signal not correctly parsed!"
    assert tracked == True, "Tracked status not correctly parsed!"
    assert px == 0.3, "Position X conversion incorrect!"
    print("Test Passed!")

if __name__ == "__main__":
    test_parse()
