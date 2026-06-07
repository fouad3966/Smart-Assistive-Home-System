"""
arm_test.py — Interactive arm calibration tool.

Lets you type servo angles and see the arm move in real-time
while the camera feed is running, so you can find the perfect
"default" position that doesn't block the camera.

Usage:
    python3 arm_test.py

Commands:
    s 80        → set shoulder to 80°
    e 120       → set elbow to 120°
    g 170       → set gripper to 170° (open)
    b 90        → set base to 90°
    all 90 80 100 70  → set base, shoulder, elbow, gripper at once
    show        → print current angles
    save        → print current angles as a Python dict (copy into pickup_sequence.py)
    q / quit    → exit

Requires on the Pi:
    - arm_servo_listener.py running (port 5559)
    - camera stream running (so you can see if the arm blocks the camera)
"""

import json
import time
import zmq

PI_IP        = "10.213.37.191"
ARM_ZMQ_ADDR = f"tcp://{PI_IP}:5559"


def main():
    ctx  = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.connect(ARM_ZMQ_ADDR)
    time.sleep(1.0)
    print(f"[arm_test] connected to {ARM_ZMQ_ADDR}")

    # Current angles (start with everything centred)
    angles = {"base": 90, "shoulder": 90, "elbow": 100, "gripper": 70}

    # Send initial position
    sock.send_string(json.dumps(angles))
    print(f"[arm_test] initial: {angles}")
    print()
    print("Commands:  s <angle>  |  e <angle>  |  g <angle>  |  b <angle>")
    print("           all <base> <shoulder> <elbow> <gripper>")
    print("           show  |  save  |  q")
    print()

    shortcuts = {
        "s": "shoulder",
        "e": "elbow",
        "g": "gripper",
        "b": "base",
    }

    try:
        while True:
            raw = input("arm> ").strip().lower()
            if not raw:
                continue

            if raw in ("q", "quit", "exit"):
                break

            if raw == "show":
                print(f"  {angles}")
                continue

            if raw == "save":
                print(f'  "default": {angles},')
                continue

            parts = raw.split()

            # "all base shoulder elbow gripper"
            if parts[0] == "all" and len(parts) == 5:
                try:
                    angles["base"]     = int(parts[1])
                    angles["shoulder"] = int(parts[2])
                    angles["elbow"]    = int(parts[3])
                    angles["gripper"]  = int(parts[4])
                    sock.send_string(json.dumps(angles))
                    print(f"  → {angles}")
                except ValueError:
                    print("  usage: all <base> <shoulder> <elbow> <gripper>")
                continue

            # Single joint: "s 80", "e 120", etc.
            if len(parts) == 2 and parts[0] in shortcuts:
                joint = shortcuts[parts[0]]
                try:
                    angle = int(parts[1])
                    angles[joint] = angle
                    sock.send_string(json.dumps(angles))
                    print(f"  {joint} → {angle}°   ({angles})")
                except ValueError:
                    print(f"  usage: {parts[0]} <angle>")
                continue

            print("  unknown command. Try: s 80, e 120, g 170, b 90, all 90 80 100 70, show, save, q")

    except (KeyboardInterrupt, EOFError):
        print("\n[arm_test] done")
    finally:
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
