"""
pickup_sequence.py — Full pipeline: detect → align → approach → arm pickup.

Runs on YOUR PC. Sends all commands to the Pi over the network:
  - Camera: reads MJPEG stream from Pi (http://192.168.183.191:8081/stream)
  - Car:    HTTP POST to Pi (http://192.168.183.191:5000/drive)
  - Arm:    ZMQ PUB to Pi (tcp://192.168.183.191:5559)

Requires on the Pi:
  1. camera_stream.py running
  2. car.py (Flask HTTP server) running on port 5000
  3. arm_zmq.py running

Usage:
    python3 pickup_sequence.py                       # full pipeline
    python3 pickup_sequence.py --color red           # detect red object
    python3 pickup_sequence.py --skip-approach       # don't creep forward
    python3 pickup_sequence.py --test-arm            # only test arm commands

Pin mapping (on Pi — for reference):
    Base     : GPIO 4  (Pin 7)    — 0°  (left)  to 180° (right)
    Shoulder : GPIO 8  (Pin 24)   — 20° (down)   to 140° (up)
    Elbow    : GPIO 7  (Pin 26)   — 100° (back)  to 170° (forward)
    Gripper  : GPIO 9  (Pin 21)   — 70° (closed) to 170° (open)
"""

import argparse
import json
import sys
import time

import zmq

# ── Pi network config ─────────────────────────────────────────────────────────
PI_IP        = "10.213.37.191"
ARM_ZMQ_ADDR = f"tcp://{PI_IP}:5559"

# ── Arm poses ─────────────────────────────────────────────────────────────────
# All 4 joints specified explicitly.
# base=90 keeps arm facing forward throughout (update once base is calibrated).
#
#   shoulder: 140=up,  20=down
#   elbow:    170=fwd, 100=back
#   gripper:  0=open, 70=closed
#
# default  — arm tucked down & retracted (out of camera view during driving)
# Pickup sequence (NO dip down!):
#   1. extend  — elbow reaches forward, shoulder stays HIGH
#   2. open    — gripper opens wide (ready to receive object)
#   3. grab    — gripper closes on the object
#   4. lift    — shoulder goes up, elbow retracts
#   5. secure  — close gripper one more time to lock grip
#
ARM_POSES = {
    "default": {"base": 90, "shoulder": 20,  "elbow": 100, "gripper": 70},
    "extend":  {"base": 90, "shoulder": 120, "elbow": 170, "gripper": 70},
    "open":    {"base": 90, "shoulder": 120, "elbow": 170, "gripper": 0},
    "grab":    {"base": 90, "shoulder": 120, "elbow": 170, "gripper": 70},
    "lift":    {"base": 90, "shoulder": 140, "elbow": 120, "gripper": 70},
    "secure":  {"base": 90, "shoulder": 140, "elbow": 100, "gripper": 70},
}

# ── Timing ────────────────────────────────────────────────────────────────────
POSE_SETTLE_TIME = 1.5    # seconds to wait after sending a pose
GRIP_SETTLE_TIME = 2.0    # extra time for gripper to close fully


# ── Arm controller ────────────────────────────────────────────────────────────

class ArmController:
    """Send arm angle commands to arm_zmq.py on the Pi via ZMQ PUB→SUB."""

    def __init__(self):
        self._ctx  = zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.connect(ARM_ZMQ_ADDR)
        # ZMQ PUB needs ~1s for subscriber to connect before first message
        time.sleep(1.0)
        print(f"[arm] connected to {ARM_ZMQ_ADDR}")

    def send_pose(self, pose_name: str):
        """Send a named pose to the Pi. Blocks until settle time elapses."""
        if pose_name not in ARM_POSES:
            raise ValueError(
                f"Unknown pose '{pose_name}'. "
                f"Available: {list(ARM_POSES.keys())}"
            )

        # Send only the angle dict — arm_zmq.py does smooth interpolation itself
        cmd = dict(ARM_POSES[pose_name])

        print(f"\n  [arm] -> {pose_name.upper()}  {cmd}")
        self._sock.send_string(json.dumps(cmd))

        settle = GRIP_SETTLE_TIME if pose_name in ("grab", "secure") else POSE_SETTLE_TIME
        time.sleep(settle)
        print(f"  [arm] OK {pose_name.upper()} done (waited {settle}s)")

    def send_manual(self, base: int, shoulder: int, elbow: int, gripper: int):
        """Send exact angles directly (for debugging / calibration)."""
        cmd = {
            "base":     base,
            "shoulder": shoulder,
            "elbow":    elbow,
            "gripper":  gripper,
        }
        print(f"  [arm] manual: {cmd}")
        self._sock.send_string(json.dumps(cmd))
        time.sleep(POSE_SETTLE_TIME)

    def close(self):
        self._sock.close()
        self._ctx.term()
        print("[arm] ZMQ connection closed")


# ── Full pickup sequence ──────────────────────────────────────────────────────

class PickupSequence:
    """
    End-to-end pipeline: detect -> align -> approach -> pick up.

    Usage:
        pickup = PickupSequence(color="blue")
        success = pickup.run()
    """

    def __init__(
        self,
        color: str = "blue",
        skip_approach: bool = False,
        target_area: int = 11500,
    ):
        self._color         = color
        self._skip_approach = skip_approach
        self._target_area   = target_area

    def run(self) -> bool:
        """Execute the full pickup sequence. Returns True on success."""
        from color_detect import ColorDetector, STREAM_ADDR
        from align_to_object import (
            search_for_object, align_to_object,
            approach_object, _stop,
        )

        detector = ColorDetector(
            stream_addr=STREAM_ADDR,
            target_color=self._color,
        )
        arm = ArmController()

        try:
            # ── Phase 0: DEFAULT arm position (tucked down, out of camera view)
            arm.send_pose("default")

            # ── Phase 1: SEARCH ───────────────────────────────────────────
            print("\n" + "=" * 60)
            print("  PHASE 1: SEARCH")
            print("=" * 60)
            found = search_for_object(detector, scan_direction="left")
            if not found:
                print("\n[ABORT] could not find the object")
                return False

            # ── Phase 2: ALIGN ────────────────────────────────────────────
            print("\n" + "=" * 60)
            print("  PHASE 2: ALIGN")
            print("=" * 60)
            aligned = align_to_object(detector)
            if not aligned:
                print("\n[ABORT] could not align to object")
                return False

            # ── Phase 3: APPROACH ─────────────────────────────────────────
            if not self._skip_approach:
                print("\n" + "=" * 60)
                print("  PHASE 3: APPROACH")
                print("=" * 60)
                close = approach_object(
                    detector, target_area=self._target_area
                )
                if not close:
                    print("\n[ABORT] could not approach object")
                    return False
            else:
                print("\n  [skip] approach phase skipped")

            # ── Phase 4: PICKUP ───────────────────────────────────────────
            print("\n" + "=" * 60)
            print("  PHASE 4: PICKUP")
            print("=" * 60)
            _stop()
            time.sleep(0.3)   # let car fully stop before arm moves

            arm.send_pose("extend")  # elbow forward, shoulder stays UP
            arm.send_pose("open")    # gripper opens
            arm.send_pose("grab")    # gripper closes on object
            arm.send_pose("lift")    # shoulder up, retract a bit
            arm.send_pose("secure")  # close gripper again to lock

            print("\n" + "=" * 60)
            print("  PICKUP COMPLETE")
            print("=" * 60)
            return True

        except KeyboardInterrupt:
            _stop()
            print("\n[pickup] interrupted")
            return False

        finally:
            _stop()
            detector.release()
            arm.close()


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Full pickup pipeline: detect → align → approach → grab"
    )
    parser.add_argument("--color", default="blue",
                        help="Target colour (default: blue)")
    parser.add_argument("--skip-approach", action="store_true",
                        help="Skip the forward-creep phase")
    parser.add_argument("--target-area", type=int, default=15000,
                        help="Approach until object area >= this (px²)")
    parser.add_argument("--test-arm", action="store_true",
                        help="Only test arm commands (no detection/driving)")
    parser.add_argument("--pose", default=None,
                        help="Send a single named pose and exit "
                             f"({', '.join(ARM_POSES.keys())})")
    args = parser.parse_args()

    # ── Arm-only test mode ────────────────────────────────────────────────
    if args.test_arm:
        print("[test-arm] Sending arm commands to Pi via ZMQ")
        arm = ArmController()
        try:
            if args.pose:
                arm.send_pose(args.pose)
            else:
                input("  Press Enter → STOW (arm up, retracted) ... ")
                arm.send_pose("stow")

                input("  Press Enter → EXTEND (elbow forward, shoulder stays up) ... ")
                arm.send_pose("extend")

                input("  Press Enter → OPEN (gripper opens) ... ")
                arm.send_pose("open")

                input("  Press Enter → GRAB (close gripper) ... ")
                arm.send_pose("grab")

                input("  Press Enter → LIFT (shoulder up) ... ")
                arm.send_pose("lift")

                input("  Press Enter → SECURE (close gripper again) ... ")
                arm.send_pose("secure")

            print("\n[test-arm] ✓ done")
        except KeyboardInterrupt:
            print("\n[test-arm] interrupted")
        finally:
            arm.close()
        return

    # ── Full pipeline ─────────────────────────────────────────────────────
    pickup = PickupSequence(
        color=args.color,
        skip_approach=args.skip_approach,
        target_area=args.target_area,
    )
    success = pickup.run()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()