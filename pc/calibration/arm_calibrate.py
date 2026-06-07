"""
arm_calibrate.py — Interactive shoulder & elbow calibration tool.

Runs on YOUR PC. Sends arm commands to the Pi via ZMQ.
Base and gripper are locked to their default values from pickup_sequence.py.

Usage:
    python arm_calibrate.py
"""

import json
import time
import zmq

# ── Config ────────────────────────────────────────────────────────────────────
PI_IP        = "10.213.37.191"
ARM_ZMQ_ADDR = f"tcp://{PI_IP}:5559"

# Fixed values (not being calibrated)
BASE    = 90
GRIPPER = 70

# Starting defaults (from pickup_sequence.py "default" pose)
DEFAULT_SHOULDER = 20
DEFAULT_ELBOW    = 100

# ── ZMQ setup ─────────────────────────────────────────────────────────────────
def connect_zmq():
    ctx  = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.connect(ARM_ZMQ_ADDR)
    print(f"[zmq] connecting to {ARM_ZMQ_ADDR} ...")
    time.sleep(1.0)   # PUB needs time for subscriber to connect
    print("[zmq] ready\n")
    return ctx, sock

def send_angles(sock, shoulder, elbow):
    cmd = {
        "base":     BASE,
        "shoulder": shoulder,
        "elbow":    elbow,
        "gripper":  GRIPPER,
    }
    sock.send_string(json.dumps(cmd))
    print(f"  → sent: shoulder={shoulder}°  elbow={elbow}°  (base={BASE}, gripper={GRIPPER})")

# ── Helpers ───────────────────────────────────────────────────────────────────
def prompt_int(prompt, current, lo=0, hi=180):
    """Ask for an integer, show current value, validate range."""
    while True:
        raw = input(f"  {prompt} [{current}] (Enter = keep): ").strip()
        if raw == "":
            return current
        try:
            val = int(raw)
            if lo <= val <= hi:
                return val
            print(f"    ⚠  out of range ({lo}–{hi}), try again")
        except ValueError:
            print("    ⚠  enter a number")

# ── Main loop ─────────────────────────────────────────────────────────────────
def main():
    print("=" * 55)
    print("  ARM CALIBRATION — shoulder & elbow")
    print("  Ranges:  shoulder 20°(down) – 140°(up)")
    print("           elbow   100°(back) – 170°(forward)")
    print("  Type 'q' or Ctrl-C to quit and save last values.")
    print("=" * 55 + "\n")

    ctx, sock = connect_zmq()

    shoulder = DEFAULT_SHOULDER
    elbow    = DEFAULT_ELBOW

    # Send starting position immediately
    print(f"[init] sending default pose: shoulder={shoulder}°  elbow={elbow}°")
    send_angles(sock, shoulder, elbow)
    time.sleep(1.5)

    saved = []   # log of every accepted position

    try:
        while True:
            print()
            new_shoulder = prompt_int("shoulder", shoulder, lo=20, hi=140)
            new_elbow    = prompt_int("elbow",    elbow,    lo=100, hi=170)

            if new_shoulder == shoulder and new_elbow == elbow:
                print("  (no change)")
                continue

            shoulder = new_shoulder
            elbow    = new_elbow

            send_angles(sock, shoulder, elbow)
            time.sleep(0.8)   # small settle

            mark = input("  Mark this position? (y/n) [n]: ").strip().lower()
            if mark == "y":
                label = input("  Label (e.g. 'pickup_ready'): ").strip() or f"pos_{len(saved)+1}"
                saved.append({"label": label, "shoulder": shoulder, "elbow": elbow})
                print(f"  ✓ saved as '{label}'")

    except (KeyboardInterrupt, EOFError):
        pass

    finally:
        sock.close()
        ctx.term()
        print("\n[zmq] closed")

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f"\n{'='*55}")
    print(f"  Final values:  shoulder={shoulder}°   elbow={elbow}°")
    print(f"{'='*55}")

    if saved:
        print("\n  Marked positions:")
        for p in saved:
            print(f"    \"{p['label']}\": shoulder={p['shoulder']}°  elbow={p['elbow']}°")
        print()
        print("  Copy into ARM_POSES in pickup_sequence.py:")
        print()
        for p in saved:
            print(f'    "{p["label"]}": {{"base": {BASE}, "shoulder": {p["shoulder"]}, '
                  f'"elbow": {p["elbow"]}, "gripper": {GRIPPER}}},')
    else:
        print("\n  No positions were marked.")
        print(f"  Use these if they felt right:")
        print(f'    "default": {{"base": {BASE}, "shoulder": {shoulder}, '
              f'"elbow": {elbow}, "gripper": {GRIPPER}}},')

if __name__ == "__main__":
    main()
