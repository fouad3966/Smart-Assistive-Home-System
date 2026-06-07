"""
align_to_object.py — Spin the car until the detected object is centred in frame.

Runs on YOUR PC. Reads detection from Pi's camera stream, sends drive
commands to the Pi's car HTTP API over the network.

Usage:
    python3 align_to_object.py                        # default blue
    python3 align_to_object.py --color red
    python3 align_to_object.py --skip-approach         # only spin, don't creep fwd

Requires on the Pi:
    - camera_zmq.py running (port 5555)
    - car HTTP server running (port 5000)
"""

import argparse
import time

import requests

from color_detect import ColorDetector, STREAM_ADDR

# ── Pi network config ─────────────────────────────────────────────────────────
PI_IP   = "10.213.37.191"
PI_PORT = 5000
CAR_URL = f"http://{PI_IP}:{PI_PORT}/drive"

# ── Tuning ────────────────────────────────────────────────────────────────────
DEAD_ZONE        = 40       # pixels from centre — "close enough"
CONFIRM_FRAMES   = 3        # must be centred for N consecutive frames
SEARCH_TIMEOUT   = 20.0
ALIGN_TIMEOUT    = 30.0

# Alignment spin speeds
SPIN_START       = 45       # duty % on first iteration
SPIN_MIN         = 35       # absolute floor — must be high enough to overcome friction!
SPIN_DECAY       = 1        # gentle decay per iteration

# Stuck recovery: if the motor can't turn at SPIN_MIN, try this harder
UNSTICK_SPIN     = 60       # burst speed to break free
UNSTICK_PULSE    = 0.12     # short strong burst

SEARCH_SPIN      = 35


# ── Low-level car commands ────────────────────────────────────────────────────

def _stop():
    try:
        requests.post(CAR_URL,
            json={"w": False, "a": False, "s": False, "d": False,
                  "total": 0, "inner": 0},
            timeout=0.5)
    except Exception:
        pass


def _spin_left(speed: int):
    """Pure left spin (a key only)."""
    try:
        requests.post(CAR_URL,
            json={"w": False, "a": True, "s": False, "d": False,
                  "total": speed, "inner": speed},
            timeout=0.5)
    except Exception:
        pass


def _spin_right(speed: int):
    """Pure right spin (d key only)."""
    try:
        requests.post(CAR_URL,
            json={"w": False, "a": False, "s": False, "d": True,
                  "total": speed, "inner": speed},
            timeout=0.5)
    except Exception:
        pass


def _forward(speed: int = 50):
    try:
        requests.post(CAR_URL,
            json={"w": True, "a": False, "s": False, "d": False,
                  "total": speed, "inner": speed},
            timeout=0.5)
    except Exception:
        pass


def _backward(speed: int = 35):
    try:
        requests.post(CAR_URL,
            json={"w": False, "a": False, "s": True, "d": False,
                  "total": speed, "inner": speed},
            timeout=0.5)
    except Exception:
        pass


def _detect(detector: ColorDetector):
    """Get a single detection. Returns (found, error_px, area, frame_w)."""
    det = detector.detect_once()
    if not det["found"]:
        return False, 0, 0, det.get("frame_w", 640)
    error = det["cx"] - (det["frame_w"] // 2)
    return True, error, det["area"], det["frame_w"]


def _detect_fresh(detector: ColorDetector, flush_count: int = 3):
    """
    Flush stale buffered frames, return only the LATEST detection.
    This prevents acting on old data from before the car stopped.
    """
    result = (False, 0, 0, 640)
    for _ in range(flush_count):
        result = _detect(detector)
    return result


# ── Search: slow scan until object appears ────────────────────────────────────

def search_for_object(
    detector: ColorDetector,
    scan_direction: str = "left",
    timeout: float = SEARCH_TIMEOUT,
) -> bool:
    """Slowly rotate until the object enters the camera frame."""
    print(f"\n[search] scanning {scan_direction} (timeout {timeout}s)")
    start = time.time()

    while (time.time() - start) < timeout:
        found, error, area, _ = _detect(detector)
        if found:
            _stop()
            print(f"[search] ✓ found  err={error:+d}px  area={area}")
            return True

        if scan_direction == "left":
            _spin_left(SEARCH_SPIN)
        else:
            _spin_right(SEARCH_SPIN)
        time.sleep(0.05)

    _stop()
    print("[search] ✗ timed out")
    return False


# ── Align: iterative nudge-and-verify ────────────────────────────────────────

def align_to_object(
    detector: ColorDetector,
    dead_zone: int = DEAD_ZONE,
    confirm_frames: int = CONFIRM_FRAMES,
    timeout: float = ALIGN_TIMEOUT,
) -> bool:
    """
    Iterative nudge-and-verify alignment:
      1. Detect FRESH object position (flush stale frames first!)
      2. If off-centre: nudge in that direction
      3. Stop, settle, flush stale frames, detect again
      4. Spin speed decreases gently each iteration (coarse → fine)
      5. If stuck (motor too weak), burst at higher speed briefly
      6. Once centred for confirm_frames in a row → success
    """
    print(f"\n[align] centering  dead_zone=±{dead_zone}px  confirm={confirm_frames}")

    centred_count  = 0
    stuck_count    = 0
    last_abs_err   = 9999
    iteration      = 0
    start          = time.time()

    while (time.time() - start) < timeout:

        # 1. Get FRESH detection (flush any stale buffered frames)
        found, error, area, fw = _detect_fresh(detector)

        if not found:
            stuck_count += 1
            if stuck_count >= 20:
                print("[align] ✗ object lost")
                return False
            time.sleep(0.04)
            continue

        abs_err = abs(error)

        # 2. Check if centred
        if abs_err <= dead_zone:
            centred_count += 1
            print(f"  centred {centred_count}/{confirm_frames}  err={error:+d}px")
            if centred_count >= confirm_frames:
                _stop()
                print(f"[align] ✓ aligned  area={area}")
                return True
            stuck_count = 0
            time.sleep(0.05)
            continue

        # Lost centring streak
        centred_count = 0
        iteration    += 1

        # 3. Spin speed — decays gently, never below SPIN_MIN
        spin_speed = max(SPIN_MIN, SPIN_START - (iteration - 1) * SPIN_DECAY)

        # 4. Pulse duration — proportional to error size
        #    Small error → tiny pulse (0.05s), large error → bigger pulse (0.18s)
        half_frame     = fw / 2.0
        ratio          = min(abs_err / half_frame, 1.0)
        pulse_duration = 0.05 + ratio * 0.13

        # DIRECTION: object LEFT of centre (error<0) → car must turn LEFT
        #           object RIGHT of centre (error>0) → car must turn RIGHT
        direction = "L" if error < 0 else "R"
        print(f"  iter={iteration:3d}  {direction}  err={error:+d}px  "
              f"spd={spin_speed}  pulse={int(pulse_duration*1000)}ms")

        # 5. Nudge
        if error < 0:
            _spin_left(spin_speed)
        else:
            _spin_right(spin_speed)
        time.sleep(pulse_duration)
        _stop()

        # 6. CRITICAL: Wait for car to physically stop, then flush stale
        #    frames so next _detect_fresh reads the settled position
        time.sleep(0.12)

        # 7. Stuck detection — error hasn't changed
        if abs_err >= last_abs_err - 3:
            stuck_count += 1
            if stuck_count >= 6:
                # Wheels stuck — jog forward then backward to break friction
                print("  [align] stuck — jogging fwd/bwd to break friction")
                _forward(40)
                time.sleep(0.15)
                _stop()
                time.sleep(0.08)
                _backward(35)
                time.sleep(0.10)
                _stop()
                time.sleep(0.10)

                stuck_count = 0
                iteration   = max(0, iteration - 5)  # loosen decay
        else:
            stuck_count = 0

        last_abs_err = abs_err

    _stop()
    print(f"[align] ✗ timed out after {timeout:.0f}s")
    return False


# ── Approach: nudge forward → stop → verify → repeat ─────────────────────────

def approach_object(
    detector: ColorDetector,
    target_area: int = 15000,
    timeout: float = 40.0,
) -> bool:

    """
    Proportional forward approach with ZUPT-style stuck detection:
      - Drive in short bursts, verify area after each
      - Slow down as area grows (proportional speed)
      - ZUPT: if area hasn't grown over several readings, car is stuck
        → boost speed or jog backward then push harder
      - Re-align if object drifts off-centre
    """
    MAX_FWD_SPEED  = 50    # speed when far
    MIN_FWD_SPEED  = 35    # speed near target — high enough to always move!
    MAX_PULSE      = 0.20  # seconds — burst when far
    MIN_PULSE      = 0.08  # seconds — burst when close
    BOOST_SPEED    = 60    # used when stuck (short burst)
    BOOST_PULSE    = 0.15

    # ZUPT parameters
    ZUPT_WINDOW    = 5     # check area growth over last N readings
    ZUPT_MIN_GROWTH = 100  # area must grow by at least this much over window
    MAX_STUCK      = 3     # after this many stuck detections, jog backward

    print(f"\n[approach] target area >= {target_area}")
    start = time.time()
    lost  = 0
    area_history = []       # rolling window of recent area readings
    stuck_count  = 0

    while (time.time() - start) < timeout:
        # Use fresh detection to avoid stale-frame drift
        found, error, area, _ = _detect_fresh(detector)

        if not found:
            lost += 1
            if lost > 15:
                print("[approach] ✗ object lost")
                return False
            time.sleep(0.05)
            continue
        lost = 0

        # Arrived?
        if area >= target_area:
            _stop()
            print(f"[approach] ✓ reached target  area={area}")
            return True

        # Re-align if drifting
        if abs(error) > DEAD_ZONE:
            _stop()
            print(f"  [approach] drifted (err={error:+d}px) — re-aligning")
            align_to_object(detector, timeout=10.0)
            area_history.clear()
            stuck_count = 0
            continue

        # ── ZUPT: stuck detection ──
        area_history.append(area)
        if len(area_history) > ZUPT_WINDOW:
            area_history.pop(0)

        is_stuck = False
        if len(area_history) >= ZUPT_WINDOW:
            growth = area_history[-1] - area_history[0]
            if growth < ZUPT_MIN_GROWTH:
                is_stuck = True

        if is_stuck:
            stuck_count += 1
            if stuck_count >= MAX_STUCK:
                # Hard unstick: jog backward, then strong forward push
                print(f"  [ZUPT] car stuck ({stuck_count}x) — backward jog + hard push")
                _backward(40)
                time.sleep(0.18)
                _stop()
                time.sleep(0.10)
                _forward(BOOST_SPEED)
                time.sleep(0.25)
                _stop()
                time.sleep(0.10)
                stuck_count = 0
                area_history.clear()
                continue
            else:
                # Soft unstick: single boosted forward burst
                print(f"  [ZUPT] car stuck ({stuck_count}x) — boosting to {BOOST_SPEED}%")
                _forward(BOOST_SPEED)
                time.sleep(BOOST_PULSE)
                _stop()
                time.sleep(0.10)
                area_history.clear()
                continue

        # ── Normal proportional approach ──
        stuck_count = 0
        progress = min(area / target_area, 1.0)  # 0=far, 1=at target
        speed    = int(MAX_FWD_SPEED - progress * (MAX_FWD_SPEED - MIN_FWD_SPEED))
        pulse    = MAX_PULSE - progress * (MAX_PULSE - MIN_PULSE)

        print(f"  area={area:6d}/{target_area}  err={error:+d}px  "
              f"spd={speed}  pulse={int(pulse*1000)}ms")

        _forward(speed)
        time.sleep(pulse)
        _stop()
        time.sleep(0.10)  # settle before next detect

    _stop()
    print(f"[approach] ✗ timed out after {timeout:.0f}s")
    return False


# ── CLI entry point ───────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Align car to a detected object, then approach it."
    )
    parser.add_argument("--color", default="blue",
                        help="Target colour (default: blue)")
    parser.add_argument("--skip-approach", action="store_true",
                        help="Only align, don't drive forward")
    parser.add_argument("--target-area", type=int, default=15000,
                        help="Approach until object area >= this (default 15000)")
    args = parser.parse_args()

    detector = ColorDetector(
        stream_addr=STREAM_ADDR,
        target_color=args.color,
    )

    try:
        found = search_for_object(detector, scan_direction="left")
        if not found:
            print("\n[ABORT] object not found")
            return

        aligned = align_to_object(detector)
        if not aligned:
            print("\n[ABORT] could not align")
            return

        if not args.skip_approach:
            close = approach_object(detector, target_area=args.target_area)
            if not close:
                print("\n[ABORT] could not approach")
                return

        print("\n[DONE] aligned and in position ✓")

    except KeyboardInterrupt:
        _stop()
        print("\n[stopped]")
    finally:
        _stop()
        detector.release()


if __name__ == "__main__":
    main()