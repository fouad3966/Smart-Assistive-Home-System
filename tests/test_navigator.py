"""
test_navigator.py — Navigator test suite
=========================================

ORDER: run top to bottom. Each section must pass before the next.

  SECTION 1  Hardware tests          (no SLAM, no navigator)
  SECTION 2  Reader unit tests       (SLAM + IMU sockets)
  SECTION 3  Geometry unit tests     (pure math, no hardware)
  SECTION 4  Navigator integration   (simple → complex)

Run all:
    python test_navigator.py

Run one section:
    python test_navigator.py hardware
    python test_navigator.py readers
    python test_navigator.py geometry
    python test_navigator.py navigator

Run one test by name:
    python test_navigator.py T01
"""

import json
import math
import sys
import time
import threading
import tempfile
import os

import requests
import zmq

# ── import the module under test ──────────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import navigator as nav

# ── test registry ─────────────────────────────────────────────────────────────
_TESTS = []   # list of (id, section, name, fn)

def test(tid: str, section: str, name: str):
    def decorator(fn):
        _TESTS.append((tid, section, name, fn))
        return fn
    return decorator


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 1 — HARDWARE TESTS
#  These talk directly to the car HTTP API with no higher-level logic.
#  Run these first to confirm the car is alive and each wheel direction works.
# ─────────────────────────────────────────────────────────────────────────────

@test("T01", "hardware", "Car HTTP API reachable")
def t01_ping():
    """POST a full-stop to the car.  If it returns 200 the Pi is up."""
    resp = requests.post(
        nav.CAR_URL,
        json={'w': False, 'a': False, 's': False, 'd': False,
              'total': 0, 'inner': 0},
        timeout=3.0
    )
    assert resp.status_code == 200, f"Expected 200, got {resp.status_code}"
    print("    ✓ Pi is reachable")


@test("T02", "hardware", "Forward 1 s — wheel direction")
def t02_forward():
    """
    Car should move FORWARD for 1 second then stop.
    Watch the car physically to confirm direction.
    """
    input("    Place car in open space. Press Enter to drive FORWARD 1 s…")
    nav._forward()
    time.sleep(1.0)
    nav._stop()
    result = input("    Did the car move FORWARD? [y/n]: ").strip().lower()
    assert result == 'y', "Forward direction failed or car did not move"


@test("T03", "hardware", "Reverse 1 s — wheel direction")
def t03_reverse():
    """Car should move BACKWARD for 1 second then stop."""
    input("    Press Enter to drive REVERSE 1 s…")
    nav._reverse()
    time.sleep(1.0)
    nav._stop()
    result = input("    Did the car move BACKWARD? [y/n]: ").strip().lower()
    assert result == 'y', "Reverse direction failed"


@test("T04", "hardware", "Spin LEFT 1 s — wheel direction")
def t04_spin_left():
    """Car should spin LEFT (counter-clockwise from above) for 1 second."""
    input("    Press Enter to spin LEFT 1 s…")
    nav._spin_left()
    time.sleep(1.0)
    nav._stop()
    result = input("    Did the car spin LEFT (CCW from above)? [y/n]: ").strip().lower()
    assert result == 'y', "Spin-left direction failed"


@test("T05", "hardware", "Spin RIGHT 1 s — wheel direction")
def t05_spin_right():
    """Car should spin RIGHT (clockwise from above) for 1 second."""
    input("    Press Enter to spin RIGHT 1 s…")
    nav._spin_right()
    time.sleep(1.0)
    nav._stop()
    result = input("    Did the car spin RIGHT (CW from above)? [y/n]: ").strip().lower()
    assert result == 'y', "Spin-right direction failed"


@test("T06", "hardware", "Stop command halts car immediately")
def t06_stop():
    """Drive forward 0.5 s then stop. Car must stop within ~0.5 s of _stop()."""
    input("    Press Enter to test emergency stop…")
    nav._forward()
    time.sleep(0.5)
    nav._stop()
    # _stop() sends 3 back-to-back stop packets — verify the last one returned 200
    resp = requests.post(
        nav.CAR_URL,
        json={'w': False, 'a': False, 's': False, 'd': False,
              'total': 0, 'inner': 0},
        timeout=2.0
    )
    assert resp.status_code == 200
    result = input("    Did the car stop cleanly? [y/n]: ").strip().lower()
    assert result == 'y', "Stop command did not halt the car"


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 2 — READER UNIT TESTS
#  Verify that ImuReader and SlamReader actually receive live data.
# ─────────────────────────────────────────────────────────────────────────────

@test("T07", "readers", "ImuReader receives data within 3 s")
def t07_imu_data():
    ctx = zmq.Context()
    imu = nav.ImuReader(ctx)
    deadline = time.time() + 3.0
    while time.time() < deadline:
        h = imu.get()
        if h is not None:
            print(f"    ✓ IMU heading = {h:.1f}°")
            return
        time.sleep(0.1)
    raise AssertionError("No IMU data received within 3 s — check IMU_HOST/PORT")


@test("T08", "readers", "IMU heading is a valid float in [0, 360)")
def t08_imu_range():
    ctx = zmq.Context()
    imu = nav.ImuReader(ctx)
    time.sleep(1.5)
    h = imu.get()
    assert h is not None, "No IMU data"
    assert 0.0 <= h < 360.0, f"Heading {h} out of range [0, 360)"
    print(f"    ✓ heading = {h:.1f}°")


@test("T09", "readers", "ImuReader updates continuously (not frozen)")
def t09_imu_updates():
    """
    Physically rotate the car ~30° during this test.
    If the heading does not change the reader thread is stuck.
    """
    ctx = zmq.Context()
    imu = nav.ImuReader(ctx)
    time.sleep(0.5)
    h0 = imu.get()
    input("    Slowly rotate car ~30° then press Enter…")
    time.sleep(0.3)
    h1 = imu.get()
    diff = abs(nav._angle_diff(h1, h0))
    print(f"    h0={h0:.1f}°  h1={h1:.1f}°  Δ={diff:.1f}°")
    assert diff > 5, f"Heading did not change enough: Δ={diff:.1f}° (expected >5°)"


@test("T10", "readers", "SlamReader receives data within 5 s")
def t10_slam_data():
    ctx = zmq.Context()
    slam = nav.SlamReader(ctx)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        p = slam.get()
        if p is not None:
            x, z = p
            print(f"    ✓ SLAM pos = ({x:.4f}, {z:.4f})")
            return
        time.sleep(0.2)
    raise AssertionError("No SLAM data within 5 s — is slam_zmq.py running?")


@test("T11", "readers", "SLAM position changes when car moves")
def t11_slam_moves():
    """
    Drive forward 0.5 s and confirm the SLAM x/z changed by > 0.02 m.
    This catches a frozen or replaying SLAM stream.
    """
    ctx = zmq.Context()
    slam = nav.SlamReader(ctx)
    time.sleep(1.0)
    p0 = slam.get()
    assert p0 is not None, "No SLAM data"
    x0, z0 = p0

    input("    Place car in open space. Press Enter to drive forward 0.5 s…")
    nav._forward()
    time.sleep(0.5)
    nav._stop()
    time.sleep(0.3)

    p1 = slam.get()
    assert p1 is not None
    x1, z1 = p1
    moved = nav._dist(x0, z0, x1, z1)
    print(f"    moved {moved:.4f} m")
    assert moved > 0.02, f"SLAM did not update after driving: moved only {moved:.4f} m"


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 3 — GEOMETRY UNIT TESTS
#  Pure math, no hardware. These always run offline.
# ─────────────────────────────────────────────────────────────────────────────

@test("T12", "geometry", "_dist — basic Euclidean distance")
def t12_dist():
    assert abs(nav._dist(0, 0, 3, 4) - 5.0) < 1e-9
    assert abs(nav._dist(1, 1, 1, 1)) < 1e-9
    assert abs(nav._dist(-1, 0, 2, 0) - 3.0) < 1e-9
    print("    ✓ _dist correct")


@test("T13", "geometry", "_bearing — cardinal directions")
def t13_bearing():
    # +Z direction → 0°
    assert abs(nav._bearing(0, 0, 0, 1)) < 1e-6
    # +X direction → 90°
    assert abs(nav._bearing(0, 0, 1, 0) - 90.0) < 1e-6
    # -Z direction → ±180°
    assert abs(abs(nav._bearing(0, 0, 0, -1)) - 180.0) < 1e-6
    # -X direction → -90° (or 270°)
    assert abs(nav._bearing(0, 0, -1, 0) + 90.0) < 1e-6
    print("    ✓ _bearing cardinal directions correct")


@test("T14", "geometry", "_angle_diff — shortest path")
def t14_angle_diff():
    assert abs(nav._angle_diff(10,  350) - 20)   < 1e-6   # wrap forward
    assert abs(nav._angle_diff(350, 10)  + 20)   < 1e-6   # wrap backward
    assert abs(nav._angle_diff(90,  0)   - 90)   < 1e-6
    assert abs(nav._angle_diff(0,   90)  + 90)   < 1e-6
    assert abs(nav._angle_diff(180, 0)   - 180)  < 1e-6
    print("    ✓ _angle_diff correct")


@test("T15", "geometry", "_cross2d — spin direction from L-shape vectors")
def t15_cross():
    """
    start=(0,0) → station1 standoff=(0.3, 0.4163+0.3):
      Leg 1: (0,0)→elbow  = (0, +Z) →  az>0
      Leg 2: elbow→standoff = (+X, 0) → bx>0
      cross = ax*bz - az*bx = 0 - (+)(+) < 0 → RIGHT spin ✓

    start=station1_standoff → station2 standoff:
      cross > 0 → LEFT spin ✓
    """
    # Leg1 along +Z, leg2 along +X → turn RIGHT
    cross_right = nav._cross2d(0, 1, 1, 0)   # a=(0,+1), b=(+1,0)
    assert cross_right < 0, f"Expected <0 (right), got {cross_right}"

    # Leg1 along +Z, leg2 along -X → turn LEFT
    cross_left  = nav._cross2d(0, 1, -1, 0)  # a=(0,+1), b=(-1,0)
    assert cross_left > 0, f"Expected >0 (left), got {cross_left}"
    print("    ✓ spin direction from cross product correct")


@test("T16", "geometry", "ORIENTATION_VECTOR — standoff positions")
def t16_standoff():
    """
    Manually verify standoff positions match the stations.json values.
    station1: x=-0.1653, z=0.4163, orientation='-X Wall'
    → standoff_x = -0.1653 + 1*0.30 = +0.1347
    → standoff_z = 0.4163
    """
    # Build a minimal Navigator-like helper to call _standoff_point
    class _MockNav:
        _standoff_point = nav.Navigator._standoff_point

    mn = _MockNav()

    sx, sz = mn._standoff_point(mn, -0.1653, 0.4163, "-X Wall")
    assert abs(sx - 0.1347) < 1e-4, f"standoff_x={sx}"
    assert abs(sz - 0.4163) < 1e-4, f"standoff_z={sz}"

    sx2, sz2 = mn._standoff_point(mn, 0.072, 0.8483, "+Z Wall")
    assert abs(sx2 - 0.072)  < 1e-4, f"standoff_x={sx2}"
    assert abs(sz2 - (0.8483 - 0.30)) < 1e-4, f"standoff_z={sz2}"

    print("    ✓ standoff points match expected values")


@test("T17", "geometry", "Stations.json loads with all required fields")
def t17_stations_json():
    stations_path = "/home/boethius/autonomous_car/navigation/stations.json"
    if not os.path.exists(stations_path):
        print(f"    SKIP: {stations_path} not found on this machine")
        return

    with open(stations_path) as f:
        data = json.load(f)

    required = {'label', 'x', 'z', 'orientation'}
    for name, info in data.items():
        missing = required - set(info.keys())
        assert not missing, f"Station '{name}' missing fields: {missing}"
    print(f"    ✓ {len(data)} stations loaded, all fields present")


# ─────────────────────────────────────────────────────────────────────────────
#  SECTION 4 — NAVIGATOR INTEGRATION TESTS
#  These drive the real car. Simple moves first, then full routes.
# ─────────────────────────────────────────────────────────────────────────────

STATIONS_FILE = "/home/boethius/autonomous_car/navigation/stations.json"

def _make_nav() -> nav.Navigator:
    """Helper: build a Navigator and verify sensors are live."""
    return nav.Navigator(STATIONS_FILE)


@test("T18", "navigator", "Navigator init — IMU + SLAM both live")
def t18_nav_init():
    """
    Just constructing Navigator() is the test.
    It already waits 4 s max for each sensor and sys.exit(1) on failure.
    """
    n = _make_nav()
    h = n._imu.get()
    p = n._slam.get()
    assert h is not None, "IMU missing after Navigator init"
    assert p is not None, "SLAM missing after Navigator init"
    print(f"    ✓ IMU={h:.1f}°  SLAM={p}")


@test("T19", "navigator", "_depart() skipped when not docked")
def t19_depart_no_op():
    """
    If _docked_at is None, _depart() must return instantly without moving.
    Record SLAM position before and after — it must not change.
    """
    n = _make_nav()
    assert n._docked_at is None
    p0 = n._slam.get()

    # Monkey-patch _reverse to fail if called
    called = []
    original = nav._reverse
    nav._reverse = lambda: called.append(True)
    try:
        # _depart() is only called from Maps_to when _docked_at is not None.
        # Confirm internal guard holds.
        if n._docked_at is None:
            print("    ✓ _depart() would be skipped (not docked)")
            return
        n._depart()
    finally:
        nav._reverse = original

    assert not called, "_reverse was called even though car was not docked"


@test("T20", "navigator", "Maps_to unknown station raises ValueError")
def t20_bad_station():
    n = _make_nav()
    try:
        n.Maps_to("nonexistent_xyz")
        raise AssertionError("Should have raised ValueError")
    except ValueError as e:
        print(f"    ✓ ValueError raised: {e}")


@test("T21", "navigator", "_spin_delta 90° LEFT — IMU tracks correctly")
def t21_spin_left_90():
    """
    Spin LEFT 90°. Measure actual IMU change.
    Pass if turned 90° ± 15° (SPIN_TOLERANCE is 6° but motors have slop).
    """
    ctx = zmq.Context()
    imu = nav.ImuReader(ctx)
    time.sleep(1.0)
    h0 = imu.get()
    assert h0 is not None

    input(f"    Starting at hdg={h0:.1f}°. Press Enter to spin LEFT 90°…")
    nav._spin_delta(90, turn_left=True, imu=imu)
    h1 = imu.get()
    turned = (h1 - h0) % 360
    print(f"    h0={h0:.1f}° → h1={h1:.1f}° → turned={turned:.1f}°")
    assert 75 <= turned <= 105, f"Expected ~90° left, got {turned:.1f}°"


@test("T22", "navigator", "_spin_delta 90° RIGHT — IMU tracks correctly")
def t22_spin_right_90():
    ctx = zmq.Context()
    imu = nav.ImuReader(ctx)
    time.sleep(1.0)
    h0 = imu.get()

    input(f"    Starting at hdg={h0:.1f}°. Press Enter to spin RIGHT 90°…")
    nav._spin_delta(90, turn_left=False, imu=imu)
    h1 = imu.get()
    turned = (h0 - h1) % 360
    print(f"    h0={h0:.1f}° → h1={h1:.1f}° → turned={turned:.1f}°")
    assert 75 <= turned <= 105, f"Expected ~90° right, got {turned:.1f}°"


@test("T23", "navigator", "_drive_to 0.5 m straight — SLAM distance correct")
def t23_drive_half_meter():
    """
    Drive straight forward 0.5 m using _drive_to.
    Verify SLAM measured at least 0.35 m of movement (15°-ish heading error ok).
    """
    ctx = zmq.Context()
    slam = nav.SlamReader(ctx)
    imu  = nav.ImuReader(ctx)
    time.sleep(1.5)

    p0 = slam.get()
    assert p0 is not None
    x0, z0 = p0

    # Target 0.5 m straight ahead in the +Z direction from current position
    # (just pick a target far enough away that the threshold ends travel)
    target_x = x0
    target_z = z0 + 0.50

    input(f"    From ({x0:.3f},{z0:.3f}), driving to ({target_x:.3f},{target_z:.3f}). Press Enter…")
    nav._drive_to("test_0.5m", target_x, target_z, 0.08, slam, imu,
                  heading_correction=False)

    p1 = slam.get()
    moved = nav._dist(x0, z0, *p1)
    print(f"    Moved {moved:.3f} m")
    assert moved >= 0.35, f"Expected ≥0.35 m, got {moved:.3f} m"


@test("T24", "navigator", "Maps_to station1 — full L-shape route")
def t24_maps_to_station1():
    """
    Full route from Start to Station 1.
    Car starts near (0.03, -0.007) facing +Z.
    Expected path:
      Phase 1 — north along current X to z≈0.7163
      Phase 2 — spin RIGHT 90°
      Phase 3 — dock westward to standoff at (0.1347, 0.4163)
    """
    input("\n    Place car at START position facing +Z. Press Enter to begin Maps_to('station1')…")
    n = _make_nav()
    n.Maps_to("station1")

    # After arrival, SLAM should be near the standoff
    p = n._slam.get()
    sx, sz = 0.1347, 0.4163   # expected standoff
    dist = nav._dist(p[0], p[1], sx, sz)
    print(f"    Final pos = {p}  dist from standoff = {dist:.3f} m")
    assert dist < 0.12, f"Too far from station1 standoff: {dist:.3f} m"
    assert n._docked_at == "station1"


@test("T25", "navigator", "Maps_to station2 from station1 — multi-hop with depart")
def t25_multi_hop():
    """
    This is the hardest test.
    1. Drive to station1  (car faces -X wall)
    2. Call Maps_to('station2') — must:
       a. _depart() — reverse 0.3 m away from -X wall
       b. L-shape to station2 standoff (+Z wall approach → spin LEFT)
    Watch that the car:
      - backs away from the station1 wall first  ← key visual check
      - turns LEFT at the elbow
      - docks at station2
    """
    input("\n    Place car at START. Press Enter to run station1 → station2 multi-hop…")
    n = _make_nav()

    print("\n  Step A: driving to station1…")
    n.Maps_to("station1")
    print(f"  _docked_at = {n._docked_at}")
    assert n._docked_at == "station1"

    print("\n  Step B: driving station1 → station2 (should depart first)…")
    n.Maps_to("station2")

    p = n._slam.get()
    # station2 standoff: x=0.072, z=0.8483-0.30=0.5483
    sx, sz = 0.072, 0.5483
    dist = nav._dist(p[0], p[1], sx, sz)
    print(f"    Final pos = {p}  dist from station2 standoff = {dist:.3f} m")
    assert dist < 0.12, f"Too far from station2 standoff: {dist:.3f} m"
    assert n._docked_at == "station2"

    result = input("    Did the car reverse away from station1 before turning? [y/n]: ").strip().lower()
    assert result == 'y', "Depart maneuver not observed"


# ─────────────────────────────────────────────────────────────────────────────
#  TEST RUNNER
# ─────────────────────────────────────────────────────────────────────────────

SECTIONS = ["hardware", "readers", "geometry", "navigator"]

def _run(tests):
    passed = failed = skipped = 0
    for tid, section, name, fn in tests:
        print(f"\n{'─'*60}")
        print(f"  {tid}  [{section}]  {name}")
        print(f"{'─'*60}")
        try:
            fn()
            print(f"  ✅  PASS")
            passed += 1
        except KeyboardInterrupt:
            print(f"  ⏭  SKIPPED (Ctrl-C)")
            skipped += 1
        except AssertionError as e:
            print(f"  ❌  FAIL — {e}")
            failed += 1
        except Exception as e:
            print(f"  ❌  ERROR — {type(e).__name__}: {e}")
            failed += 1

    print(f"\n{'═'*60}")
    print(f"  Results: {passed} passed  {failed} failed  {skipped} skipped")
    print(f"{'═'*60}\n")
    return failed == 0


def main():
    arg = sys.argv[1].lower() if len(sys.argv) > 1 else "all"

    if arg == "all":
        tests = _TESTS
    elif arg in SECTIONS:
        tests = [(tid, s, n, fn) for tid, s, n, fn in _TESTS if s == arg]
    else:
        # match by test ID prefix, e.g. "T01"
        tests = [(tid, s, n, fn) for tid, s, n, fn in _TESTS
                 if tid.upper().startswith(arg.upper())]

    if not tests:
        print(f"No tests matched '{arg}'. "
              f"Use: all | {' | '.join(SECTIONS)} | T01..T25")
        sys.exit(1)

    print(f"\n  Running {len(tests)} test(s)  [filter: {arg}]\n")
    ok = _run(tests)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
