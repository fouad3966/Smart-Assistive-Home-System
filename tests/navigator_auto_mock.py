"""
navigator.py — TEST / MOCK version (Fixed Generic L-Path)
==========================================================

Fixes over the previous version:
  1. Phase 0: spin to face the elbow (or target if straight) BEFORE driving.
  2. Straight-line detection uses lateral offset from the approach axis.
  3. Departure reverses straight back along the approach axis.
  4. FIX: _drive_to reverse=True now snaps heading so _reverse() actually
     moves toward the target (heading = bearing+180, so backing up = correct dir).
"""

import json
import math
import sys
import time
import threading

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly

# ── TUNING ────────────────────────────────────────────────────────────────────
FORWARD_SPEED      = 45
REVERSE_SPEED      = 45
SPIN_SPEED         = 40
WAYPOINT_THRESHOLD = 0.10
STATION_THRESHOLD  = 0.04
ANGLE_THRESHOLD    = 15
SPIN_TOLERANCE     = 6
STALL_TIMEOUT      = 3.0
DEPART_DIST        = 0.30
ELBOW_CLEARANCE    = 0.10
STRAIGHT_THRESHOLD = 0.03   # only truly aligned (< 3cm off axis) goes straight

# ── MAP DISPLAY ───────────────────────────────────────────────────────────────
MAPPOINTS    = "/home/boethius/autonomous_car/data/MapPoints.txt"
X_MIN, X_MAX = -0.60,  0.65
Z_MIN, Z_MAX = -0.50,  1.55
Y_MIN, Y_MAX = -0.14,  0.10

_STATION_COLORS = ['#ff3366', '#ff9900', '#aa44ff', '#00ccff', '#ffff00']


def _load_hull():
    try:
        from scipy.spatial import ConvexHull
        pts = np.loadtxt(MAPPOINTS)
        px, py, pz = pts[:, 0], pts[:, 1], pts[:, 2]
        mask = ((px >= X_MIN) & (px <= X_MAX) &
                (pz >= Z_MIN) & (pz <= Z_MAX) &
                (py >= Y_MIN) & (py <= Y_MAX))
        pts2d = np.column_stack([px[mask], pz[mask]])
        hull  = ConvexHull(pts2d)
        hull_pts = pts2d[hull.vertices]
        return np.vstack([hull_pts, hull_pts[0]])
    except Exception as e:
        print(f"  [map] could not load hull: {e}")
        return None


def show_map(start, elbow, stations_info, is_straight=False):
    hull_closed = _load_hull()

    fig, ax = plt.subplots(figsize=(8, 12))
    fig.patch.set_facecolor('#0a0a0f')
    ax.set_facecolor('#0d1117')

    if hull_closed is not None:
        room = MplPoly(hull_closed, closed=True,
                       facecolor='#0d1f2d', edgecolor='#2a6090',
                       linewidth=2, zorder=1)
        ax.add_patch(room)

    target = next((s for s in stations_info if s['is_target']), None)

    for i, s in enumerate(stations_info):
        if s['is_target']:
            continue
        color = _STATION_COLORS[i % len(_STATION_COLORS)]
        ax.scatter(s['x'], s['z'], c=color, marker='o',
                   s=120, zorder=6, edgecolors='white', linewidths=1.0)
        ax.annotate(s['name'].upper(), (s['x'], s['z']),
                    textcoords='offset points', xytext=(8, 4),
                    color=color, fontsize=9, fontweight='bold')

    if target is not None:
        sx, sz = start
        ex, ez = elbow
        tx, tz = target['x'], target['z']

        if is_straight:
            leg_len = math.sqrt((tx - sx)**2 + (tz - sz)**2)
            ax.plot([sx, tx], [sz, tz], '-', color='#00ff88', linewidth=3.5,
                    alpha=0.95, zorder=4, label=f'straight  {leg_len:.2f}')
            if leg_len > 0.02:
                mx, mz = (sx + tx) / 2, (sz + tz) / 2
                d1x = (tx - sx) / leg_len * 0.04
                d1z = (tz - sz) / leg_len * 0.04
                ax.annotate('', xy=(mx + d1x, mz + d1z),
                            xytext=(mx - d1x, mz - d1z),
                            arrowprops=dict(arrowstyle='->', color='#00ff88',
                                            lw=2.0), zorder=6)
        else:
            leg1_len = math.sqrt((ex - sx)**2 + (ez - sz)**2)
            leg2_len = math.sqrt((tx - ex)**2 + (tz - ez)**2)

            ax.plot([sx, ex], [sz, ez], '-', color='#00ff88', linewidth=3.5,
                    alpha=0.95, zorder=4, label=f'leg 1  {leg1_len:.2f}')
            if leg1_len > 0.02:
                mx, mz = (sx + ex) / 2, (sz + ez) / 2
                d1x = (ex - sx) / leg1_len * 0.04
                d1z = (ez - sz) / leg1_len * 0.04
                ax.annotate('', xy=(mx + d1x, mz + d1z),
                            xytext=(mx - d1x, mz - d1z),
                            arrowprops=dict(arrowstyle='->', color='#00ff88',
                                            lw=2.0), zorder=6)

            ax.plot([ex, tx], [ez, tz], '-', color='#ffcc00', linewidth=3.5,
                    alpha=0.95, zorder=4, label=f'leg 2  {leg2_len:.2f}')
            if leg2_len > 0.02:
                mx2, mz2 = (ex + tx) / 2, (ez + tz) / 2
                d2x = (tx - ex) / leg2_len * 0.04
                d2z = (tz - ez) / leg2_len * 0.04
                ax.annotate('', xy=(mx2 + d2x, mz2 + d2z),
                            xytext=(mx2 - d2x, mz2 - d2z),
                            arrowprops=dict(arrowstyle='->', color='#ffcc00',
                                            lw=2.0), zorder=6)

            ax.scatter([ex], [ez], c='#00ff88', s=120, marker='D', zorder=7)
            ax.annotate('SPIN HERE', (ex, ez),
                        textcoords='offset points', xytext=(10, 6),
                        color='#00ff88', fontsize=8)

            if leg1_len > 0 and leg2_len > 0:
                cs = 0.025
                l1x = (ex - sx) / leg1_len
                l1z = (ez - sz) / leg1_len
                l2x = (tx - ex) / leg2_len
                l2z = (tz - ez) / leg2_len
                p1x, p1z = ex - l1x * cs, ez - l1z * cs
                p2x, p2z = p1x + l2x * cs, p1z + l2z * cs
                p3x, p3z = ex + l2x * cs, ez + l2z * cs
                ax.plot([p1x, p2x, p3x], [p1z, p2z, p3z],
                        '-', color='#00ff88', linewidth=1.2, alpha=0.7, zorder=5)

        ti = next(i for i, s in enumerate(stations_info) if s['is_target'])
        tcolor = _STATION_COLORS[ti % len(_STATION_COLORS)]
        ax.scatter(target['x'], target['z'], c=tcolor, marker='*',
                   s=320, zorder=8, edgecolors='white', linewidths=1.5)
        ax.annotate(target['name'].upper(), (target['x'], target['z']),
                    textcoords='offset points', xytext=(10, 6),
                    color=tcolor, fontsize=10, fontweight='bold')

    ax.scatter(start[0], start[1], c='#ffffff', marker='s',
               s=160, zorder=9, label='Start')
    ax.annotate('START', (start[0], start[1]),
                textcoords='offset points', xytext=(8, 5),
                color='#ffffff', fontsize=9, fontweight='bold')

    ax.set_xlim(X_MIN, X_MAX)
    ax.set_ylim(Z_MIN, Z_MAX)
    ax.set_xlabel('X  ->', color='#667788', fontsize=10)
    ax.set_ylabel('Z  ->', color='#667788', fontsize=10)

    target_name = target['name'].upper() if target else '?'
    mode_str = 'STRAIGHT' if is_straight else 'green = leg 1   yellow = leg 2'
    ax.set_title(
        f"Route -> {target_name}  |  {mode_str}\nClose window to continue",
        color='#e0e0e0', fontsize=10, fontweight='bold', pad=10)

    ax.tick_params(colors='#334455')
    ax.grid(True, color='#111820', linewidth=0.5)
    for spine in ax.spines.values():
        spine.set_edgecolor('#1e2e3e')
    ax.set_aspect('equal')
    ax.legend(facecolor='#0a0a0f', edgecolor='#1e2e3e',
              labelcolor='#aabbcc', fontsize=9, loc='lower right')

    plt.tight_layout()
    print("\n  [map] window open -- close it to continue")
    plt.show()


# ─────────────────────────────────────────────────────────────────────────────
#  MOCK IMU / SLAM
# ─────────────────────────────────────────────────────────────────────────────

_mock_heading = 270.0           # facing -X wall (docked at station1)
_mock_pose    = (-0.1653, 0.4163)  # station1 docked position
_mock_lock    = threading.Lock()


class ImuReader:
    def get(self) -> float | None:
        with _mock_lock:
            return _mock_heading


class SlamReader:
    def get(self) -> tuple[float, float] | None:
        with _mock_lock:
            return _mock_pose


# ─────────────────────────────────────────────────────────────────────────────
#  MOCK car commands
# ─────────────────────────────────────────────────────────────────────────────

_MOCK_MOVE_STEP = 0.02
_MOCK_SPIN_STEP = 3.0


def _stop():
    print("  [mock] stop")


def _forward():
    global _mock_pose
    with _mock_lock:
        heading_rad = math.radians(_mock_heading)
        dx = math.sin(heading_rad) * _MOCK_MOVE_STEP
        dz = math.cos(heading_rad) * _MOCK_MOVE_STEP
        x, z = _mock_pose
        _mock_pose = (x + dx, z + dz)


def _reverse():
    global _mock_pose
    with _mock_lock:
        heading_rad = math.radians(_mock_heading)
        dx = math.sin(heading_rad) * _MOCK_MOVE_STEP
        dz = math.cos(heading_rad) * _MOCK_MOVE_STEP
        x, z = _mock_pose
        _mock_pose = (x - dx, z - dz)


def _spin_left():
    global _mock_heading
    with _mock_lock:
        _mock_heading = (_mock_heading + _MOCK_SPIN_STEP) % 360


def _spin_right():
    global _mock_heading
    with _mock_lock:
        _mock_heading = (_mock_heading - _MOCK_SPIN_STEP) % 360


# ─────────────────────────────────────────────────────────────────────────────
#  Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

def _dist(x1, z1, x2, z2):
    return math.sqrt((x2 - x1)**2 + (z2 - z1)**2)


def _bearing(cx, cz, tx, tz):
    """0° = +Z, 90° = +X"""
    return math.degrees(math.atan2(tx - cx, tz - cz))


def _angle_diff(target, current):
    return (target - current + 180) % 360 - 180


def _cross2d(ax, az, bx, bz):
    return ax * bz - az * bx


def _segment_point_dist(ax, az, bx, bz, px, pz):
    abx, abz = bx - ax, bz - az
    apx, apz = px - ax, pz - az
    ab_sq = abx*abx + abz*abz
    if ab_sq < 1e-12:
        return _dist(ax, az, px, pz)
    t = max(0.0, min(1.0, (apx*abx + apz*abz) / ab_sq))
    return _dist(ax + t*abx, az + t*abz, px, pz)


def _lateral_offset(car_x, car_z, target_x, target_z, approach_vx, approach_vz):
    dx = target_x - car_x
    dz = target_z - car_z
    perp_x = -approach_vz
    perp_z =  approach_vx
    return abs(dx * perp_x + dz * perp_z)


# ─────────────────────────────────────────────────────────────────────────────
#  Path planning
# ─────────────────────────────────────────────────────────────────────────────

def _parse_approach(orientation):
    if not orientation or len(orientation) < 2:
        return (0.0, 0.0)
    sign = +1.0 if orientation[0] == '+' else -1.0
    axis = orientation[1].upper()
    return (sign, 0.0) if axis == 'X' else (0.0, sign)


def _pick_elbow(sx, sz, tx, tz, approach, obstacles, clearance):
    along_x  = abs(approach[0]) > abs(approach[1])
    elbow_a  = (sx, tz)
    elbow_b  = (tx, sz)
    preferred, fallback = (elbow_a, elbow_b) if along_x else (elbow_b, elbow_a)

    def _collides(elbow):
        ex, ez = elbow
        for ox, oz in obstacles:
            if (_segment_point_dist(sx, sz, ex, ez, ox, oz) < clearance or
                    _segment_point_dist(ex, ez, tx, tz, ox, oz) < clearance):
                return True
        return False

    if not _collides(preferred):
        return preferred
    if not _collides(fallback):
        return fallback

    ex, ez = preferred
    push_x, push_z = clearance, 0.0
    min_d = float('inf')
    for ox, oz in obstacles:
        d = _dist(ex, ez, ox, oz)
        if d < min_d:
            min_d = d
            if d > 1e-9:
                push_x = (ex - ox) / d * clearance
                push_z = (ez - oz) / d * clearance
    return (ex + push_x, ez + push_z)


# ─────────────────────────────────────────────────────────────────────────────
#  Motion primitives
# ─────────────────────────────────────────────────────────────────────────────

def _spin_to_bearing(target_bear, imu):
    global _mock_heading
    print(f"  [spin] -> bearing {target_bear:.1f} deg")
    while True:
        hdg = imu.get()
        if hdg is None:
            time.sleep(0.02)
            continue
        df = _angle_diff(target_bear, hdg)
        print(f"  hdg={hdg:.1f}  diff={df:.1f}")
        if abs(df) <= SPIN_TOLERANCE:
            _stop()
            with _mock_lock:
                _mock_heading = target_bear % 360
            print(f"  [spin] done -- hdg={imu.get():.1f}")
            return
        _spin_left() if df > 0 else _spin_right()
        time.sleep(0.02)


def _spin_delta(degrees, turn_left, imu):
    start = imu.get()
    direction = 'left' if turn_left else 'right'
    print(f"  [spin] {direction} {degrees:.1f} deg  from hdg={start:.1f}")
    while True:
        hdg = imu.get()
        if hdg is None:
            time.sleep(0.02)
            continue
        turned = (hdg - start) % 360 if turn_left else (start - hdg) % 360
        print(f"  hdg={hdg:.1f}  turned={turned:.1f}")
        if turned >= degrees - SPIN_TOLERANCE:
            _stop()
            print(f"  [spin] done -- turned {turned:.1f} deg")
            return
        _spin_left() if turn_left else _spin_right()
        time.sleep(0.02)


def _correct_heading(target_bear, imu):
    global _mock_heading
    while True:
        hdg = imu.get()
        if hdg is None:
            time.sleep(0.02)
            continue
        df = _angle_diff(target_bear, hdg)
        if abs(df) <= SPIN_TOLERANCE:
            _stop()
            with _mock_lock:
                _mock_heading = target_bear % 360
            return
        _spin_left() if df > 0 else _spin_right()
        time.sleep(0.02)


def _drive_to(name, tx, tz, threshold, slam, imu,
              heading_correction=True, reverse=False):
    """
    Drive until within threshold of (tx, tz).

    reverse=True  → use _reverse() to move.
                    Heading is re-locked to (bearing+180) every iteration
                    so _reverse() always travels straight toward the target.

    reverse=False → use _forward(), heading snapped toward target as before.
    """
    global _mock_heading

    # Initial heading snap
    pose = slam.get()
    if pose is not None:
        cx, cz = pose
        bear = _bearing(cx, cz, tx, tz)
        with _mock_lock:
            if reverse:
                _mock_heading = (bear + 180) % 360
            else:
                _mock_heading = bear % 360

    print(f"\n  -> {name} ({tx:.3f}, {tz:.3f})")
    stuck  = 0
    last_d = 9999.0

    while True:
        pose = slam.get()
        if pose is None:
            print("  SLAM lost")
            _stop()
            time.sleep(0.5)
            continue

        cx, cz = pose
        d = _dist(cx, cz, tx, tz)
        print(f"    pos=({cx:.3f},{cz:.3f})  dist={d:.3f}")

        if d < threshold:
            _stop()
            print(f"  ok {name}")
            return

        if reverse:
            # Re-lock heading every step so mock reverse stays perfectly on axis
            bear = _bearing(cx, cz, tx, tz)
            with _mock_lock:
                _mock_heading = (bear + 180) % 360
        elif heading_correction and d > 0.15:
            br  = _bearing(cx, cz, tx, tz)
            hdg = imu.get() or 0.0
            df  = _angle_diff(br, hdg)
            if abs(df) > ANGLE_THRESHOLD:
                _stop()
                time.sleep(0.1)
                _correct_heading(br, imu)
                stuck = 0
                continue

        if abs(last_d - d) < 0.002:
            stuck += 1
        else:
            stuck = 0
        last_d = d

        if stuck > 80:
            print("  stuck")
            _stop()
            stuck = 0
            continue

        _reverse() if reverse else _forward()
        time.sleep(0.05)


# ─────────────────────────────────────────────────────────────────────────────
#  Navigator
# ─────────────────────────────────────────────────────────────────────────────

class Navigator:

    def __init__(self, stations_file):
        with open(stations_file) as f:
            self._stations = json.load(f)

        self._imu  = ImuReader()
        self._slam = SlamReader()
        self._docked_at = "station1"  # mock: start already docked here

        print("[nav] Mock mode -- no real hardware")
        print(f"[nav] IMU  OK  hdg={self._imu.get():.1f} deg")
        cx, cz = self._slam.get()
        print(f"[nav] SLAM OK  pos=({cx:.4f}, {cz:.4f})")

    def Maps_to(self, target_name):
        if target_name not in self._stations:
            raise ValueError(f"Unknown station '{target_name}'. "
                             f"Available: {list(self._stations.keys())}")

        if self._docked_at is not None:
            print(f"\n[nav] departing '{self._docked_at}' before routing to '{target_name}'")
            self._depart()

        info        = self._stations[target_name]
        target_x    = info['x']
        target_z    = info['z']
        orientation = info.get('orientation', '')
        approach    = _parse_approach(orientation)

        pose = self._slam.get()
        if pose is None:
            print("ERROR: SLAM not available"); sys.exit(1)
        cx, cz = pose

        obstacles = [
            (s['x'], s['z'])
            for name, s in self._stations.items()
            if name != target_name and name != 'start'
        ]

        lateral     = _lateral_offset(cx, cz, target_x, target_z,
                                       approach[0], approach[1])
        dot         = ((target_x - cx) * approach[0] +
                       (target_z - cz) * approach[1])
        is_straight = lateral < STRAIGHT_THRESHOLD and dot > 0

        if is_straight:
            elbow_x, elbow_z = target_x, target_z

            print(f"\n[nav] -- Maps_to('{target_name}') -- STRAIGHT LINE --")
            print(f"  start    : ({cx:.3f}, {cz:.3f})")
            print(f"  target   : ({target_x:.3f}, {target_z:.3f})")
            print(f"  approach : {approach}  [{orientation}]")
            print(f"  lateral offset {lateral:.3f} m < {STRAIGHT_THRESHOLD} m -> no elbow")

        else:
            elbow_x, elbow_z = _pick_elbow(
                cx, cz, target_x, target_z,
                approach, obstacles, ELBOW_CLEARANCE,
            )

            v1x, v1z  = elbow_x - cx,        elbow_z - cz
            v2x, v2z  = target_x - elbow_x,  target_z - elbow_z
            cross      = _cross2d(v1x, v1z, v2x, v2z)
            turn_left  = cross > 0
            direction  = 'left' if turn_left else 'right'

            print(f"\n[nav] -- Maps_to('{target_name}') -- L-SHAPE --")
            print(f"  start    : ({cx:.3f}, {cz:.3f})")
            print(f"  elbow    : ({elbow_x:.3f}, {elbow_z:.3f})")
            print(f"  target   : ({target_x:.3f}, {target_z:.3f})")
            print(f"  approach : {approach}  [{orientation}]")
            print(f"  lateral offset {lateral:.3f} m")
            print(f"  spin     : 90 deg {direction}")

        stations_info = [
            {'name': n, 'x': s['x'], 'z': s['z'], 'is_target': n == target_name}
            for n, s in self._stations.items() if n != 'start'
        ]
        show_map(
            start=(cx, cz),
            elbow=(elbow_x, elbow_z),
            stations_info=stations_info,
            is_straight=is_straight,
        )

        if is_straight:
            bear = _bearing(cx, cz, target_x, target_z)
            input(f"\n  Press Enter for Phase 0 (spin to {bear:.1f} deg)...")
            print(f"\n[Phase 0] spin to face target  bearing={bear:.1f} deg")
            _spin_to_bearing(bear, self._imu)
            time.sleep(0.3)

            input("  Press Enter for Phase 3 (drive straight to station)...")
            print(f"\n[Phase 3] drive straight to station ({target_x:.3f}, {target_z:.3f})")
            _drive_to("station", target_x, target_z,
                      STATION_THRESHOLD, self._slam, self._imu,
                      heading_correction=False)

        else:
            bear_to_elbow = _bearing(cx, cz, elbow_x, elbow_z)
            input(f"\n  Press Enter for Phase 0 (spin to face elbow  {bear_to_elbow:.1f} deg)...")
            print(f"\n[Phase 0] spin to face elbow  bearing={bear_to_elbow:.1f} deg")
            _spin_to_bearing(bear_to_elbow, self._imu)
            time.sleep(0.3)

            input("  Press Enter for Phase 1 (drive to elbow)...")
            print(f"\n[Phase 1] drive to elbow ({elbow_x:.3f}, {elbow_z:.3f})")
            _drive_to("elbow", elbow_x, elbow_z,
                      WAYPOINT_THRESHOLD, self._slam, self._imu,
                      heading_correction=True)
            time.sleep(0.2)

            input(f"  Press Enter for Phase 2 (spin {direction} 90 deg)...")
            print(f"\n[Phase 2] spin {direction} 90 deg")
            _spin_delta(90, turn_left, self._imu)
            time.sleep(0.3)

            input("  Press Enter for Phase 3 (drive to station)...")
            print(f"\n[Phase 3] drive to station ({target_x:.3f}, {target_z:.3f})")
            _drive_to("station", target_x, target_z,
                      STATION_THRESHOLD, self._slam, self._imu,
                      heading_correction=False)

        self._docked_at = target_name
        print(f"\n[nav] arrived at '{target_name}'")
        print(f"[nav]   final pos: {self._slam.get()}")

    def _depart(self):
        """
        Reverse straight back from the station along the approach axis.
        Car is docked facing the wall (facing along approach vector).
        Back-out = opposite of approach vector.

        _drive_to with reverse=True now correctly snaps heading to
        (bearing+180) so _reverse() actually moves toward the clear point.
        """
        pose = self._slam.get()
        if pose is None:
            print("  [depart] WARNING: no SLAM -- skipping")
            self._docked_at = None
            return

        info        = self._stations[self._docked_at]
        orientation = info.get('orientation', '')
        approach    = _parse_approach(orientation)

        cx, cz  = pose
        clear_x = cx - approach[0] * DEPART_DIST
        clear_z = cz - approach[1] * DEPART_DIST

        print(f"  [depart] reversing from ({cx:.3f}, {cz:.3f}) "
              f"to ({clear_x:.3f}, {clear_z:.3f})  [{orientation}]")

        _drive_to("depart_clear", clear_x, clear_z,
                  WAYPOINT_THRESHOLD, self._slam, self._imu,
                  heading_correction=False, reverse=True)

        self._docked_at = None
        print("  [depart] done")


# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Navigate to named station(s) — MOCK mode."
    )
    parser.add_argument("targets", nargs="*", default=["station1"])
    parser.add_argument("--stations",
                        default="/home/boethius/autonomous_car/navigation/stations.json")
    args = parser.parse_args()

    nav = Navigator(args.stations)
    for t in args.targets:
        nav.Maps_to(t)
    print("\n[nav] all targets reached -- done")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        _stop()
        print("\n[nav] interrupted -- car stopped")
    except Exception as e:
        _stop()
        print(f"\n[nav] CRASH -- car stopped. Error: {e}")
        raise