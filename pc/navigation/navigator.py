"""
navigator.py — Reusable Navigator class for L-shape station routing.

Step 1 scope (this file):
  - Navigator(stations_file)       load stations.json, init ZMQ/IMU/SLAM
  - Maps_to(target_name)           full 3-phase drive to standoff point
  - _depart()                      back out 0.3 m before the next Maps_to call

NOT in this step:
  - SLAM loss / map-reset handling  (Step 2)
  - slam_zmq ACK handshake          (Step 2)

Orientation strings in stations.json → standoff offset:
  "-X Wall"  : station faces -X, car approaches from +X  → standoff at +X
  "+X Wall"  : station faces +X, car approaches from -X  → standoff at -X
  "-Z Wall"  : station faces -Z, car approaches from +Z  → standoff at +Z
  "+Z Wall"  : station faces +Z, car approaches from -Z  → standoff at -Z
"""

import json
import math
import sys
import time
import threading

import requests
import zmq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly

# ── CONFIG ────────────────────────────────────────────────────────────────────
PI_HOST  = "localhost"
PI_PORT  = 5000
CAR_URL  = f"http://{PI_HOST}:{PI_PORT}/drive"

SLAM_HOST = "localhost"
SLAM_PORT = 5557

IMU_HOST  = "10.37.171.191"
IMU_PORT  = 5556

# ── TUNING ────────────────────────────────────────────────────────────────────
FORWARD_SPEED      = 45
REVERSE_SPEED      = 45
SPIN_SPEED         = 40
WAYPOINT_THRESHOLD = 0.10   # metres — elbow point
STATION_THRESHOLD  = 0.04   # metres — standoff arrival
ANGLE_THRESHOLD    = 15     # degrees — mid-drive correction trigger
SPIN_TOLERANCE     = 6      # degrees — spin done tolerance
STALL_TIMEOUT      = 3.0    # seconds — no progress → stall
DEPART_DIST        = 0.30   # metres — how far to back up before next trip

# ── ORIENTATION → STANDOFF VECTOR ────────────────────────────────────────────
# The vector points FROM the station TOWARD the car approach side.
# Standoff = station_pos + STANDOFF_DIST * vector
STANDOFF_DIST = 0.30   # metres

ORIENTATION_VECTOR = {
    "-X Wall": (+1,  0),   # station on -X wall  → approach from +X
    "+X Wall": (-1,  0),   # station on +X wall  → approach from -X
    "-Z Wall": ( 0, +1),   # station on -Z wall  → approach from +Z
    "+Z Wall": ( 0, -1),   # station on +Z wall  → approach from -Z
}

# ── MAP DISPLAY ───────────────────────────────────────────────────────────────
MAPPOINTS    = "/home/boethius/autonomous_car/data/MapPoints.txt"
X_MIN, X_MAX = -0.60,  0.65
Z_MIN, Z_MAX = -0.50,  1.55
Y_MIN, Y_MAX = -0.14,  0.10

# Station label colours so each station gets its own colour on the map
_STATION_COLORS = ['#ff3366', '#ff9900', '#aa44ff', '#00ccff', '#ffff00']

def _load_hull():
    """Load MapPoints.txt and return the 2-D convex hull boundary."""
    try:
        from scipy.spatial import ConvexHull
        from matplotlib.path import Path as MplPath
        pts = np.loadtxt(MAPPOINTS)
        px, py, pz = pts[:, 0], pts[:, 1], pts[:, 2]
        mask = ((px >= X_MIN) & (px <= X_MAX) &
                (pz >= Z_MIN) & (pz <= Z_MAX) &
                (py >= Y_MIN) & (py <= Y_MAX))
        pts2d = np.column_stack([px[mask], pz[mask]])
        hull  = ConvexHull(pts2d)
        hull_pts = pts2d[hull.vertices]
        return np.vstack([hull_pts, hull_pts[0]])   # closed polygon
    except Exception as e:
        print(f"  [map] could not load hull: {e}")
        return None


def show_map(start, elbow, stations_info):
    """
    Draw the room map with the planned L-shape path and all stations.

    Parameters
    ----------
    start         : (x, z)  current car position
    elbow         : (x, z)  elbow waypoint (corner of the L)
    stations_info : list of dicts, each with keys:
                      'name', 'x', 'z', 'standoff_x', 'standoff_z',
                      'is_target' (bool)
    """
    hull_closed = _load_hull()

    fig, ax = plt.subplots(figsize=(8, 12))
    fig.patch.set_facecolor('#0a0a0f')
    ax.set_facecolor('#0d1117')

    # Room outline
    if hull_closed is not None:
        room = MplPoly(hull_closed, closed=True,
                       facecolor='#0d1f2d', edgecolor='#2a6090',
                       linewidth=2, zorder=1)
        ax.add_patch(room)

    target = next((s for s in stations_info if s['is_target']), None)

    # ── Non-target stations: dot + hollow standoff + dashed connector ──────
    for i, s in enumerate(stations_info):
        if s['is_target']:
            continue
        color = _STATION_COLORS[i % len(_STATION_COLORS)]
        # Station dot
        ax.scatter(s['x'], s['z'], c=color, marker='o',
                   s=120, zorder=6, edgecolors='white', linewidths=1.0)
        ax.annotate(s['name'].upper(), (s['x'], s['z']),
                    textcoords='offset points', xytext=(8, 4),
                    color=color, fontsize=9, fontweight='bold')
        # Standoff hollow ring
        ax.scatter(s['standoff_x'], s['standoff_z'],
                   facecolors='none', edgecolors=color,
                   s=70, linewidths=1.5, zorder=5)
        # Dashed line station → standoff
        ax.plot([s['x'], s['standoff_x']], [s['z'], s['standoff_z']],
                '--', color=color, linewidth=1.0, alpha=0.5, zorder=3)

    # ── Planned L-shape path: start → elbow → standoff ─────────────────────
    if target is not None:
        sx, sz   = start
        ex, ez   = elbow
        tx, tz   = target['standoff_x'], target['standoff_z']

        leg1_len = math.sqrt((ex - sx)**2 + (ez - sz)**2)
        leg2_len = math.sqrt((tx - ex)**2 + (tz - ez)**2)

        # Leg 1  start → elbow  — GREEN
        ax.plot([sx, ex], [sz, ez], '-', color='#00ff88', linewidth=3.5,
                alpha=0.95, zorder=4, label=f'leg 1  {leg1_len:.2f} m')
        if leg1_len > 0.02:
            mx, mz = (sx + ex) / 2, (sz + ez) / 2
            d1x = (ex - sx) / leg1_len * 0.04
            d1z = (ez - sz) / leg1_len * 0.04
            ax.annotate('', xy=(mx + d1x, mz + d1z),
                        xytext=(mx - d1x, mz - d1z),
                        arrowprops=dict(arrowstyle='->', color='#00ff88',
                                        lw=2.0), zorder=6)

        # Leg 2  elbow → standoff  — YELLOW
        ax.plot([ex, tx], [ez, tz], '-', color='#ffcc00', linewidth=3.5,
                alpha=0.95, zorder=4, label=f'leg 2  {leg2_len:.2f} m')
        if leg2_len > 0.02:
            mx2, mz2 = (ex + tx) / 2, (ez + tz) / 2
            d2x = (tx - ex) / leg2_len * 0.04
            d2z = (tz - ez) / leg2_len * 0.04
            ax.annotate('', xy=(mx2 + d2x, mz2 + d2z),
                        xytext=(mx2 - d2x, mz2 - d2z),
                        arrowprops=dict(arrowstyle='->', color='#ffcc00',
                                        lw=2.0), zorder=6)

        # Elbow corner marker
        ax.scatter([ex], [ez], c='#00ff88', s=120, marker='D', zorder=7)
        ax.annotate('SPIN HERE', (ex, ez),
                    textcoords='offset points', xytext=(10, 6),
                    color='#00ff88', fontsize=8)

        # Right-angle box at elbow  (works for any L orientation)
        cs = 0.025
        l1x = (ex - sx) / leg1_len if leg1_len > 0 else 0
        l1z = (ez - sz) / leg1_len if leg1_len > 0 else 0
        l2x = (tx - ex) / leg2_len if leg2_len > 0 else 0
        l2z = (tz - ez) / leg2_len if leg2_len > 0 else 0
        p1x, p1z = ex - l1x * cs,               ez - l1z * cs
        p2x, p2z = p1x + l2x * cs,              p1z + l2z * cs
        p3x, p3z = ex  + l2x * cs,              ez  + l2z * cs
        ax.plot([p1x, p2x, p3x], [p1z, p2z, p3z],
                '-', color='#00ff88', linewidth=1.2, alpha=0.7, zorder=5)

        # Target: star + standoff ring
        ti = next(i for i, s in enumerate(stations_info) if s['is_target'])
        tcolor = _STATION_COLORS[ti % len(_STATION_COLORS)]
        ax.scatter(target['x'], target['z'], c=tcolor, marker='*',
                   s=320, zorder=8, edgecolors='white', linewidths=1.5)
        ax.annotate(target['name'].upper(), (target['x'], target['z']),
                    textcoords='offset points', xytext=(10, 6),
                    color=tcolor, fontsize=10, fontweight='bold')
        ax.scatter(tx, tz, facecolors='none', edgecolors=tcolor,
                   s=120, linewidths=2.0, zorder=7)
        ax.annotate('standoff', (tx, tz),
                    textcoords='offset points', xytext=(8, -14),
                    color=tcolor, fontsize=7, alpha=0.8)

    # ── Start ───────────────────────────────────────────────────────────────
    ax.scatter(start[0], start[1], c='#ffffff', marker='s',
               s=160, zorder=9, label='Start')
    ax.annotate('START', (start[0], start[1]),
                textcoords='offset points', xytext=(8, 5),
                color='#ffffff', fontsize=9, fontweight='bold')

    ax.set_xlim(X_MIN, X_MAX)
    ax.set_ylim(Z_MIN, Z_MAX)
    ax.set_xlabel('X  →', color='#667788', fontsize=10)
    ax.set_ylabel('Z  →', color='#667788', fontsize=10)

    target_name = target['name'].upper() if target else '?'
    ax.set_title(
        (f"Route → {target_name}  |  green = leg 1   yellow = leg 2   ◆ = spin here"
         "\nClose window to continue"),
        color='#e0e0e0', fontsize=10, fontweight='bold', pad=10)

    ax.tick_params(colors='#334455')
    ax.grid(True, color='#111820', linewidth=0.5)
    for spine in ax.spines.values():
        spine.set_edgecolor('#1e2e3e')
    ax.set_aspect('equal')
    ax.legend(facecolor='#0a0a0f', edgecolor='#1e2e3e',
              labelcolor='#aabbcc', fontsize=9, loc='lower right')

    plt.tight_layout()
    print("\n  [map] window open — close it to continue")
    plt.show()

# ─────────────────────────────────────────────────────────────────────────────
#  Low-level car commands
# ─────────────────────────────────────────────────────────────────────────────

def _stop():
    for _ in range(3):
        try:
            requests.post(CAR_URL,
                json={'w': False, 'a': False, 's': False, 'd': False,
                      'total': 0, 'inner': 0},
                timeout=1.0)
        except Exception:
            pass
        time.sleep(0.05)


def _forward():
    try:
        requests.post(CAR_URL,
            json={'w': True,  'a': False, 's': False, 'd': False,
                  'total': FORWARD_SPEED,
                  'inner': int(FORWARD_SPEED * 0.75)},
            timeout=1.0)
    except Exception as e:
        print(f"  [forward] ERROR: {e}")


def _reverse():
    try:
        requests.post(CAR_URL,
            json={'w': False, 'a': False, 's': True, 'd': False,
                  'total': REVERSE_SPEED,
                  'inner': int(REVERSE_SPEED * 0.75)},
            timeout=1.0)
    except Exception as e:
        print(f"  [reverse] ERROR: {e}")


def _spin_left():
    try:
        requests.post(CAR_URL,
            json={'w': False, 'a': True, 's': False, 'd': False,
                  'total': SPIN_SPEED, 'inner': SPIN_SPEED},
            timeout=1.0)
    except Exception:
        pass


def _spin_right():
    try:
        requests.post(CAR_URL,
            json={'w': False, 'a': False, 's': False, 'd': True,
                  'total': SPIN_SPEED, 'inner': SPIN_SPEED},
            timeout=1.0)
    except Exception:
        pass


# ─────────────────────────────────────────────────────────────────────────────
#  ImuReader  (unchanged from path_finding.py)
# ─────────────────────────────────────────────────────────────────────────────

class ImuReader:
    def __init__(self, ctx: zmq.Context):
        self._heading = None
        self._lock    = threading.Lock()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt_string(zmq.SUBSCRIBE, '')
        sock.connect(f"tcp://{IMU_HOST}:{IMU_PORT}")
        self._sock = sock
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                with self._lock:
                    self._heading = json.loads(
                        self._sock.recv_string())['heading_deg']
            except Exception:
                pass

    def get(self) -> float | None:
        with self._lock:
            return self._heading


# ─────────────────────────────────────────────────────────────────────────────
#  SlamReader  (minimal — ok/reset awareness added in Step 2)
# ─────────────────────────────────────────────────────────────────────────────

class SlamReader:
    def __init__(self, ctx: zmq.Context):
        self._pose = None
        self._lock = threading.Lock()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt_string(zmq.SUBSCRIBE, '')
        sock.connect(f"tcp://{SLAM_HOST}:{SLAM_PORT}")
        sock.setsockopt(zmq.RCVTIMEO, 2000)
        self._sock = sock
        time.sleep(1)
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                data = json.loads(self._sock.recv_string())
                with self._lock:
                    self._pose = (data['x'], data['z'])
            except Exception:
                pass

    def get(self) -> tuple[float, float] | None:
        with self._lock:
            return self._pose


# ─────────────────────────────────────────────────────────────────────────────
#  Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────

def _dist(x1: float, z1: float, x2: float, z2: float) -> float:
    return math.sqrt((x2 - x1) ** 2 + (z2 - z1) ** 2)


def _bearing(cx: float, cz: float, tx: float, tz: float) -> float:
    """Compass bearing in degrees from (cx,cz) toward (tx,tz).
    0° = +Z, 90° = +X (right-hand world frame).
    """
    return math.degrees(math.atan2(tx - cx, tz - cz))


def _angle_diff(target: float, current: float) -> float:
    """Signed shortest angular distance  [-180, +180]."""
    return (target - current + 180) % 360 - 180


def _cross2d(ax: float, az: float, bx: float, bz: float) -> float:
    """2-D cross product a × b.  Positive → b is CCW (left) from a."""
    return ax * bz - az * bx


# ─────────────────────────────────────────────────────────────────────────────
#  Low-level motion primitives  (spin_delta, correct_heading, drive_to)
# ─────────────────────────────────────────────────────────────────────────────

def _spin_delta(degrees: float, turn_left: bool, imu: ImuReader) -> None:
    """Spin exactly `degrees` tracked by IMU delta."""
    start = imu.get()
    direction = 'left' if turn_left else 'right'
    print(f"  [spin] {direction} {degrees:.1f}°  from hdg={start:.1f}")
    last_turned = 0.0
    stall_start = time.time()

    while True:
        hdg = imu.get()
        if hdg is None:
            time.sleep(0.02)
            continue

        turned = (hdg - start) % 360 if turn_left else (start - hdg) % 360
        print(f"  hdg={hdg:.1f}  turned={turned:.1f}")

        if turned >= degrees - SPIN_TOLERANCE:
            _stop()
            time.sleep(0.3)
            print(f"  [spin] done — turned {turned:.1f}°")
            return

        if abs(turned - last_turned) > 0.5:
            last_turned  = turned
            stall_start  = time.time()
        elif time.time() - stall_start > STALL_TIMEOUT:
            _stop()
            input("  STALLED — fix manually then press Enter: ")
            return

        _spin_left() if turn_left else _spin_right()
        time.sleep(0.02)


def _correct_heading(target_bear: float, imu: ImuReader) -> None:
    """Correct heading to `target_bear` using the IMU."""
    print(f"  [heading] → {target_bear:.1f}°")
    stall_start = time.time()
    last_diff   = None

    while True:
        hdg = imu.get()
        if hdg is None:
            time.sleep(0.02)
            continue

        df = _angle_diff(target_bear, hdg)
        print(f"  hdg={hdg:.1f}  diff={df:.1f}")

        if abs(df) <= SPIN_TOLERANCE:
            _stop()
            time.sleep(0.2)
            print("  [heading] done")
            return

        if last_diff is not None and abs(abs(df) - abs(last_diff)) < 0.1:
            if time.time() - stall_start > STALL_TIMEOUT:
                _stop()
                input("  STALLED — fix manually then press Enter: ")
                return
        else:
            stall_start = time.time()
        last_diff = df

        _spin_left() if df > 0 else _spin_right()
        time.sleep(0.02)


def _drive_to(name: str, tx: float, tz: float, threshold: float,
              slam: SlamReader, imu: ImuReader,
              heading_correction: bool = True) -> None:
    """Drive forward until SLAM position is within `threshold` metres of (tx, tz)."""
    print(f"\n  → {name} ({tx:.3f}, {tz:.3f})")
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
        if cx is None or cz is None:
            print("  SLAM pose invalid — waiting")
            _stop()
            time.sleep(0.5)
            continue

        d = _dist(cx, cz, tx, tz)
        print(f"    pos=({cx:.3f},{cz:.3f})  dist={d:.3f}m")

        if d < threshold:
            _stop()
            print(f"  ✓ {name}")
            return

        if heading_correction and d > 0.15:
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
            print("  stuck — reversing")
            _reverse()
            time.sleep(0.4)
            _stop()
            stuck = 0
            continue

        _forward()
        time.sleep(0.1)


# ─────────────────────────────────────────────────────────────────────────────
#  Navigator
# ─────────────────────────────────────────────────────────────────────────────

class Navigator:
    """
    Usage:
        nav = Navigator("/path/to/stations.json")
        nav.Maps_to("station1")   # drive from wherever we are to station1
        nav.Maps_to("station2")   # automatically departs station1 first
    """

    def __init__(self, stations_file: str):
        with open(stations_file) as f:
            self._stations = json.load(f)

        self._ctx  = zmq.Context()
        self._imu  = ImuReader(self._ctx)
        self._slam = SlamReader(self._ctx)

        # Track where we just docked so _depart() knows how to back out.
        # None = we're at an unknown / open position, no departure needed.
        self._docked_at: str | None = None

        print("[nav] waiting for IMU …")
        for _ in range(40):
            if self._imu.get() is not None:
                break
            time.sleep(0.1)
        if self._imu.get() is None:
            print("ERROR: no IMU data"); sys.exit(1)
        print(f"[nav] IMU  OK  hdg={self._imu.get():.1f}°")

        print("[nav] waiting for SLAM …")
        for _ in range(20):
            if self._slam.get() is not None:
                break
            time.sleep(0.2)
        if self._slam.get() is None:
            print("ERROR: no SLAM data"); sys.exit(1)
        cx, cz = self._slam.get()
        print(f"[nav] SLAM OK  pos=({cx:.4f}, {cz:.4f})")

    # ── public API ────────────────────────────────────────────────────────────

    def Maps_to(self, target_name: str) -> None:
        """
        Navigate from current SLAM position to the standoff point in front of
        `target_name` using a 3-phase L-shape path.

        Phase 1 — drive forward to the elbow (corner of the L)
        Phase 2 — spin 90° (direction derived from geometry)
        Phase 3 — drive straight into the standoff point
        """
        if target_name not in self._stations:
            raise ValueError(
                f"Unknown station '{target_name}'. "
                f"Available: {list(self._stations.keys())}"
            )

        # ── Step 0: depart current station if docked ──────────────────────
        if self._docked_at is not None:
            print(f"\n[nav] departing '{self._docked_at}' before routing to '{target_name}'")
            self._depart()

        # ── Resolve target geometry ───────────────────────────────────────
        info        = self._stations[target_name]
        target_x    = info['x']
        target_z    = info['z']
        orientation = info.get('orientation', '')

        standoff_x, standoff_z = self._standoff_point(
            target_x, target_z, orientation
        )

        # ── Current position ──────────────────────────────────────────────
        pose = self._slam.get()
        if pose is None:
            print("ERROR: SLAM not available"); sys.exit(1)
        cx, cz = pose

        # ── Elbow point ───────────────────────────────────────────────────
        # X-Wall stations (-X Wall / +X Wall):
        #   standoff is sideways from station → drive along Z first, then X
        #   elbow = (start_x, standoff_z)
        #
        # Z-Wall stations (+Z Wall / -Z Wall):
        #   standoff is ahead/behind station → drive along X first, then Z
        #   elbow = (standoff_x, start_z)
        #
        # This ensures the car always arrives at the standoff facing INTO the station.
        if orientation in ('-X Wall', '+X Wall'):
            elbow_x, elbow_z = cx, standoff_z
        else:   # '-Z Wall' or '+Z Wall'
            elbow_x, elbow_z = standoff_x, cz

        # ── Spin direction from cross product ─────────────────────────────
        # cross > 0  →  leg 2 is CCW from leg 1  →  turn LEFT
        # cross < 0  →  leg 2 is CW  from leg 1  →  turn RIGHT
        _ax, _az = elbow_x - cx,            elbow_z - cz          # leg 1
        _bx, _bz = standoff_x - elbow_x,   standoff_z - elbow_z  # leg 2
        cross      = _cross2d(_ax, _az, _bx, _bz)
        turn_left  = cross > 0
        direction  = 'left' if turn_left else 'right'

        # ── Summary ───────────────────────────────────────────────────────
        print(f"\n[nav] ── Maps_to('{target_name}') ──────────────────────")
        print(f"  start    : ({cx:.3f}, {cz:.3f})")
        print(f"  elbow    : ({elbow_x:.3f}, {elbow_z:.3f})")
        print(f"  standoff : ({standoff_x:.3f}, {standoff_z:.3f})")
        print(f"             [{orientation}  →  0.3 m clearance]")
        print(f"  spin     : 90° {direction}")

        # ── Map ───────────────────────────────────────────────────────────
        stations_info = []
        for _sname, _sinfo in self._stations.items():
            if _sname == 'start':
                continue
            _ox = _sinfo['x']
            _oz = _sinfo['z']
            _ori = _sinfo.get('orientation', '')
            _sx, _sz = self._standoff_point(_ox, _oz, _ori)
            stations_info.append({
                'name':       _sname,
                'x':          _ox,
                'z':          _oz,
                'standoff_x': _sx,
                'standoff_z': _sz,
                'is_target':  _sname == target_name,
            })
        show_map(
            start=(cx, cz),
            elbow=(elbow_x, elbow_z),
            stations_info=stations_info,
        )

        input("\n  Press Enter to start Phase 1 (elbow)…")

        # ── Phase 1: drive to elbow ───────────────────────────────────────
        print(f"\n[Phase 1] drive to elbow ({elbow_x:.3f}, {elbow_z:.3f})")
        _drive_to("elbow", elbow_x, elbow_z,
                  WAYPOINT_THRESHOLD, self._slam, self._imu,
                  heading_correction=True)
        time.sleep(0.3)

        # ── Phase 2: spin 90° ─────────────────────────────────────────────
        input(f"  Press Enter to start Phase 2 (spin {direction} 90°)…")
        print(f"\n[Phase 2] spin {direction} 90°")
        _spin_delta(90, turn_left, self._imu)
        time.sleep(0.5)

        # ── Phase 3: drive straight to standoff (no heading correction) ───
        # The spin already aligned the car with the final leg.
        # Re-correcting mid-dock causes jitter — skip it here.
        input(f"  Press Enter to start Phase 3 (dock into standoff)…")
        print(f"\n[Phase 3] drive to standoff ({standoff_x:.3f}, {standoff_z:.3f})")
        _drive_to("standoff", standoff_x, standoff_z,
                  STATION_THRESHOLD, self._slam, self._imu,
                  heading_correction=False)

        # ── Record docked state ───────────────────────────────────────────
        self._docked_at = target_name
        print(f"\n[nav] ✓ arrived at '{target_name}' standoff")
        print(f"[nav]   final pos: {self._slam.get()}")

    # ── private helpers ───────────────────────────────────────────────────────

    def _depart(self) -> None:
        """
        Back the car out DEPART_DIST metres from the current standoff position.

        Why reverse instead of forward:
          At the standoff the car is facing the station wall.
          Reversing pulls it away from the wall into open space,
          so the next Maps_to() can plan a clean L-shape without
          starting right up against an obstacle.

        The move is tracked by SLAM distance, not time, so it is
        accurate regardless of battery level.
        """
        pose = self._slam.get()
        if pose is None:
            print("  [depart] WARNING: no SLAM — skipping departure")
            self._docked_at = None
            return

        start_x, start_z = pose
        target_dist = DEPART_DIST

        print(f"  [depart] reversing {DEPART_DIST:.2f} m from ({start_x:.3f}, {start_z:.3f})")

        stall_count = 0
        last_d      = 0.0

        while True:
            pose = self._slam.get()
            if pose is None:
                _stop()
                time.sleep(0.3)
                continue

            cx, cz = pose
            travelled = _dist(start_x, start_z, cx, cz)
            print(f"    reversed {travelled:.3f} m / {target_dist:.2f} m")

            if travelled >= target_dist:
                _stop()
                print(f"  [depart] ✓ backed out {travelled:.3f} m")
                self._docked_at = None
                return

            if abs(travelled - last_d) < 0.002:
                stall_count += 1
            else:
                stall_count = 0
            last_d = travelled

            if stall_count > 80:
                _stop()
                input("  [depart] STALLED — fix manually then press Enter: ")
                stall_count = 0
                continue

            _reverse()
            time.sleep(0.1)

    def _standoff_point(
        self, sx: float, sz: float, orientation: str
    ) -> tuple[float, float]:
        """
        Return the (x, z) standoff position 0.3 m in front of the station.

        'In front' means on the approach side — i.e. the side the car
        will drive in from.  The ORIENTATION_VECTOR table encodes this.
        """
        if orientation not in ORIENTATION_VECTOR:
            print(
                f"  [nav] WARNING: unknown orientation '{orientation}' "
                f"— driving directly to station coordinates (no standoff)"
            )
            return sx, sz

        vx, vz = ORIENTATION_VECTOR[orientation]
        return sx + vx * STANDOFF_DIST, sz + vz * STANDOFF_DIST


# ─────────────────────────────────────────────────────────────────────────────
#  CLI entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Navigate to a named station using L-shape routing."
    )
    parser.add_argument(
        "target",
        nargs="?",
        default="station1",
        help="Station name from stations.json (default: station1)",
    )
    parser.add_argument(
        "--stations",
        default="/home/boethius/autonomous_car/navigation/stations.json",
        help="Path to stations.json",
    )
    args = parser.parse_args()

    nav = Navigator(args.stations)
    nav.Maps_to(args.target)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        _stop()
        print("\n[nav] interrupted — car stopped")
    except Exception as e:
        _stop()
        print(f"\n[nav] CRASH — car stopped. Error: {e}")
        raise
