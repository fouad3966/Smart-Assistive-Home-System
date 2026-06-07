#!/usr/bin/env python3
"""
server.py — FastAPI web dashboard for autonomous car navigation.

Usage:
    python3 web/server.py --mode test     # mock navigator
    python3 web/server.py --mode real     # real hardware
"""

import argparse
import builtins
import json
import os
import sys
import threading
import time
import re
import io

import matplotlib
matplotlib.use('Agg')           # headless — must come before any pyplot import

import numpy as np
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse, FileResponse
import uvicorn

# ─── Paths ────────────────────────────────────────────────────────────────────
BASE_DIR      = os.path.dirname(os.path.abspath(__file__))
NAV_DIR       = os.path.dirname(BASE_DIR)                       # navigation/
PROJECT_DIR   = os.path.dirname(NAV_DIR)                        # autonomous_car/
STATIONS_FILE = os.path.join(NAV_DIR, "stations.json")
MAPPOINTS     = os.path.join(PROJECT_DIR, "data", "MapPoints.txt")
INDEX_HTML     = os.path.join(BASE_DIR, "index.html")

# ─── Globals ──────────────────────────────────────────────────────────────────
MODE = "test"
nav  = None                       # Navigator instance (set after init)

nav_status = {
    "phase":     "idle",
    "docked_at": None,
    "running":   False,
    "path":      None,            # {start, elbow, target, is_straight} for frontend
}
_status_lock  = threading.Lock()
_nav_thread   = None

# ─── Convex hull (cached) ────────────────────────────────────────────────────
_hull_cache = None

def _compute_hull():
    global _hull_cache
    if _hull_cache is not None:
        return _hull_cache
    try:
        from scipy.spatial import ConvexHull
        pts = np.loadtxt(MAPPOINTS)
        px, py, pz = pts[:, 0], pts[:, 1], pts[:, 2]
        X_MIN, X_MAX = -0.60,  0.65
        Z_MIN, Z_MAX = -0.50,  1.55
        Y_MIN, Y_MAX = -0.14,  0.10
        mask = ((px >= X_MIN) & (px <= X_MAX) &
                (pz >= Z_MIN) & (pz <= Z_MAX) &
                (py >= Y_MIN) & (py <= Y_MAX))
        pts2d = np.column_stack([px[mask], pz[mask]])
        hull = ConvexHull(pts2d)
        hull_pts = pts2d[hull.vertices].tolist()
        hull_pts.append(hull_pts[0])       # close polygon
        _hull_cache = hull_pts
        return _hull_cache
    except Exception as e:
        print(f"[server] hull error: {e}")
        return []


# ─── Phase-tracking stdout wrapper ───────────────────────────────────────────
_PHASE_RE = re.compile(
    r"\[Phase (\d)\]"
    r"|\[nav\] departing"
    r"|\[nav\] arrived"
    r"|\[depart\]"
)

class _PhaseCapture(io.TextIOBase):
    """Wraps stdout, intercepts navigator prints to update nav_status."""
    def __init__(self, real_stdout):
        self._real = real_stdout

    def write(self, s):
        self._real.write(s)
        m = _PHASE_RE.search(s)
        if m:
            with _status_lock:
                if m.group(1):                     # [Phase 0] … [Phase 3]
                    nav_status["phase"] = f"phase{m.group(1)}"
                elif "departing" in s:
                    nav_status["phase"] = "departing"
                elif "arrived" in s:
                    nav_status["phase"] = "idle"
                    nav_status["running"] = False
                    nav_status["path"] = None
        return len(s)

    def flush(self):
        self._real.flush()

    # Forward attributes so libs don't crash
    def fileno(self):          return self._real.fileno()
    def isatty(self):          return False
    @property
    def encoding(self):        return getattr(self._real, 'encoding', 'utf-8')


# ─── Navigation runner ───────────────────────────────────────────────────────
def _run_navigation(target_name: str):
    """Run Maps_to in a thread with input() disabled."""
    _original_input = builtins.input
    builtins.input = lambda *a, **kw: None            # skip "Press Enter…"

    # Suppress matplotlib show_map (replaced by web canvas)
    if MODE == "test":
        import navigation_test_mod as tn
        _orig_show = tn.show_map
        tn.show_map = lambda *a, **kw: None
    else:
        import navigation_real_mod as rn
        _orig_show = rn.show_map
        rn.show_map = lambda *a, **kw: None

    try:
        with _status_lock:
            nav_status["running"] = True
            nav_status["phase"]   = "planning"
        nav.Maps_to(target_name)
        with _status_lock:
            nav_status["phase"]     = "idle"
            nav_status["running"]   = False
            nav_status["docked_at"] = nav._docked_at
            nav_status["path"]      = None
    except Exception as e:
        print(f"[server] navigation error: {e}")
        with _status_lock:
            nav_status["phase"]   = "idle"
            nav_status["running"] = False
            nav_status["path"]    = None
    finally:
        builtins.input = _original_input
        if MODE == "test":
            tn.show_map = _orig_show
        else:
            rn.show_map = _orig_show


def _compute_planned_path(target_name: str):
    """Pre-compute path geometry so the frontend can draw it before execution."""
    with open(STATIONS_FILE) as f:
        stations = json.load(f)

    info        = stations[target_name]
    target_x    = info["x"]
    target_z    = info["z"]
    orientation = info.get("orientation", "")

    # Standoff point (same logic as navigator)
    STANDOFF_DIST = 0.30
    ORI_VEC = {
        "-X Wall": (+1,  0),
        "+X Wall": (-1,  0),
        "-Z Wall": ( 0, +1),
        "+Z Wall": ( 0, -1),
    }
    if orientation in ORI_VEC:
        vx, vz = ORI_VEC[orientation]
        so_x = target_x + vx * STANDOFF_DIST
        so_z = target_z + vz * STANDOFF_DIST
    else:
        so_x, so_z = target_x, target_z

    # Current car position
    if MODE == "test":
        import navigation_test_mod as tn
        with tn._mock_lock:
            cx, cz = tn._mock_pose
    else:
        pose = nav._slam.get()
        if pose is None:
            return None
        cx, cz = pose

    # Test navigator uses _pick_elbow; real uses orientation-based elbow.
    # We mirror the test navigator's logic here since it's more generic.
    if MODE == "test":
        approach = _parse_approach_simple(orientation)
        dx = target_x - cx
        dz = target_z - cz
        perp_x = -approach[1]
        perp_z =  approach[0]
        lateral = abs(dx * perp_x + dz * perp_z)
        dot     = dx * approach[0] + dz * approach[1]
        is_straight = lateral < 0.03 and dot > 0
        if is_straight:
            return {
                "start": [cx, cz],
                "elbow": [target_x, target_z],
                "target": [target_x, target_z],
                "standoff": [so_x, so_z],
                "is_straight": True,
            }
        # Pick elbow (simplified — matches test navigator)
        along_x = abs(approach[0]) > abs(approach[1])
        if along_x:
            elbow_x, elbow_z = cx, target_z
        else:
            elbow_x, elbow_z = target_x, cz
    else:
        # Real navigator logic (orientation-based)
        if orientation in ("-X Wall", "+X Wall"):
            elbow_x, elbow_z = cx, so_z
        else:
            elbow_x, elbow_z = so_x, cz
        is_straight = False

    return {
        "start": [cx, cz],
        "elbow": [elbow_x, elbow_z],
        "target": [target_x, target_z],
        "standoff": [so_x, so_z],
        "is_straight": is_straight if MODE == "test" else False,
    }


def _parse_approach_simple(orientation):
    if not orientation or len(orientation) < 2:
        return (0.0, 0.0)
    sign = +1.0 if orientation[0] == '+' else -1.0
    axis = orientation[1].upper()
    return (sign, 0.0) if axis == 'X' else (0.0, sign)


# ─── FastAPI app ──────────────────────────────────────────────────────────────
app = FastAPI()


@app.get("/", response_class=HTMLResponse)
async def root():
    return FileResponse(INDEX_HTML, media_type="text/html")


@app.get("/state")
async def get_state():
    pos = {"x": 0.0, "z": 0.0}
    heading = 0.0

    if MODE == "test":
        import navigation_test_mod as tn
        with tn._mock_lock:
            pos = {"x": tn._mock_pose[0], "z": tn._mock_pose[1]}
            heading = tn._mock_heading
    else:
        if nav and nav._slam.get():
            cx, cz = nav._slam.get()
            pos = {"x": cx, "z": cz}
        if nav and nav._imu.get():
            heading = nav._imu.get()

    with _status_lock:
        return {
            "pos":       pos,
            "heading":   heading,
            "docked_at": nav_status["docked_at"],
            "phase":     nav_status["phase"],
            "mode":      MODE,
            "path":      nav_status["path"],
        }


@app.get("/stations")
async def get_stations():
    with open(STATIONS_FILE) as f:
        return json.load(f)


@app.post("/go")
async def go(request: Request):
    global _nav_thread
    body = await request.json()
    target = body.get("target", "")

    with _status_lock:
        if nav_status["running"]:
            return {"status": "busy"}

    # Compute path for frontend
    path = _compute_planned_path(target)
    with _status_lock:
        nav_status["path"]    = path
        nav_status["running"] = True
        nav_status["phase"]   = "planning"

    _nav_thread = threading.Thread(target=_run_navigation, args=(target,), daemon=True)
    _nav_thread.start()
    return {"status": "started", "path": path}


@app.post("/stop")
async def stop():
    if MODE == "test":
        import navigation_test_mod as tn
        tn._stop()
    else:
        import navigation_real_mod as rn
        rn._stop()

    with _status_lock:
        nav_status["phase"]   = "idle"
        nav_status["running"] = False
        nav_status["path"]    = None
    return {"status": "stopped"}


@app.get("/map_hull")
async def map_hull():
    return _compute_hull()


@app.post("/stations", response_class=JSONResponse)
async def save_stations(request: Request):
    body = await request.json()
    with open(STATIONS_FILE, "w") as f:
        json.dump(body, f, indent=2)
    return {"status": "saved"}


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    global MODE, nav

    parser = argparse.ArgumentParser(description="Autonomous car web dashboard")
    parser.add_argument("--mode", choices=["test", "real"], default="test")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    MODE = args.mode

    # ── Import the correct navigator module ──────────────────────────
    # We import them under predictable names so the rest of the code
    # can reference them without if/else on every access.
    if MODE == "test":
        sys.path.insert(0, os.path.join(NAV_DIR, "test"))
        import navigator as _nav_mod
        sys.modules["navigation_test_mod"] = _nav_mod
    else:
        sys.path.insert(0, NAV_DIR)
        import navigator as _nav_mod
        sys.modules["navigation_real_mod"] = _nav_mod

    # Install phase-capture stdout wrapper
    sys.stdout = _PhaseCapture(sys.__stdout__)

    # Create Navigator
    nav = _nav_mod.Navigator(STATIONS_FILE)
    if MODE == "test":
        try:
            with open(STATIONS_FILE) as f:
                sts = json.load(f)
            if "start" in sts:
                _nav_mod._mock_pose = (sts["start"]["x"], sts["start"]["z"])
                _nav_mod._mock_heading = 0.0
                nav._docked_at = "start"
                print("[server] Forced test start position to 'start'")
        except:
            pass

    with _status_lock:
        nav_status["docked_at"] = nav._docked_at

    print(f"[server] mode={MODE}  port={args.port}")
    print(f"[server] http://localhost:{args.port}")

    uvicorn.run(app, host="0.0.0.0", port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
