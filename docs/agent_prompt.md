# Web Dashboard for Autonomous Car Navigation

## Goal
Build a web dashboard that replaces the current terminal + matplotlib workflow.
The user opens a browser, sees a live map, clicks a station, and the car drives there.
No more `python3 navigator.py` in the terminal. No more matplotlib popups.

---

## Project location
All files are in `/home/boethius/autonomous_car/navigation/`

```
navigation/
├── navigator.py          ← REAL hardware navigator (READ THIS FIRST)
├── test/
│   └── navigator.py      ← MOCK navigator for test mode (READ THIS SECOND)
├── stations.json         ← station definitions
└── /home/boethius/autonomous_car/data/MapPoints.txt  ← SLAM map points
```

Read both navigator.py files completely before writing any code.

---

## What to build

### 1. Backend — `server.py` (FastAPI)
Single file. Runs at `http://localhost:8080`.

**Two modes selected at startup:**
```bash
python3 server.py --mode real    # uses navigation/navigator.py
python3 server.py --mode test    # uses navigation/test/navigator.py
```

In test mode, import from `test/navigator.py`.
In real mode, import from `navigator.py`.
Both files expose the same `Navigator` class and `Maps_to(target_name)` method.
Do not duplicate navigation logic — import it directly.

**Endpoints:**

`GET /state`
Returns JSON updated every 200ms:
```json
{
  "pos": {"x": 0.03, "z": -0.007},
  "heading": 270.0,
  "docked_at": "station1",
  "phase": "idle",
  "mode": "test"
}
```
`phase` values: `"idle"`, `"departing"`, `"phase0"`, `"phase1"`, `"phase2"`, `"phase3"`

`GET /stations`
Returns the full contents of `stations.json`.

`POST /go`
Body: `{"target": "station2"}`
Runs `nav.Maps_to(target)` in a background thread.
Returns immediately with `{"status": "started"}`.
Returns `{"status": "busy"}` if already navigating.

`POST /stop`
Calls `_stop()` immediately and cancels current navigation.

`GET /map_hull`
Returns the room convex hull as a list of `[x, z]` points.
Compute it from `MapPoints.txt` using the same logic as `_load_hull()` in navigator.py:
- Filter points: X in [-0.60, 0.65], Z in [-0.50, 1.55], Y in [-0.14, 0.10]
- Take X and Z columns, compute ConvexHull, return closed polygon vertices.

`POST /stations` (mapping tool)
Body: full stations.json content.
Overwrites `stations.json` on disk.
Returns `{"status": "saved"}`.

---

### 2. Frontend — single `index.html`
Vanilla JS, no framework, no build step.
Served directly by FastAPI from the same `server.py`.

**Layout: two columns**
- Left: map canvas (SVG or Canvas, 400×600px minimum)
- Right: controls panel

**Map canvas — draw in this order:**
1. Room outline — filled dark polygon from `/map_hull` data (draw once on load)
2. All stations — colored dots with name labels
3. Standoff points — hollow rings connected to their station by a dashed line
   - Standoff = station_pos + orientation_vector * 0.30m
   - Orientation vectors: `-X Wall`→(+1,0), `+X Wall`→(-1,0), `-Z Wall`→(0,+1), `+Z Wall`→(0,-1)
4. Planned path — green line (leg 1) + yellow line (leg 2) + diamond at elbow
   - Only shown while navigating, cleared on idle
5. Car — white square with a heading arrow, updated from `/state` every 200ms

**Map coordinate system:**
- World X maps to canvas X (left/right)
- World Z maps to canvas Y (bottom=Z_MIN=-0.50, top=Z_MAX=1.55)
- Scale to fit canvas. Room bounds: X[-0.60, 0.65], Z[-0.50, 1.55]

**Controls panel:**
- Mode badge (TEST / REAL) — read from `/state`
- Current phase badge — idle / departing / phase0 / phase1 / phase2 / phase3
- Car position display — updates live
- Station buttons — one button per station from `/stations`
  - Clicking sends `POST /go` with that station name
  - Disabled while navigating
  - Highlight the docked station
- STOP button — always enabled, sends `POST /stop`
- Mapping tool section (collapsible):
  - Table of stations with editable name, x, z, orientation fields
  - Orientation dropdown: `-X Wall`, `+X Wall`, `-Z Wall`, `+Z Wall`
  - Save button — sends `POST /stations`
  - Add row / Delete row buttons

**Navigation status:**
- While navigating, show a log panel that streams what phase is running
- Use SSE (`/events`) or poll `/state` every 200ms — polling is fine

---

### 3. Phase status tracking
The Navigator class runs `Maps_to()` which calls phases sequentially.
The server needs to expose which phase is currently running.

Add a module-level status dict in `server.py`:
```python
nav_status = {
    "phase": "idle",
    "docked_at": None,
    "running": False
}
```

Wrap `Maps_to()` in a thread that updates `nav_status["phase"]` before each phase.
The Navigator class already prints phase names — mirror those to nav_status.

Do NOT modify the Navigator class. Instead subclass it or wrap Maps_to in the server.

---

## Hardware facts (from navigator.py — do not change these)

**Car commands:** HTTP POST to `http://localhost:5000/drive`
```json
{"w": true, "a": false, "s": false, "d": false, "total": 45, "inner": 33}
```

**IMU:** ZMQ SUB socket at `tcp://10.37.171.191:5556`
Receives JSON: `{"heading_deg": 270.0}`

**SLAM:** ZMQ SUB socket at `tcp://localhost:5557`
Receives JSON: `{"x": 0.03, "z": -0.007}`

**Test mode mocks:** all of the above are replaced by module-level globals
`_mock_heading` and `_mock_pose` in `test/navigator.py`.
In test mode, the `/state` endpoint reads those globals directly.

---

## stations.json format
```json
{
  "start": {"label": "Start", "x": 0.0296, "z": -0.0073, "orientation": "-Z Wall"},
  "station1": {"label": "station1", "x": -0.1653, "z": 0.4163, "orientation": "-X Wall"},
  "station2": {"label": "Station2", "x": 0.072, "z": 0.8483, "orientation": "+Z Wall"}
}
```

---

## Constraints
- Do not modify `navigator.py` or `test/navigator.py`
- Do not duplicate path planning logic — import it
- No React, no webpack, no npm — vanilla JS only
- `server.py` and `index.html` go in `/home/boethius/autonomous_car/navigation/web/`
- Must run with: `python3 web/server.py --mode test`
- Dependencies allowed: `fastapi`, `uvicorn`, `scipy`, `numpy`, `zmq`, `requests`
  (all already installed for the existing code)

---

## Deliverables
1. `web/server.py` — FastAPI backend
2. `web/index.html` — single file frontend
3. Brief usage note — how to start and what URL to open
