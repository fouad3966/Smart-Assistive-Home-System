#!/usr/bin/env python3
"""
mapping_tool.py — FastAPI mapping tool for the autonomous car.

Serves a dark-themed frontend at GET /
Saves stations via POST /save-station (reads live pose from ZMQ port 5557)

Run on Arch Linux:
    uvicorn mapping_tool:app --host 0.0.0.0 --port 8000 --reload
"""

import json
import os
import zmq
from pathlib import Path
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel

# ── Config ────────────────────────────────────────────────────────────────────
ZMQ_PUB_ADDR   = "tcp://localhost:5557"
STATIONS_FILE  = Path(__file__).parent / "stations.json"
ZMQ_TIMEOUT_MS = 3000   # give SLAM 3 s to deliver a valid pose

# ── FastAPI app ───────────────────────────────────────────────────────────────
app = FastAPI(title="Car Mapping Tool")


# ── Pydantic model ────────────────────────────────────────────────────────────
class SaveRequest(BaseModel):
    name: str
    orientation: str


# ── ZMQ helper ────────────────────────────────────────────────────────────────
def grab_pose() -> dict:
    """
    Subscribe to the SLAM ZMQ PUB socket, wait up to ZMQ_TIMEOUT_MS ms for a
    valid pose (ok=True, reset=False), then disconnect.

    Returns {"x": float, "z": float}.
    Raises RuntimeError if no valid pose arrives in time.
    """
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVTIMEO, ZMQ_TIMEOUT_MS)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")   # subscribe to all topics
    sock.connect(ZMQ_PUB_ADDR)

    try:
        deadline = ZMQ_TIMEOUT_MS   # total budget in ms
        while deadline > 0:
            try:
                raw = sock.recv_string()
                msg = json.loads(raw)
            except zmq.Again:
                raise RuntimeError(
                    f"No pose received from SLAM within {ZMQ_TIMEOUT_MS} ms. "
                    "Is slam_zmq.py running?"
                )

            if msg.get("ok") and not msg.get("reset"):
                return {"x": msg["x"], "z": msg["z"]}
            # got a lost/reset frame — keep waiting, budget is consumed per recv
    finally:
        sock.close()

    raise RuntimeError("Could not obtain a valid (non-reset) SLAM pose in time.")


# ── Station helpers ───────────────────────────────────────────────────────────
def load_stations() -> dict:
    if STATIONS_FILE.exists():
        with open(STATIONS_FILE, "r") as f:
            return json.load(f)
    return {}


def save_stations(data: dict) -> None:
    with open(STATIONS_FILE, "w") as f:
        json.dump(data, f, indent=2)


# ── API routes ────────────────────────────────────────────────────────────────
@app.post("/save-station")
async def save_station(req: SaveRequest):
    name = req.name.strip()
    if not name:
        raise HTTPException(status_code=422, detail="Station name cannot be empty.")

    try:
        pose = grab_pose()
    except RuntimeError as e:
        raise HTTPException(status_code=503, detail=str(e))

    stations = load_stations()
    key = name.lower().replace(" ", "_")
    stations[key] = {
        "label":       name,
        "x":           round(pose["x"], 6),
        "z":           round(pose["z"], 6),
        "orientation": req.orientation,
    }
    save_stations(stations)

    return {
        "saved":       key,
        "label":       name,
        "x":           pose["x"],
        "z":           pose["z"],
        "orientation": req.orientation,
    }


@app.get("/stations")
async def list_stations():
    return load_stations()


@app.delete("/stations/{key}")
async def delete_station(key: str):
    stations = load_stations()
    if key not in stations:
        raise HTTPException(status_code=404, detail=f"Station '{key}' not found.")
    del stations[key]
    save_stations(stations)
    return {"deleted": key}


# ── Frontend ──────────────────────────────────────────────────────────────────
HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<title>Station Mapper</title>
<link rel="preconnect" href="https://fonts.googleapis.com"/>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Syne:wght@400;700;800&display=swap"/>
<style>
  /* ── Reset & base ─────────────────────────────────────────────────────── */
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

  :root {
    --bg:       #090c0f;
    --surface:  #0e1318;
    --border:   #1c2730;
    --accent:   #00e5a0;
    --accent2:  #0099ff;
    --muted:    #3a4a56;
    --text:     #c8d8e0;
    --text-dim: #607080;
    --danger:   #ff4d6a;
    --success:  #00e5a0;
    --mono:     'Share Tech Mono', monospace;
    --sans:     'Syne', sans-serif;
    --radius:   4px;
  }

  html, body {
    height: 100%;
    background: var(--bg);
    color: var(--text);
    font-family: var(--mono);
    font-size: 14px;
    line-height: 1.5;
  }

  /* ── Grid background ──────────────────────────────────────────────────── */
  body::before {
    content: '';
    position: fixed;
    inset: 0;
    background-image:
      linear-gradient(var(--border) 1px, transparent 1px),
      linear-gradient(90deg, var(--border) 1px, transparent 1px);
    background-size: 40px 40px;
    opacity: 0.5;
    pointer-events: none;
    z-index: 0;
  }

  /* ── Layout ───────────────────────────────────────────────────────────── */
  .page {
    position: relative;
    z-index: 1;
    min-height: 100vh;
    display: grid;
    grid-template-rows: auto 1fr auto;
    max-width: 860px;
    margin: 0 auto;
    padding: 0 24px;
  }

  /* ── Header ───────────────────────────────────────────────────────────── */
  header {
    padding: 32px 0 24px;
    border-bottom: 1px solid var(--border);
    display: flex;
    align-items: baseline;
    gap: 18px;
  }
  header h1 {
    font-family: var(--sans);
    font-weight: 800;
    font-size: 26px;
    letter-spacing: -0.5px;
    color: #fff;
  }
  header h1 span { color: var(--accent); }
  .tagline {
    font-size: 11px;
    color: var(--text-dim);
    letter-spacing: 0.08em;
    text-transform: uppercase;
  }

  /* ── Main ─────────────────────────────────────────────────────────────── */
  main {
    padding: 32px 0;
    display: grid;
    grid-template-columns: 340px 1fr;
    gap: 28px;
    align-items: start;
  }

  /* ── Card ─────────────────────────────────────────────────────────────── */
  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    padding: 24px;
  }
  .card-title {
    font-family: var(--sans);
    font-weight: 700;
    font-size: 12px;
    letter-spacing: 0.12em;
    text-transform: uppercase;
    color: var(--accent);
    margin-bottom: 20px;
  }

  /* ── Form ─────────────────────────────────────────────────────────────── */
  .form-group {
    margin-bottom: 16px;
  }
  label {
    display: block;
    font-size: 11px;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    color: var(--text-dim);
    margin-bottom: 6px;
  }
  input[type="text"],
  select {
    width: 100%;
    background: var(--bg);
    border: 1px solid var(--muted);
    border-radius: var(--radius);
    color: var(--text);
    font-family: var(--mono);
    font-size: 14px;
    padding: 10px 12px;
    outline: none;
    transition: border-color 0.15s;
    appearance: none;
    -webkit-appearance: none;
  }
  input[type="text"]:focus,
  select:focus { border-color: var(--accent); }

  .select-wrap {
    position: relative;
  }
  .select-wrap::after {
    content: '▾';
    position: absolute;
    right: 12px;
    top: 50%;
    transform: translateY(-50%);
    color: var(--accent);
    pointer-events: none;
    font-size: 12px;
  }

  /* ── Save button ──────────────────────────────────────────────────────── */
  #save-btn {
    width: 100%;
    margin-top: 8px;
    padding: 12px;
    background: transparent;
    border: 1px solid var(--accent);
    border-radius: var(--radius);
    color: var(--accent);
    font-family: var(--mono);
    font-size: 13px;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    cursor: pointer;
    transition: background 0.15s, color 0.15s, box-shadow 0.15s;
    position: relative;
    overflow: hidden;
  }
  #save-btn:hover:not(:disabled) {
    background: var(--accent);
    color: var(--bg);
    box-shadow: 0 0 18px rgba(0,229,160,0.35);
  }
  #save-btn:disabled {
    opacity: 0.5;
    cursor: not-allowed;
  }
  #save-btn.loading::after {
    content: '';
    position: absolute;
    bottom: 0; left: -100%; width: 100%; height: 2px;
    background: var(--accent);
    animation: scan 1.2s linear infinite;
  }
  @keyframes scan {
    to { left: 100%; }
  }

  /* ── Status banner ────────────────────────────────────────────────────── */
  #status {
    margin-top: 14px;
    padding: 10px 12px;
    border-radius: var(--radius);
    font-size: 12px;
    display: none;
    line-height: 1.6;
  }
  #status.ok   { display:block; background:#00e5a010; border:1px solid var(--success); color:var(--success); }
  #status.err  { display:block; background:#ff4d6a10; border:1px solid var(--danger);  color:var(--danger);  }

  /* ── Station list ─────────────────────────────────────────────────────── */
  .stations-panel { }
  .stations-panel .card-title { margin-bottom: 14px; }

  #station-list {
    display: flex;
    flex-direction: column;
    gap: 8px;
  }

  .station-row {
    display: grid;
    grid-template-columns: 1fr auto auto auto;
    align-items: center;
    gap: 12px;
    padding: 10px 14px;
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: var(--radius);
    animation: fadeIn 0.25s ease;
  }
  @keyframes fadeIn {
    from { opacity:0; transform: translateY(4px); }
    to   { opacity:1; transform: translateY(0);   }
  }
  .stn-name {
    font-family: var(--sans);
    font-weight: 700;
    font-size: 13px;
    color: #fff;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  .stn-coords {
    font-size: 11px;
    color: var(--text-dim);
    white-space: nowrap;
  }
  .stn-orient {
    font-size: 11px;
    color: var(--accent2);
    background: #0099ff18;
    border: 1px solid #0099ff40;
    border-radius: 3px;
    padding: 2px 6px;
    white-space: nowrap;
  }

  .stn-delete {
    background: transparent;
    border: 1px solid transparent;
    border-radius: var(--radius);
    color: var(--muted);
    font-family: var(--mono);
    font-size: 13px;
    line-height: 1;
    padding: 4px 7px;
    cursor: pointer;
    transition: color 0.15s, border-color 0.15s, background 0.15s;
  }
  .stn-delete:hover {
    color: var(--danger);
    border-color: var(--danger);
    background: #ff4d6a12;
  }
  .station-row.deleting {
    opacity: 0;
    transform: translateX(10px);
    transition: opacity 0.2s ease, transform 0.2s ease;
  }

  .empty-hint {
    font-size: 12px;
    color: var(--muted);
    padding: 16px 0;
    text-align: center;
  }

  /* ── Footer ───────────────────────────────────────────────────────────── */
  footer {
    padding: 16px 0;
    border-top: 1px solid var(--border);
    font-size: 11px;
    color: var(--muted);
    display: flex;
    justify-content: space-between;
  }
  #zmq-indicator {
    display: flex;
    align-items: center;
    gap: 6px;
  }
  #zmq-dot {
    width: 6px; height: 6px;
    border-radius: 50%;
    background: var(--muted);
    transition: background 0.3s;
  }
  #zmq-dot.live { background: var(--accent); box-shadow: 0 0 6px var(--accent); }
  #zmq-dot.err  { background: var(--danger); }
</style>
</head>
<body>
<div class="page">

  <header>
    <h1>STATION <span>MAPPER</span></h1>
    <span class="tagline">autonomous car · pose capture</span>
  </header>

  <main>
    <!-- ── Save form ── -->
    <div class="card">
      <div class="card-title">▸ Capture Station</div>

      <div class="form-group">
        <label for="stn-name">Station Name</label>
        <input id="stn-name" type="text" placeholder="e.g. charging_dock" autocomplete="off"/>
      </div>

      <div class="form-group">
        <label for="stn-orient">Orientation</label>
        <div class="select-wrap">
          <select id="stn-orient">
            <option value="+X Wall">+X Wall</option>
            <option value="-X Wall">-X Wall</option>
            <option value="+Z Wall">+Z Wall</option>
            <option value="-Z Wall">-Z Wall</option>
          </select>
        </div>
      </div>

      <button id="save-btn" onclick="saveStation()">⬡ Save Current Position</button>
      <div id="status"></div>
    </div>

    <!-- ── Station list ── -->
    <div class="card stations-panel">
      <div class="card-title">▸ Saved Stations</div>
      <div id="station-list">
        <div class="empty-hint">Loading stations…</div>
      </div>
    </div>
  </main>

  <footer>
    <span>stations.json · live</span>
    <span id="zmq-indicator">
      <span id="zmq-dot"></span>
      <span id="zmq-label">ZMQ tcp://localhost:5557</span>
    </span>
  </footer>

</div>

<script>
  /* ── Load existing stations on page load ──────────────────────────────── */
  async function loadStations() {
    try {
      const res  = await fetch('/stations');
      const data = await res.json();
      renderStations(data);
    } catch (e) {
      document.getElementById('station-list').innerHTML =
        '<div class="empty-hint">Could not load stations.</div>';
    }
  }

  function renderStations(data) {
    const list = document.getElementById('station-list');
    const keys = Object.keys(data);
    if (!keys.length) {
      list.innerHTML = '<div class="empty-hint">No stations saved yet.</div>';
      return;
    }
    list.innerHTML = keys.map(k => {
      const s     = data[k];
      const label = s.label || k;
      const x     = (s.x !== undefined && s.x !== null) ? s.x.toFixed(4) : '?';
      const z     = (s.z !== undefined && s.z !== null) ? s.z.toFixed(4) : '?';
      const ori   = s.orientation || '—';
      return `
        <div class="station-row" id="row-${escHtml(k)}">
          <span class="stn-name">${escHtml(label)}</span>
          <span class="stn-coords">x ${x} · z ${z}</span>
          <span class="stn-orient">${escHtml(ori)}</span>
          <button class="stn-delete" title="Delete station" onclick="deleteStation('${escHtml(k)}')">✕</button>
        </div>`;
    }).join('');
  }

  function escHtml(s) {
    return String(s)
      .replace(/&/g,'&amp;').replace(/</g,'&lt;')
      .replace(/>/g,'&gt;').replace(/"/g,'&quot;');
  }

  /* ── Save station ─────────────────────────────────────────────────────── */
  async function saveStation() {
    const name   = document.getElementById('stn-name').value.trim();
    const orient = document.getElementById('stn-orient').value;
    const btn    = document.getElementById('save-btn');
    const status = document.getElementById('status');

    if (!name) {
      setStatus('err', '⚠ Station name is required.');
      document.getElementById('stn-name').focus();
      return;
    }

    btn.disabled = true;
    btn.classList.add('loading');
    btn.textContent = 'Reading SLAM pose…';
    setStatus('', '');

    try {
      const res  = await fetch('/save-station', {
        method:  'POST',
        headers: { 'Content-Type': 'application/json' },
        body:    JSON.stringify({ name, orientation: orient }),
      });
      const body = await res.json();

      if (!res.ok) {
        setStatus('err', '✗ ' + (body.detail || 'Unknown error'));
        setZmq('err');
      } else {
        setStatus('ok',
          `✓ Saved "${body.label}"\n` +
          `  x = ${body.x.toFixed(6)}  z = ${body.z.toFixed(6)}\n` +
          `  orientation: ${body.orientation}`
        );
        setZmq('live');
        document.getElementById('stn-name').value = '';
        await loadStations();
      }
    } catch (e) {
      setStatus('err', '✗ Network error: ' + e.message);
      setZmq('err');
    } finally {
      btn.disabled = false;
      btn.classList.remove('loading');
      btn.textContent = '⬡ Save Current Position';
    }
  }

  function setStatus(cls, msg) {
    const el = document.getElementById('status');
    el.className = cls;
    el.style.whiteSpace = 'pre';
    el.textContent = msg;
  }

  function setZmq(state) {
    const dot   = document.getElementById('zmq-dot');
    const label = document.getElementById('zmq-label');
    dot.className = state;
    if (state === 'live') label.textContent = 'ZMQ · pose captured';
    if (state === 'err')  label.textContent = 'ZMQ · unreachable';
  }

  /* ── Delete station ───────────────────────────────────────────────────── */
  async function deleteStation(key) {
    const row = document.getElementById('row-' + key);
    if (row) row.classList.add('deleting');

    try {
      const res = await fetch('/stations/' + encodeURIComponent(key), { method: 'DELETE' });
      if (!res.ok) {
        const body = await res.json();
        alert('Delete failed: ' + (body.detail || res.status));
        if (row) row.classList.remove('deleting');
        return;
      }
      // Let the fade-out finish before reloading the list
      setTimeout(loadStations, 220);
    } catch (e) {
      alert('Network error: ' + e.message);
      if (row) row.classList.remove('deleting');
    }
  }

  /* ── Allow Enter key to save ──────────────────────────────────────────── */
  document.getElementById('stn-name')
    .addEventListener('keydown', e => { if (e.key === 'Enter') saveStation(); });

  /* ── Boot ─────────────────────────────────────────────────────────────── */
  loadStations();
</script>
</body>
</html>"""


@app.get("/", response_class=HTMLResponse)
async def index():
    return HTMLResponse(content=HTML)
