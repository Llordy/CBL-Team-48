#!/usr/bin/env python3
"""
dashboard_node.py
Independent HTTP + WebSocket dashboard interface communicating purely via Shared Memory.
"""

import asyncio
import json
import math
import struct
import threading
import multiprocessing
from multiprocessing.shared_memory import SharedMemory

# ─────────────────────────────────────────
#  Constants
# ─────────────────────────────────────────
RADIUS         = 6371008.7714
CIRCUM         = 2 * math.pi * RADIUS
METERS_PER_DEG = CIRCUM / 360

SHM_NAME   = "navnode_state"
SHM_FORMAT = "13d?2d?"
SHM_SIZE   = struct.calcsize(SHM_FORMAT)

DASHBOARD_HTTP_PORT = 8080
DASHBOARD_WS_PORT   = 8765
DASHBOARD_HZ        = 5

# ─────────────────────────────────────────
#  Geometry & Extraction Helpers
# ─────────────────────────────────────────
def offset_position(lat, lon, north, east):
    dlat = north / METERS_PER_DEG
    dlon = east  / (METERS_PER_DEG * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon

def rotate(angle_rad, x, y):
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return c * x - s * y, s * x + c * y

def bearing_to(lat1, lon1, lat2, lon2):
    rl1, ro1, rl2, ro2 = map(math.radians, (lat1, lon1, lat2, lon2))
    x = math.cos(rl2) * math.sin(ro2 - ro1)
    y = math.cos(rl1) * math.sin(rl2) - math.sin(rl1) * math.cos(rl2) * math.cos(ro2 - ro1)
    return (720 + math.degrees(math.atan2(x, y))) % 360

def gps_distance(lat1, lon1, lat2, lon2):
    p1, o1, p2, o2 = map(math.radians, (lat1, lon1, lat2, lon2))
    hav = 0.5 * (1 - math.cos(p2 - p1) + math.cos(p1) * math.cos(p2) * (1 - math.cos(o2 - o1)))
    return 2 * math.asin(math.sqrt(hav)) * RADIUS

def shm_to_dict(shm: SharedMemory) -> dict:
    f = struct.unpack_from(SHM_FORMAT, shm.buf)
    gps_lat, gps_lon       = f[0],  f[1]
    gps_odom_x, gps_odom_y = f[2],  f[3]
    odom_x, odom_y         = f[4],  f[5]
    odom_heading            = f[6]
    compass_heading         = f[7]
    odom_at_compass         = f[8]
    goal_lat, goal_lon      = f[9],  f[10]
    has_goal                = f[13]

    print(f"gps pos: {gps_lat}, {gps_lon}")
    print(f"odom at gps: {gps_odom_x}, {gps_odom_y}")
    print(f"odom pos: {odom_x}, {odom_y}")

    est_lat, est_lon = gps_lat, gps_lon
    est_heading      = odom_heading
    if gps_lat != 0.0 or gps_lon != 0.0 or True:
        pos_offset  = (odom_x - gps_odom_x, -(odom_y - gps_odom_y))
        heading_err = compass_heading - odom_at_compass
        ne_offset   = rotate(math.radians(heading_err), *pos_offset)
        est_lat, est_lon = offset_position(gps_lat, gps_lon, *ne_offset)
        est_heading = (720 + odom_heading - heading_err) % 360
    print(f"estimated position: {est_lat}, {est_lon}")

    distance = None
    target_bearing = None
    if has_goal and (gps_lat != 0.0 or gps_lon != 0.0 or True):
        distance       = round(gps_distance(est_lat, est_lon, goal_lat, goal_lon), 2)
        target_bearing = round(bearing_to(est_lat, est_lon, goal_lat, goal_lon), 1)

    return {
        "gps":     {"lat": gps_lat,    "lon": gps_lon},
        "est_pos": {"lat": est_lat,    "lon": est_lon},
        "heading": {
            "odom":     round(odom_heading, 1),
            "compass":  round(compass_heading, 1),
            "fused":    round(est_heading, 1),
        },
        "goal": {
            "lat":        goal_lat,
            "lon":        goal_lon,
            "has_goal":   has_goal,
            "distance_m": distance,
            "bearing":    target_bearing,
        },
    }

# Embed Front-end HTML
DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Nav dashboard</title>
<style>
  :root {
    --bg:      #0d1117;
    --surface: #161b22;
    --border:  #30363d;
    --text:    #e6edf3;
    --muted:   #8b949e;
    --green:   #3fb950;
    --amber:   #d29922;
    --red:     #f85149;
    --blue:    #58a6ff;
    --mono:    'Courier New', monospace;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
    font-size: 14px;
    padding: 18px;
  }
  header { display: flex; align-items: center; gap: 10px; margin-bottom: 18px; padding-bottom: 14px; border-bottom: 1px solid var(--border); }
  header h1 { font-size: 15px; font-weight: 600; letter-spacing: .03em; flex: 1; }
  #status-dot { width: 9px; height: 9px; border-radius: 50%; background: var(--red); flex-shrink: 0; transition: background .3s; }
  #status-dot.ok { background: var(--green); }
  #status-text { color: var(--muted); font-size: 12px; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(270px, 1fr)); gap: 14px; }
  .card { background: var(--surface); border: 1px solid var(--border); border-radius: 8px; padding: 14px 16px; }
  .card-title { font-size: 10px; font-weight: 700; text-transform: uppercase; letter-spacing: .1em; color: var(--muted); margin-bottom: 11px; }
  .row { display: flex; justify-content: space-between; align-items: baseline; padding: 5px 0; border-bottom: 1px solid var(--border); }
  .row:last-child { border-bottom: none; }
  .lbl { color: var(--muted); font-size: 13px; }
  .val { font-family: var(--mono); font-size: 13px; }
  #compass-wrap { display: flex; justify-content: center; margin: 6px 0 10px; }
  .pill { display: inline-block; padding: 1px 9px; border-radius: 20px; font-size: 11px; font-weight: 700; }
  .pill.on  { background: #1a3a1a; color: var(--green); }
  .pill.off { background: #2a1a1a; color: var(--muted); }
  .bar-bg { height: 5px; background: var(--border); border-radius: 3px; overflow: hidden; margin-top: 10px; }
  .bar-fill { height: 100%; width: 0; background: var(--blue); border-radius: 3px; transition: width .4s ease; }
  .field-group { display: flex; flex-direction: column; gap: 8px; margin-bottom: 10px; }
  .field-row { display: flex; gap: 8px; }
  .field-row label { flex: 1; display: flex; flex-direction: column; gap: 3px; }
  .field-row label span { font-size: 11px; color: var(--muted); }
  input[type=number], input[type=text] { width: 100%; background: var(--bg); color: var(--text); border: 1px solid var(--border); border-radius: 5px; padding: 6px 8px; font-family: var(--mono); font-size: 13px; outline: none; }
  input:focus { border-color: var(--blue); }
  .btn-row { display: flex; gap: 8px; margin-top: 2px; }
  button { flex: 1; padding: 7px 10px; border-radius: 5px; border: none; font-size: 13px; font-weight: 600; cursor: pointer; transition: opacity .15s; }
  button:hover { opacity: .85; }
  button:disabled { opacity: .4; cursor: not-allowed; }
  #btn-calc   { background: #21262d; color: var(--text); border: 1px solid var(--border); }
  #btn-send   { background: var(--blue); color: #000; }
  #btn-cancel { background: #3a1a1a; color: var(--red); border: 1px solid #5a2a2a; flex: 0 0 auto; padding: 7px 14px; }
  .send-hint  { font-size: 11px; color: var(--muted); margin-top: 6px; min-height: 16px; }
  #last-update { color: var(--muted); font-size: 11px; text-align: right; margin-top: 16px; }
</style>
</head>
<body>
<header>
  <div id="status-dot"></div>
  <h1>Robot nav</h1>
  <span id="status-text">connecting…</span>
</header>
<div class="grid">
  <div class="card">
    <div class="card-title">Position</div>
    <div class="row"><span class="lbl">GPS lat</span>   <span class="val" id="gps-lat">—</span></div>
    <div class="row"><span class="lbl">GPS lon</span>   <span class="val" id="gps-lon">—</span></div>
    <div class="row"><span class="lbl">Est. lat</span>  <span class="val" id="est-lat">—</span></div>
    <div class="row"><span class="lbl">Est. lon</span>  <span class="val" id="est-lon">—</span></div>
  </div>
  <div class="card">
    <div class="card-title">Heading</div>
    <div id="compass-wrap">
      <svg id="compass-svg" width="120" height="120" viewBox="0 0 120 120">
        <circle cx="60" cy="60" r="54" fill="#0d1117" stroke="#30363d" stroke-width="1"/>
        <text x="60" y="13"  text-anchor="middle" fill="#8b949e" font-size="10">N</text>
        <text x="60" y="112" text-anchor="middle" fill="#8b949e" font-size="10">S</text>
        <text x="8"  y="64"  text-anchor="middle" fill="#8b949e" font-size="10">W</text>
        <text x="112" y="64" text-anchor="middle" fill="#8b949e" font-size="10">E</text>
        <g stroke="#30363d" stroke-width="1">
          <line x1="60" y1="8"  x2="60" y2="16"/>
          <line x1="60" y1="104" x2="60" y2="112"/>
          <line x1="8"  y1="60" x2="16" y2="60"/>
          <line x1="104" y1="60" x2="112" y2="60"/>
        </g>
        <g id="needle-target" transform="rotate(0 60 60)">
          <line x1="60" y1="18" x2="60" y2="54" stroke="#d29922" stroke-width="1.5" stroke-dasharray="4 3" opacity="0.7"/>
        </g>
        <g id="needle-fused" transform="rotate(0 60 60)">
          <polygon points="60,16 56,60 64,60" fill="#58a6ff" opacity="0.9"/>
          <polygon points="60,104 56,60 64,60" fill="#1a3050"/>
        </g>
        <circle cx="60" cy="60" r="4" fill="#0d1117" stroke="#30363d" stroke-width="1"/>
      </svg>
    </div>
    <div class="row"><span class="lbl">Fused heading</span>  <span class="val" id="h-fused">—</span></div>
    <div class="row"><span class="lbl">Compass</span>        <span class="val" id="h-comp">—</span></div>
    <div class="row"><span class="lbl">Odometry</span>       <span class="val" id="h-odom">—</span></div>
    <div class="row"><span class="lbl">Target bearing</span> <span class="val" id="h-target">—</span></div>
  </div>
  <div class="card">
    <div class="card-title">Active goal</div>
    <div class="row">
      <span class="lbl">Status</span>
      <span class="val"><span class="pill off" id="goal-pill">no goal</span></span>
    </div>
    <div class="row"><span class="lbl">Goal lat</span>   <span class="val" id="goal-lat">—</span></div>
    <div class="row"><span class="lbl">Goal lon</span>   <span class="val" id="goal-lon">—</span></div>
    <div class="row"><span class="lbl">Distance</span>   <span class="val" id="goal-dist">—</span></div>
    <div class="row"><span class="lbl">Bearing</span>    <span class="val" id="goal-bearing">—</span></div>
    <div class="bar-bg"><div class="bar-fill" id="dist-bar"></div></div>
  </div>
  <div class="card">
    <div class="card-title">Send goal</div>
    <div style="font-size:11px;color:var(--muted);margin-bottom:6px;">Offset from current position</div>
    <div class="field-group">
      <div class="field-row">
        <label><span>North (m)</span><input type="number" id="inp-north" value="0" step="0.5"></label>
        <label><span>East (m)</span> <input type="number" id="inp-east"  value="0" step="0.5"></label>
      </div>
      <button id="btn-calc">Calculate target position</button>
    </div>
    <div style="border-top:1px solid var(--border);margin:10px 0 10px;"></div>
    <div style="font-size:11px;color:var(--muted);margin-bottom:6px;">Target coordinates</div>
    <div class="field-group">
      <div class="field-row">
        <label><span>Latitude</span>  <input type="number" id="inp-lat" step="0.000001" placeholder="—"></label>
        <label><span>Longitude</span> <input type="number" id="inp-lon" step="0.000001" placeholder="—"></label>
      </div>
    </div>
    <div class="btn-row">
      <button id="btn-send" disabled>Send goal</button>
      <button id="btn-cancel">Cancel goal</button>
    </div>
    <div class="send-hint" id="send-hint"></div>
  </div>
</div>
<div id="last-update">never updated</div>

<script>
const WS_URL = `ws://${location.hostname}:__WS_PORT__`;
const dot   = document.getElementById('status-dot');
const stxt  = document.getElementById('status-text');
const hint  = document.getElementById('send-hint');
const btnSend   = document.getElementById('btn-send');
const btnCancel = document.getElementById('btn-cancel');
const btnCalc   = document.getElementById('btn-calc');
const inpLat    = document.getElementById('inp-lat');
const inpLon    = document.getElementById('inp-lon');
const inpNorth  = document.getElementById('inp-north');
const inpEast   = document.getElementById('inp-east');

const RADIUS         = 6371008.7714;
const CIRCUM         = 2 * Math.PI * RADIUS;
const METERS_PER_DEG = CIRCUM / 360;

function offsetPosition(lat, lon, north, east) {
  const dlat = north / METERS_PER_DEG;
  const dlon = east  / (METERS_PER_DEG * Math.cos(lat * Math.PI / 180));
  return [lat + dlat, lon + dlon];
}

let ws;
let lastEstLat = null, lastEstLon = null;
let maxDist    = null;

function fmt(v, dp = 6) { return (v == null) ? '—' : Number(v).toFixed(dp); }
function rotateEl(id, deg) { document.getElementById(id).setAttribute('transform', `rotate(${deg} 60 60)`); }

function update(d) {
  document.getElementById('gps-lat').textContent = fmt(d.gps.lat);
  document.getElementById('gps-lon').textContent = fmt(d.gps.lon);
  document.getElementById('est-lat').textContent = fmt(d.est_pos.lat);
  document.getElementById('est-lon').textContent = fmt(d.est_pos.lon);
  lastEstLat = d.est_pos.lat;
  lastEstLon = d.est_pos.lon;

  const h = d.heading;
  document.getElementById('h-fused').textContent  = fmt(h.fused,   1) + '°';
  document.getElementById('h-comp').textContent   = fmt(h.compass, 1) + '°';
  document.getElementById('h-odom').textContent   = fmt(h.odom,    1) + '°';
  rotateEl('needle-fused', h.fused);

  const g    = d.goal;
  const pill = document.getElementById('goal-pill');
  pill.textContent = g.has_goal ? 'active' : 'no goal';
  pill.className   = 'pill ' + (g.has_goal ? 'on' : 'off');

  document.getElementById('goal-lat').textContent  = fmt(g.lat);
  document.getElementById('goal-lon').textContent  = fmt(g.lon);
  document.getElementById('goal-dist').textContent = g.distance_m != null ? g.distance_m + ' m' : '—';

  const bearing = g.bearing;
  document.getElementById('goal-bearing').textContent = bearing != null ? bearing + '°' : '—';
  document.getElementById('h-target').textContent     = bearing != null ? bearing + '°' : '—';

  if (bearing != null) rotateEl('needle-target', bearing);

  if (g.has_goal && g.distance_m != null) {
    if (maxDist == null || g.distance_m > maxDist) maxDist = g.distance_m;
    const pct = maxDist > 0 ? (1 - g.distance_m / maxDist) * 100 : 100;
    document.getElementById('dist-bar').style.width = pct + '%';
  } else {
    if (!g.has_goal) maxDist = null;
    document.getElementById('dist-bar').style.width = '0%';
  }
  document.getElementById('last-update').textContent = 'last update: ' + new Date().toLocaleTimeString();
}

function checkInputs() {
  const lat = parseFloat(inpLat.value);
  const lon = parseFloat(inpLon.value);
  btnSend.disabled = !(isFinite(lat) && isFinite(lon));
}
inpLat.addEventListener('input', checkInputs);
inpLon.addEventListener('input', checkInputs);

btnCalc.addEventListener('click', () => {
  if (lastEstLat == null) { hint.textContent = 'No position fix yet.'; return; }
  const north = parseFloat(inpNorth.value) || 0;
  const east  = parseFloat(inpEast.value)  || 0;
  const [lat, lon] = offsetPosition(lastEstLat, lastEstLon, north, east);
  inpLat.value = lat.toFixed(8);
  inpLon.value = lon.toFixed(8);
  hint.textContent = `Calculated from (${lastEstLat.toFixed(6)}, ${lastEstLon.toFixed(6)})`;
  checkInputs();
});

btnSend.addEventListener('click', () => {
  const lat = parseFloat(inpLat.value);
  const lon = parseFloat(inpLon.value);
  if (!isFinite(lat) || !isFinite(lon)) return;
  if (!ws || ws.readyState !== WebSocket.OPEN) { hint.textContent = 'Not connected.'; return; }
  ws.send(JSON.stringify({ type: 'set_goal', lat, lon }));
  hint.textContent = `Goal sent: (${lat.toFixed(6)}, ${lon.toFixed(6)})`;
  maxDist = null;
});

btnCancel.addEventListener('click', () => {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(JSON.stringify({ type: 'cancel_goal' }));
  hint.textContent = 'Goal cancelled.';
  maxDist = null;
});

function connect() {
  ws = new WebSocket(WS_URL);
  ws.onopen = () => { dot.className = 'ok'; stxt.textContent = 'connected'; };
  ws.onmessage = (ev) => { try { update(JSON.parse(ev.data)); } catch(e) { console.error(e); } };
  ws.onclose = () => { dot.className = ''; stxt.textContent = 'reconnecting…'; setTimeout(connect, 2000); };
  ws.onerror = () => ws.close();
}
connect();
</script>
</body>
</html>
"""

class DashboardProcess(multiprocessing.Process):
    def __init__(self, shm_name: str,
                 http_port: int = DASHBOARD_HTTP_PORT,
                 ws_port:   int = DASHBOARD_WS_PORT):
        super().__init__(name="DashboardProcess", daemon=True)
        self._shm_name  = shm_name
        self._http_port = http_port
        self._ws_port   = ws_port

    def run(self):
        import websockets
        import websockets.server
        from http.server import BaseHTTPRequestHandler, HTTPServer

        # Connect to existing buffer initialized by the sensor node
        shm = SharedMemory(name=self._shm_name, create=False, size=SHM_SIZE)

        def request_goal_via_shm(lat: float, lon: float):
            """Writes the command payload directly to memory for sensor node retrieval."""
            f = list(struct.unpack_from(SHM_FORMAT, shm.buf))
            f[14] = lat    # cmd_lat
            f[15] = lon    # cmd_lon
            f[16] = True   # cmd_trigger_flag
            shm.buf[:SHM_SIZE] = struct.pack(SHM_FORMAT, *f)

        def cancel_goal():
            f = list(struct.unpack_from(SHM_FORMAT, shm.buf))
            f[13] = False  # has_goal
            shm.buf[:SHM_SIZE] = struct.pack(SHM_FORMAT, *f)

        # ── HTTP Server Setup ─────────────────────────────────────────────
        html = DASHBOARD_HTML.replace('__WS_PORT__', str(self._ws_port))

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.end_headers()
                self.wfile.write(html.encode())
            def log_message(self, *_):
                pass

        http = HTTPServer(('0.0.0.0', self._http_port), Handler)
        threading.Thread(target=http.serve_forever, daemon=True).start()
        print(f"[dashboard] http://localhost:{self._http_port}/", flush=True)

        # ── WebSocket Server Setup ────────────────────────────────────────
        clients: set = set()

        async def ws_handler(ws):
            clients.add(ws)
            try:
                async for raw in ws:
                    try:
                        msg = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    kind = msg.get('type')
                    if kind == 'set_goal':
                        lat = float(msg['lat'])
                        lon = float(msg['lon'])
                        request_goal_via_shm(lat, lon)
                        print(f"[dashboard] command sent to SHM buffer → ({lat}, {lon})", flush=True)
                    elif kind == 'cancel_goal':
                        cancel_goal()
                        print("[dashboard] goal cancellation updated in SHM", flush=True)
            finally:
                clients.discard(ws)

        async def broadcast_loop():
            interval = 1.0 / DASHBOARD_HZ
            while True:
                if clients:
                    payload = json.dumps(shm_to_dict(shm))
                    await asyncio.gather(
                        *[c.send(payload) for c in list(clients)],
                        return_exceptions=True,
                    )
                await asyncio.sleep(interval)

        async def serve():
            async with websockets.server.serve(ws_handler, '0.0.0.0', self._ws_port):
                print(f"[dashboard] ws://localhost:{self._ws_port}/", flush=True)
                await broadcast_loop()

        try:
            asyncio.run(serve())
        except KeyboardInterrupt:
            pass
        finally:
            http.shutdown()
            shm.close()


def main():
    dashboard = DashboardProcess(SHM_NAME)
    dashboard.start()
    
    print("[Dashboard Node] Running dashboard pipeline. Assumes Sensor Node has allocated SHM.")
    try:
        dashboard.join()
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.terminate()
        dashboard.join()


if __name__ == '__main__':
    multiprocessing.set_start_method('spawn')
    main()
