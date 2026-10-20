"""Phone web joystick core without ROS (standard library only).

Serves a touch joystick page. Safety mirrors the gamepad deadman:

- the page only sends commands while a finger or the mouse is held on the pad,
- a watchdog zeroes the command when no command arrived for watchdog_s
  seconds (covers a WiFi drop, a locked phone or a closed tab),
- E-STOP latches zero until it is reset from the page, and a command that
  was in flight before the reset is not resumed.

The ROS node only reads WebState.current() at a fixed rate and publishes it.
"""
from __future__ import annotations

import hmac
import json
import math
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit


@dataclass
class WebConfig:
    max_linear: float = 0.3      # m/s at full deflection
    max_angular: float = 0.6     # rad/s at full deflection
    watchdog_s: float = 0.4      # zero when no command arrived for this long
    deadzone: float = 0.12       # applied in the page
    token: str = ""              # empty = no token required


class WebState:
    def __init__(self, cfg: WebConfig | None = None, clock=time.monotonic):
        self.cfg = cfg or WebConfig()
        self.clock = clock
        self.lock = threading.Lock()
        self.v = 0.0
        self.w = 0.0
        self.stamp = None         # clock time of the last accepted command
        self.estop = False
        self.battery_v = None
        self.clients = {}         # address -> last seen

    def set_cmd(self, v, w, addr="?"):
        """v and w are -1..1 from the page; scaled to the configured maxima.

        A value that is not a finite number (NaN or infinity) stops the robot."""
        now = self.clock()
        v, w = float(v), float(w)
        with self.lock:
            if self.estop or not (math.isfinite(v) and math.isfinite(w)):
                self.v = self.w = 0.0
            else:
                self.v = max(-1.0, min(1.0, float(v))) * self.cfg.max_linear
                self.w = max(-1.0, min(1.0, float(w))) * self.cfg.max_angular
            self.stamp = now
            self.clients[addr] = now

    def set_estop(self, on):
        with self.lock:
            self.estop = bool(on)
            self.v = self.w = 0.0

    def _age(self, now):
        return None if self.stamp is None else now - self.stamp

    def current(self):
        """(linear_x, angular_z, live) to publish now, after the e-stop and the watchdog."""
        now = self.clock()
        with self.lock:
            age = self._age(now)
            if self.estop or age is None or age > self.cfg.watchdog_s:
                return 0.0, 0.0, False
            return self.v, self.w, True

    def snapshot(self):
        now = self.clock()
        with self.lock:
            age = self._age(now)
            live = (not self.estop) and age is not None and age <= self.cfg.watchdog_s
            return {
                "v": round(self.v, 3),
                "w": round(self.w, 3),
                "live": live,
                "estop": self.estop,
                "age_ms": None if age is None else int(age * 1000),
                "battery_v": self.battery_v,
                "lin_max": self.cfg.max_linear,
                "ang_max": self.cfg.max_angular,
                "clients": len(self.clients),
            }


PAGE = r"""<!doctype html><html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>teleop</title>
<style>
:root{--bg:#111417;--fg:#e8edf2;--dim:#8b97a3;--accent:#3ea6ff;--warn:#ff5c5c;--ok:#3ddc84}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);
  font:15px/1.4 -apple-system,Roboto,"Helvetica Neue",sans-serif;
  overscroll-behavior:none;touch-action:none;user-select:none}
.wrap{display:flex;flex-direction:column;align-items:center;gap:14px;padding:14px;height:100%}
.bar{display:flex;gap:14px;align-items:center;font-variant-numeric:tabular-nums;
  font-size:13px;color:var(--dim);width:100%;max-width:380px;justify-content:space-between}
.dot{width:9px;height:9px;border-radius:50%;background:#555;display:inline-block;margin-right:6px}
.dot.on{background:var(--ok);box-shadow:0 0 8px var(--ok)}
.dot.stop{background:var(--warn);box-shadow:0 0 8px var(--warn)}
#pad{position:relative;width:min(86vw,360px);height:min(86vw,360px);border-radius:50%;
  background:radial-gradient(circle at 50% 50%,#1b2026 0%,#15191e 70%,#12161a 100%);
  border:1px solid #262c33;touch-action:none;flex:none}
#pad::before,#pad::after{content:"";position:absolute;background:#22282f}
#pad::before{left:50%;top:8%;bottom:8%;width:1px;transform:translateX(-50%)}
#pad::after{top:50%;left:8%;right:8%;height:1px;transform:translateY(-50%)}
#knob{position:absolute;width:26%;height:26%;border-radius:50%;left:37%;top:37%;
  background:var(--accent);opacity:.55;transition:opacity .12s;pointer-events:none}
#pad.active #knob{opacity:1}
.hint{color:var(--dim);font-size:13px;text-align:center;min-height:1.4em}
.readout{font-variant-numeric:tabular-nums;font-size:19px;letter-spacing:.5px}
button{font:inherit;border:0;border-radius:10px;padding:14px 18px;color:#fff;background:#2a3138}
#estop{background:var(--warn);font-weight:700;letter-spacing:1px;width:min(86vw,360px);
  padding:18px;font-size:17px}
#estop.armed{background:#2a3138;color:var(--warn);border:1px solid var(--warn)}
</style></head><body>
<div class="wrap">
  <div class="bar">
    <span><span class="dot" id="dot"></span><span id="state">idle</span></span>
    <span id="batt">battery --</span>
  </div>
  <div class="readout" id="vals">v 0.00 m/s &nbsp; w 0.00 rad/s</div>
  <div id="pad"><div id="knob"></div></div>
  <div class="hint" id="hint">hold and drag, release = stop</div>
  <button id="estop">E-STOP</button>
</div>
<script>
var TOKEN = new URLSearchParams(location.search).get('t') || '';
var pad = document.getElementById('pad'), knob = document.getElementById('knob');
var vals = document.getElementById('vals'), hint = document.getElementById('hint');
var dot = document.getElementById('dot'), stateEl = document.getElementById('state');
var battEl = document.getElementById('batt'), estopBtn = document.getElementById('estop');
var DEAD = %DEADZONE%, active = false, vx = 0, wz = 0, estop = false, keys = {};

function q() { return TOKEN ? '?t=' + encodeURIComponent(TOKEN) : ''; }
function post(path, body) {
  return fetch(path + q(), {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(body || {}), keepalive: true
  }).then(function (r) { return r.json(); }).catch(function () { return null; });
}
function setKnob(nx, ny) {
  knob.style.left = (37 + nx * 30) + '%';
  knob.style.top = (37 - ny * 30) + '%';
}
function fromPointer(e) {
  var r = pad.getBoundingClientRect(), R = r.width / 2;
  var nx = (e.clientX - r.left - R) / R, ny = -(e.clientY - r.top - R) / R;
  var m = Math.sqrt(nx * nx + ny * ny);
  if (m > 1) { nx /= m; ny /= m; }
  wz = Math.abs(nx) > DEAD ? -nx : 0;   // right = clockwise = negative yaw
  vx = Math.abs(ny) > DEAD ? ny : 0;
  setKnob(nx, ny);
}
function release() {
  if (!active) return;
  active = false; vx = 0; wz = 0; pad.classList.remove('active');
  setKnob(0, 0); post('/cmd', {v: 0, w: 0}); render();
}
pad.addEventListener('pointerdown', function (e) {
  active = true; pad.classList.add('active'); pad.setPointerCapture(e.pointerId);
  fromPointer(e); e.preventDefault();
});
pad.addEventListener('pointermove', function (e) { if (active) { fromPointer(e); e.preventDefault(); } });
['pointerup', 'pointercancel', 'pointerleave'].forEach(function (ev) {
  pad.addEventListener(ev, release);
});
window.addEventListener('blur', release);
document.addEventListener('visibilitychange', function () { if (document.hidden) release(); });

var KEYMAP = {ArrowUp: [1, 0], KeyW: [1, 0], ArrowDown: [-1, 0], KeyS: [-1, 0],
              ArrowLeft: [0, 1], KeyA: [0, 1], ArrowRight: [0, -1], KeyD: [0, -1]};
function keyUpdate() {
  var v = 0, w = 0, any = false;
  for (var k in keys) if (keys[k] && KEYMAP[k]) { v += KEYMAP[k][0]; w += KEYMAP[k][1]; any = true; }
  if (any) {
    active = true; pad.classList.add('active');
    vx = Math.max(-1, Math.min(1, v)); wz = Math.max(-1, Math.min(1, w));
    setKnob(-wz, vx);
  } else if (active) { release(); }
}
window.addEventListener('keydown', function (e) {
  if (e.code === 'Space') { toggleEstop(); e.preventDefault(); return; }
  if (!KEYMAP[e.code]) return;
  keys[e.code] = 1; keyUpdate(); e.preventDefault();
});
window.addEventListener('keyup', function (e) { keys[e.code] = 0; keyUpdate(); });

function toggleEstop() { estop = !estop; post('/estop', {on: estop}).then(poll); }
estopBtn.addEventListener('click', toggleEstop);

function render() {
  vals.textContent = 'v ' + (vx * %LIN%).toFixed(2) + ' m/s   w ' + (wz * %ANG%).toFixed(2) + ' rad/s';
}
// command loop: only sends while held, so release, lock or a dropped link = watchdog stop
setInterval(function () { if (active && !estop) { post('/cmd', {v: vx, w: wz}); render(); } }, 66);

function poll() {
  fetch('/status' + q())
    .then(function (r) { return r.json(); }).then(function (s) {
      estop = s.estop;
      estopBtn.textContent = estop ? 'E-STOP LATCHED, TAP TO RESET' : 'E-STOP';
      estopBtn.classList.toggle('armed', estop);
      dot.className = 'dot' + (s.estop ? ' stop' : (s.live ? ' on' : ''));
      stateEl.textContent = s.estop ? 'stopped' : (s.live ? 'driving' : 'idle');
      battEl.textContent = s.battery_v ? 'battery ' + s.battery_v.toFixed(1) + ' V' : 'battery --';
      hint.textContent = s.clients > 1 ? (s.clients + ' clients connected, last one wins')
                                       : 'hold and drag, release = stop';
    }).catch(function () { stateEl.textContent = 'no link'; dot.className = 'dot stop'; });
}
setInterval(poll, 1000); poll(); render();
</script></body></html>
"""


def render_page(cfg: WebConfig) -> str:
    return (PAGE.replace("%DEADZONE%", repr(cfg.deadzone))
                .replace("%LIN%", repr(cfg.max_linear))
                .replace("%ANG%", repr(cfg.max_angular)))


def make_handler(state: WebState, log=None):
    page = render_page(state.cfg).encode("utf-8")
    token = state.cfg.token

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "failsafe-teleop/1.0"

        def log_message(self, fmt, *args):
            # /cmd arrives about 15 times a second; log only the rest
            if log is not None and "/cmd" not in self.path:
                log("%s %s" % (self.client_address[0], fmt % args))

        def _authed(self):
            if not token:
                return True
            given = parse_qs(urlsplit(self.path).query).get("t", [""])[0] or self.headers.get("X-Token", "")
            return hmac.compare_digest(given.encode(), token.encode())

        def _send(self, code, body, ctype):
            if isinstance(body, str):
                body = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj), "application/json")

        def do_GET(self):
            path = urlsplit(self.path).path
            if not self._authed():
                return self._send(401, "unauthorized\n", "text/plain")
            if path in ("/", "/index.html"):
                return self._send(200, page, "text/html; charset=utf-8")
            if path == "/status":
                return self._json(state.snapshot())
            self._send(404, "not found\n", "text/plain")

        def do_POST(self):
            path = urlsplit(self.path).path
            if not self._authed():
                return self._send(401, "unauthorized\n", "text/plain")
            try:
                n = int(self.headers.get("Content-Length", 0))
                data = json.loads(self.rfile.read(n) or b"{}")
                if not isinstance(data, dict):
                    raise ValueError("not an object")
            except ValueError:
                return self._json({"error": "bad json"}, 400)
            addr = self.client_address[0]
            try:
                if path == "/cmd":
                    state.set_cmd(float(data.get("v", 0.0)), float(data.get("w", 0.0)), addr)
                    return self._json(state.snapshot())
                if path == "/stop":
                    state.set_cmd(0.0, 0.0, addr)
                    return self._json(state.snapshot())
                if path == "/estop":
                    state.set_estop(data.get("on", True))
                    return self._json(state.snapshot())
            except (TypeError, ValueError):
                return self._json({"error": "bad value"}, 400)
            self._json({"error": "not found"}, 404)

    return Handler


def make_server(state: WebState, host="0.0.0.0", port=8080, log=None) -> ThreadingHTTPServer:
    srv = ThreadingHTTPServer((host, port), make_handler(state, log))
    srv.daemon_threads = True
    return srv
