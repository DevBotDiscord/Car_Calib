"""Embedded HTTP server for Jetson Nano dashboard.

Serves:
  /                   → dashboard HTML
  /stream             → MJPEG live stream
  /api/status         → JSON telemetry
  /api/base/<cmd>     → base motor command
  /api/relay/<state>  → relay ON/OFF
  /api/power/<state>  → power ON/OFF
"""

from __future__ import annotations

import json
import logging
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from typing import Any, Callable

import cv2
import numpy as np

logger = logging.getLogger(__name__)

DASHBOARD_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>car-calib Jetson Nano</title>
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
body{font:13px/1.4 system-ui,sans-serif;background:#0d1117;color:#c9d1d9;display:flex;flex-direction:column;height:100vh;overflow:hidden}
header{display:flex;align-items:center;gap:12px;padding:8px 16px;background:#161b22;border-bottom:1px solid #30363d}
header .brand{font-weight:700;font-size:15px;color:#58a6ff}
.pills{display:flex;gap:8px;font-size:12px}
.pill{padding:3px 10px;border-radius:12px;border:1px solid #30363d}
.pill.ok{background:#0d3321;border-color:#1a7f4b;color:#3fb950}
.pill.bad{background:#3d1118;border-color:#8b2c3d;color:#f85149}
.pill.warn{background:#332b0d;border-color:#7a601a;color:#d29922}
main{display:flex;flex:1;gap:12px;padding:12px;overflow:hidden}
.panel{display:flex;flex-direction:column;gap:10px;min-width:0}
.panel.stream{flex:1;min-width:300px}
.panel.ctrl{width:340px;flex-shrink:0}
.stream-wrap{position:relative;flex:1;min-height:200px;background:#161b22;border:1px solid #30363d;border-radius:8px;overflow:hidden}
.stream-wrap img{width:100%;height:100%;object-fit:contain}
.overlay-top{position:absolute;top:8px;left:8px;font-size:11px;background:rgba(0,0,0,.6);padding:4px 8px;border-radius:4px}
.overlay-bottom{position:absolute;bottom:8px;left:8px;right:8px;display:flex;gap:8px;justify-content:space-between;font-size:11px}
.overlay-bottom span{background:rgba(0,0,0,.6);padding:4px 8px;border-radius:4px}
.metrics{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.metric{background:#161b22;border:1px solid #30363d;border-radius:6px;padding:8px 10px}
.metric .label{font-size:10px;color:#8b949e;text-transform:uppercase;letter-spacing:.5px}
.metric .value{font-size:16px;font-weight:600;margin-top:2px}
.metric .value.ok{color:#3fb950}
.metric .value.bad{color:#f85149}
.metric .value.warn{color:#d29922}
.btn{display:inline-flex;align-items:center;gap:4px;padding:8px 14px;border:1px solid #30363d;border-radius:6px;background:#21262d;color:#c9d1d9;font-size:12px;cursor:pointer;transition:background .15s}
.btn:hover{background:#30363d}
.btn.primary{background:#1f6feb;border-color:#1f6feb;color:#fff}
.btn.primary:hover{background:#388bfd}
.btn.danger{background:#da3633;border-color:#da3633;color:#fff}
.btn.danger:hover{background:#f85149}
.btn.on{background:#1a7f4b;border-color:#1a7f4b;color:#fff}
.btn-row{display:flex;gap:6px;flex-wrap:wrap}
h3{font-size:12px;color:#8b949e;text-transform:uppercase;letter-spacing:.5px;margin-bottom:4px}
.health{font-size:12px;padding:6px 12px;border-radius:4px;display:none}
.health.show{display:block}
.health.ok{background:#0d3321;color:#3fb950}
.health.warn{background:#332b0d;color:#d29922}
.health.bad{background:#3d1118;color:#f85149}
</style>
</head>
<body>
<header>
<div class="brand">car-calib Jetson</div>
<div class="pills">
<span id="pFsm" class="pill">GAPPING</span>
<span id="pBase" class="pill">STOP</span>
<span id="pRelay" class="pill">relay</span>
</div>
</header>
<div id="health" class="health"></div>
<main>
<div class="panel stream">
<div class="stream-wrap">
<img id="stream" alt="camera">
<div class="overlay-top" id="overlayTheta"></div>
<div class="overlay-bottom">
<span id="overlayAngle"></span>
<span id="overlayMs"></span>
</div>
</div>
</div>
<div class="panel ctrl">
<h3>Controls</h3>
<div class="btn-row">
<button class="btn primary" onclick="api('base','FORWARD')">▶ Forward</button>
<button class="btn danger" onclick="api('base','STOP')">■ Stop</button>
<button class="btn" onclick="api('base','BACKWARD')">◀ Backward</button>
</div>
<div class="btn-row">
<button class="btn" id="btnLeft" onclick="api('base','TURN_LEFT')">↰ Left</button>
<button class="btn" id="btnRight" onclick="api('base','TURN_RIGHT')">↱ Right</button>
</div>
<div class="btn-row" style="margin-top:4px">
<button class="btn" id="btnRelay" onclick="toggleRelay()">💡 Relay OFF</button>
<button class="btn primary" id="btnPowerOn" onclick="api('power','ON')">⏻ Bật xe</button>
<button class="btn danger" id="btnPowerOff" onclick="api('power','OFF')">⭘ Tắt xe</button>
</div>
<h3 style="margin-top:12px">Telemetry</h3>
<div class="metrics" id="metrics"></div>
</div>
</main>
<script>
let lastRelay=false, lastPower=false;
const stream=document.getElementById('stream');
stream.src='/stream';
setInterval(async()=>{
try{const r=await fetch('/api/status');if(!r.ok)return;
const d=await r.json();
document.getElementById('pFsm').textContent=d.fsm||'?';
document.getElementById('pFsm').className='pill '+(d.calib_active?'ok':(d.fsm==='GAPPING'?'bad':'warn'));
document.getElementById('pBase').textContent=d.base||'STOP';
document.getElementById('overlayTheta').textContent='θ:'+(d.theta??'--')+'° src:'+(d.theta_src||'?');
document.getElementById('overlayAngle').textContent='servo:'+parseFloat(d.servo).toFixed(1)+'°';
document.getElementById('overlayMs').textContent='loop:'+parseFloat(d.loop_ms).toFixed(0)+'ms';
document.getElementById('btnRelay').textContent=d.relay_on?'💡 Relay ON':'💡 Relay OFF';
document.getElementById('btnRelay').className='btn'+(d.relay_on?' on':'');
lastRelay=d.relay_on;
const m=document.getElementById('metrics');
m.innerHTML='';
const items=[
['FSM',d.fsm,(d.fsm==='TRACKING_PD'?'ok':(d.fsm==='GAPPING'?'bad':'warn'))],
['Theta',d.theta!=null?parseFloat(d.theta).toFixed(2)+'°':'--',''],
['Servo',parseFloat(d.servo).toFixed(1)+'°',''],
['Loop',parseFloat(d.loop_ms).toFixed(0)+'ms',''],
['Base',d.base||'STOP',''],
['Relay',d.relay_on?'ON':'OFF',d.relay_on?'ok':''],
['Power',d.power_pulsing?'pulsing':'idle',d.power_pulsing?'warn':''],
['Frames',d.frame,''],
];
for(const[l,v,c]of items){
m.innerHTML+='<div class="metric"><div class="label">'+l+'</div><div class="value'+(c?' '+c:'')+'">'+v+'</div></div>';
}
}catch(e){}
},300);
function api(cat,cmd){
fetch('/api/'+cat+'/'+cmd).then(r=>r.text()).then(console.log).catch(console.error);
}
function toggleRelay(){
fetch('/api/relay/'+(lastRelay?'OFF':'ON')).then(r=>r.text()).then(console.log);
}
</script>
</body>
</html>"""


class _RequestHandler(BaseHTTPRequestHandler):
    """Minimal request handler with CORS support."""

    def log_message(self, fmt: str, *args: Any) -> None:
        logger.debug("HTTP %s", fmt % args)

    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")

    def _text(self, code: int, body: str) -> None:
        self.send_response(code)
        self._cors()
        self.send_header("Content-Type", "text/plain")
        self.end_headers()
        self.wfile.write(body.encode())

    def _json(self, data: dict[str, Any]) -> None:
        body = json.dumps(data, default=str)
        self.send_response(200)
        self._cors()
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(body.encode())

    def do_GET(self) -> None:
        # ---- Dashboard HTML ----
        if self.path == "/" or self.path == "/dashboard":
            self.send_response(200)
            self._cors()
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(DASHBOARD_HTML.encode())
            return

        # ---- MJPEG stream ----
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            getter = self.server.frame_getter
            while True:
                frame = getter() if callable(getter) else None
                if frame is None:
                    time.sleep(0.03)
                    continue
                _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                self.wfile.write(jpeg.tobytes())
                self.wfile.write(b"\r\n")
            return

        # ---- API /status ----
        if self.path == "/api/status":
            getter = getattr(self.server, "status_getter", None)
            data = getter() if callable(getter) else {}
            self._json(data)
            return

        # ---- API /base/<cmd> ----
        if self.path.startswith("/api/base/"):
            cmd = self.path.split("/api/base/")[-1]
            handler = getattr(self.server, "base_handler", None)
            if callable(handler):
                handler(cmd)
            self._text(200, f"base:{cmd}")
            return

        # ---- API /relay/<state> ----
        if self.path.startswith("/api/relay/"):
            state = self.path.split("/api/relay/")[-1].upper()
            handler = getattr(self.server, "relay_handler", None)
            if callable(handler):
                handler(state)
            self._text(200, f"relay:{state}")
            return

        # ---- API /power/<state> ----
        if self.path.startswith("/api/power/"):
            state = self.path.split("/api/power/")[-1].upper()
            handler = getattr(self.server, "power_handler", None)
            if callable(handler):
                handler(state)
            self._text(200, f"power:{state}")
            return

        self._text(404, "not found")


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """HTTP server with threading support."""
    allow_reuse_address = True
    daemon_threads = True


class JetsonHttpServer:
    """Embedded HTTP server for the Jetson Nano dashboard."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 8080,
    ) -> None:
        self._host = host
        self._port = port
        self._server: ThreadedHTTPServer | None = None
        self._frame_getter: Callable[[], np.ndarray | None] | None = None
        self._status_getter: Callable[[], dict[str, Any]] | None = None
        self._base_handler: Callable[[str], None] | None = None
        self._relay_handler: Callable[[str], None] | None = None
        self._power_handler: Callable[[str], None] | None = None
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def set_frame_getter(self, fn: Callable[[], np.ndarray | None]) -> None:
        self._frame_getter = fn

    def set_status_getter(self, fn: Callable[[], dict[str, Any]]) -> None:
        self._status_getter = fn

    def set_base_handler(self, fn: Callable[[str], None]) -> None:
        self._base_handler = fn

    def set_relay_handler(self, fn: Callable[[str], None]) -> None:
        self._relay_handler = fn

    def set_power_handler(self, fn: Callable[[str], None]) -> None:
        self._power_handler = fn

    def start(self) -> None:
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        logger.info("Dashboard: http://%s:%d", self._host, self._port)

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
        logger.info("Dashboard: stopped")

    # ------------------------------------------------------------------ #
    # Internal
    # ------------------------------------------------------------------ #

    def _serve(self) -> None:
        self._server = ThreadedHTTPServer((self._host, self._port), _RequestHandler)
        self._server.frame_getter = self._frame_getter
        self._server.status_getter = self._status_getter
        self._server.base_handler = self._base_handler
        self._server.relay_handler = self._relay_handler
        self._server.power_handler = self._power_handler
        self._server.serve_forever()
