"""Telemetry dashboard server: UDP receiver + CSV logger + HTTP/SSE UI + TCP command link.

Run from dashboard/:  python3 -m gsdash [--phone IP] [--bind 0.0.0.0] ...
Standard library only.
"""
from __future__ import annotations

import argparse
import collections
import csv
import json
import mimetypes
import os
import re
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from . import protocol as P

STATIC_DIR = Path(__file__).resolve().parent / "static"
DEFAULT_LOG_DIR = Path(__file__).resolve().parent.parent / "logs"


def log(*a):
    print(time.strftime("%H:%M:%S"), "[gsdash]", *a, file=sys.stderr, flush=True)


# --------------------------------------------------------------------------- telemetry

class Telemetry:
    """Shared state: recent frames ring buffer (with monotonically increasing index),
    link statistics, phone source address."""

    RING = 4000   # 80 s at 50 Hz: enough to backfill a reloaded page

    def __init__(self):
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.frames = collections.deque(maxlen=self.RING)  # (idx, frame)
        self.next_idx = 0
        self.seq = P.SeqTracker()
        self.crc_errors = 0
        self.decode_errors = 0
        self.last_error = ""
        self.last_recv = None          # time.time() of last good frame
        self.source_ip = None
        self.source_port = None
        self.recv_times = collections.deque(maxlen=200)
        self.last_frame = None
        self.started = time.time()

    def add_frame(self, frame: dict, addr):
        now = time.time()
        frame["recv_time"] = now
        with self.cond:
            frame["seq_event"] = self.seq.update(frame["seq"], frame["t"])
            self.frames.append((self.next_idx, frame))
            self.next_idx += 1
            self.last_recv = now
            self.recv_times.append(now)
            self.source_ip, self.source_port = addr[0], addr[1]
            self.last_frame = frame
            self.cond.notify_all()

    def add_error(self, err: P.FrameError):
        with self.lock:
            if err.kind == "crc":
                self.crc_errors += 1
            else:
                self.decode_errors += 1
            self.last_error = str(err)

    def frames_since(self, idx: int):
        with self.lock:
            if not self.frames:
                return [], idx
            first = self.frames[0][0]
            if idx < first:
                idx = first
            out = [f for i, f in self.frames if i >= idx]
            return out, self.next_idx

    def rate_hz(self, now=None) -> float:
        now = now or time.time()
        with self.lock:
            ts = [t for t in self.recv_times if now - t <= 2.0]
        if len(ts) < 2:
            return 0.0
        span = now - ts[0]
        return (len(ts) - 1) / span if span > 0 else 0.0

    def stats(self) -> dict:
        now = time.time()
        rate = self.rate_hz(now)
        with self.lock:
            s = self.seq
            age = (now - self.last_recv) if self.last_recv else None
            return {
                "rate_hz": round(rate, 1),
                "received": s.received,
                "lost": s.lost,
                "out_of_order": s.out_of_order,
                "duplicates": s.duplicates,
                "seq_resets": s.resets,
                "crc_errors": self.crc_errors,
                "decode_errors": self.decode_errors,
                "last_error": self.last_error,
                "last_seen_age": None if age is None else round(age, 3),
                "source_ip": self.source_ip,
                "fmt": self.last_frame["fmt"] if self.last_frame else None,
            }

    def reset_stats(self):
        with self.lock:
            self.seq.reset()
            self.crc_errors = self.decode_errors = 0
            self.last_error = ""


class SessionLogger:
    """Appends every frame to logs/<timestamp>/telemetry.csv; commands to commands.csv.
    The directory is created lazily on the first frame/command."""

    def __init__(self, root: Path | None):
        self.root = root
        self.dir = None
        self.lock = threading.Lock()
        self._tf = self._tw = self._cf = self._cw = None
        self._last_flush = 0.0

    def _ensure_dir(self):
        if self.dir is None:
            self.dir = self.root / time.strftime("%Y-%m-%d_%H-%M-%S")
            self.dir.mkdir(parents=True, exist_ok=True)
            log("logging to", self.dir)

    def frame(self, f: dict):
        if self.root is None:
            return
        with self.lock:
            if self._tw is None:
                self._ensure_dir()
                self._tf = open(self.dir / "telemetry.csv", "w", newline="")
                self._tw = csv.writer(self._tf)
                self._tw.writerow(P.LOG_CSV_FIELDS)
            self._tw.writerow([_fmt(f.get(k)) for k in P.LOG_CSV_FIELDS])
            now = time.time()
            if now - self._last_flush > 1.0:
                self._tf.flush()
                self._last_flush = now

    def command(self, req: dict, reply: dict):
        if self.root is None:
            return
        with self.lock:
            if self._cw is None:
                self._ensure_dir()
                self._cf = open(self.dir / "commands.csv", "w", newline="")
                self._cw = csv.writer(self._cf)
                self._cw.writerow(("recv_time", "request", "reply"))
            self._cw.writerow((f"{time.time():.6f}", json.dumps(req), json.dumps(reply)))
            self._cf.flush()

    def close(self):
        with self.lock:
            for f in (self._tf, self._cf):
                if f:
                    f.close()
            self._tf = self._tw = self._cf = self._cw = None


def _fmt(v):
    if v is None:
        return ""
    if isinstance(v, float):
        return repr(v)
    return str(v)


class UDPReceiver(threading.Thread):
    def __init__(self, telem: Telemetry, logger: SessionLogger, host: str, port: int):
        super().__init__(daemon=True, name="udp-rx")
        self.telem, self.logger = telem, logger
        # iPhone hotspots on IPv6-only carriers give the laptop no 172.20.10.x address, so a
        # wildcard bind listens dual-stack (IPv6 + IPv4-mapped).
        if host in ("", "0.0.0.0", "::"):
            self.sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
            host = "::"
        else:
            self.sock = socket.socket(socket.AF_INET6 if ":" in host else socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind((host, port))
        self.sock.settimeout(0.5)
        self.port = self.sock.getsockname()[1]
        self.running = True

    def run(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                if addr[0].startswith("::ffff:"):
                    addr = (addr[0][7:], addr[1])
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                frame = P.decode_frame(data)
            except P.FrameError as e:
                self.telem.add_error(e)
                continue
            self.telem.add_frame(frame, addr)
            try:
                self.logger.frame(frame)
            except OSError as e:
                log("log write failed:", e)

    def stop(self):
        self.running = False
        self.sock.close()


# --------------------------------------------------------------------------- phone link

def parse_browse(out: str) -> list[str]:
    """Instance names from `dns-sd -B _groundspeed._tcp` output."""
    names = []
    for line in out.splitlines():
        # "18:40:12.123  Add  3  14 local.  _groundspeed._tcp.  iPhone GroundSpeed"
        # (before 10:00 the time has one hour digit and the line starts with a space)
        m = re.match(r"\s*\S+\s+Add\s+\S+\s+\S+\s+\S+\s+_groundspeed\._tcp\.\s+(.+)$", line)
        if m:
            names.append(m.group(1).strip())
    return names


def bonjour_discover(timeout: float = 3.0) -> tuple[str, int] | None:
    """Best effort: dns-sd -B then -L, resolve host. Returns (ip, port) or None."""
    try:
        p = subprocess.Popen(["dns-sd", "-B", "_groundspeed._tcp", "local."],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    except (FileNotFoundError, OSError):
        return None
    try:
        out, _ = p.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        p.kill()
        out, _ = p.communicate()
    for name in parse_browse(out):
        try:
            p = subprocess.Popen(["dns-sd", "-L", name, "_groundspeed._tcp", "local."],
                                 stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            try:
                out, _ = p.communicate(timeout=timeout)
            except subprocess.TimeoutExpired:
                p.kill()
                out, _ = p.communicate()
        except OSError:
            continue
        m = re.search(r"can be reached at (\S+?)\.?:(\d+)", out)
        if not m:
            continue
        host, port = m.group(1), int(m.group(2))
        try:
            infos = socket.getaddrinfo(host, port, socket.AF_UNSPEC, socket.SOCK_STREAM)
        except OSError:
            continue
        # IPv6-only carrier hotspots advertise 172.20.10.1 but give the laptop no IPv4
        # route to it, so return the first address that actually accepts a connection.
        # Link-local addresses are skipped: the phone drops the %scope when it adopts the
        # peer as its UDP destination.
        for family, _, _, _, sa in infos:
            ip = sa[0]
            if ip.startswith(("fe80", "169.254.")):
                continue
            try:
                with socket.socket(family, socket.SOCK_STREAM) as s:
                    s.settimeout(0.7)
                    s.connect(sa)
                return ip, port
            except OSError:
                continue
    return None


class PhoneLink:
    """Persistent newline-JSON TCP connection to the phone's command port.

    Target priority: explicit IP (CLI/UI) > UDP source IP > Bonjour. A keepalive thread
    connects as soon as a target is known (connecting also makes the phone adopt this
    laptop as its UDP destination) and pings every few seconds to detect dead links.
    """

    KEEPALIVE_S = 5.0

    def __init__(self, telem: Telemetry, port: int = 9001, explicit_ip: str | None = None,
                 bonjour: bool = True):
        self.telem = telem
        self.port = port
        self.explicit_ip = explicit_ip or None
        self.bonjour_enabled = bonjour
        self.bonjour_result = None     # (ip, port)
        self._bonjour_running = False
        self._last_bonjour = 0.0
        self._connect_fails = 0
        self.lock = threading.Lock()   # serialises request/reply
        self.sock = None
        self.rfile = None
        self.connected_to = None       # (ip, port)
        self.last_error = ""
        self.last_ok = None
        self.running = True
        self._last_traffic = 0.0
        threading.Thread(target=self._keepalive, daemon=True, name="phone-keepalive").start()

    # A UDP source this old is no longer trusted: the phone may have a new address (hotspot
    # reconnect, app restart) and keeps sending to the old laptop address until reconnected.
    UDP_SOURCE_MAX_AGE_S = 3.0
    # After this many consecutive failed connects to a Bonjour address, discover again.
    MAX_BONJOUR_FAILS = 3

    def target(self) -> tuple[str, int, str] | None:
        if self.explicit_ip:
            return self.explicit_ip, self.port, "manual"
        src, seen = self.telem.source_ip, self.telem.last_recv
        if src and seen is not None and time.time() - seen < self.UDP_SOURCE_MAX_AGE_S:
            return src, self.port, "udp-source"
        if self.bonjour_result:
            return self.bonjour_result[0], self.bonjour_result[1], "bonjour"
        return None

    def set_explicit(self, ip: str | None):
        with self.lock:
            self.explicit_ip = (ip or "").strip() or None
            self._close()

    def _close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = self.rfile = None
        self.connected_to = None

    def _connect(self):
        tgt = self.target()
        if tgt is None:
            raise ConnectionError("phone IP unknown (no telemetry yet, no --phone, "
                                  "Bonjour found nothing)")
        ip, port, _ = tgt
        if self.sock and self.connected_to == (ip, port):
            return
        self._close()
        s = socket.create_connection((ip, port), timeout=2.0)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(3.0)
        self.sock, self.rfile = s, s.makefile("rb")
        self.connected_to = (ip, port)
        log(f"command link connected to {ip}:{port}")

    def _roundtrip(self, req: dict) -> dict:
        self._connect()
        # measure_height replies only after ~2–5 s of LiDAR capture on the phone.
        self.sock.settimeout(10.0 if req.get("cmd") == "measure_height" else 3.0)
        self.sock.sendall((json.dumps(req, separators=(",", ":")) + "\n").encode())
        line = self.rfile.readline(65536)
        if not line:
            raise ConnectionError("phone closed the command connection")
        return P.decode_line(line)

    def send(self, req: dict, quiet: bool = False) -> dict:
        """Send one command; returns the phone's reply or {"ok":false,...,"local":true}."""
        cmd = req.get("cmd", "?")
        with self.lock:
            for attempt in (0, 1):
                try:
                    reply = self._roundtrip(req)
                    self._connect_fails = 0
                    self.last_ok = time.time()
                    self._last_traffic = time.time()
                    self.last_error = ""
                    return reply
                except (OSError, ValueError, ConnectionError) as e:
                    err = f"{type(e).__name__}: {e}" if str(e) else type(e).__name__
                    self.last_error = err
                    self._close()
                    tgt = self.target()
                    if tgt and tgt[2] == "bonjour":
                        self._connect_fails += 1
                        if self._connect_fails >= self.MAX_BONJOUR_FAILS:
                            log(f"phone not reachable at {tgt[0]}; rediscovering")
                            self.bonjour_result = None
                            self._connect_fails = 0
                            self._last_bonjour = 0.0
                    if isinstance(e, (socket.timeout, TimeoutError)) or attempt == 1 \
                            or self.target() is None:
                        break
            if not quiet:
                log(f"command {cmd} failed: {self.last_error}")
            return {"ok": False, "cmd": cmd, "error": self.last_error, "local": True}

    def _keepalive(self):
        while self.running:
            time.sleep(0.5)
            now = time.time()
            tgt = self.target()
            if tgt is None:
                if (self.bonjour_enabled and not self._bonjour_running
                        and now - self._last_bonjour > 10.0):
                    self._last_bonjour = now
                    self._bonjour_running = True
                    threading.Thread(target=self._bonjour, daemon=True).start()
                continue
            needs_connect = self.connected_to != (tgt[0], tgt[1])
            if needs_connect and now - self._last_traffic < 2.0:
                continue  # back off between failed attempts
            if needs_connect or now - self._last_traffic > self.KEEPALIVE_S:
                self._last_traffic = now
                self.send({"cmd": "ping"}, quiet=True)

    def _bonjour(self):
        try:
            res = bonjour_discover()
            if res:
                if res != self.bonjour_result:
                    log(f"bonjour found phone at {res[0]}:{res[1]}")
                self.bonjour_result = res
        finally:
            self._bonjour_running = False

    def status(self) -> dict:
        tgt = self.target()
        return {
            "target_ip": tgt[0] if tgt else None,
            "target_port": tgt[1] if tgt else self.port,
            "target_source": tgt[2] if tgt else None,
            "explicit_ip": self.explicit_ip,
            "connected": self.connected_to is not None,
            "last_error": self.last_error,
            "last_ok_age": None if self.last_ok is None else round(time.time() - self.last_ok, 1),
            "bonjour": self.bonjour_result[0] if self.bonjour_result else None,
        }

    def close(self):
        self.running = False
        with self.lock:
            self._close()


# --------------------------------------------------------------------------- HTTP

class Dashboard:
    def __init__(self, args):
        self.args = args
        self.telem = Telemetry()
        self.logger = SessionLogger(None if args.no_log else Path(args.log_dir))
        self.udp = UDPReceiver(self.telem, self.logger, args.udp_host, args.udp_port)
        self.phone = PhoneLink(self.telem, args.cmd_port, args.phone,
                               bonjour=not args.no_bonjour)
        self.httpd = ThreadingHTTPServer((args.bind, args.http_port), _make_handler(self))
        self.httpd.daemon_threads = True
        self.http_port = self.httpd.server_address[1]
        self.udp_port = self.udp.port
        self.cmd_log = collections.deque(maxlen=100)
        self.cmd_lock = threading.Lock()

    def start(self):
        self.udp.start()
        threading.Thread(target=self.httpd.serve_forever, daemon=True, name="http").start()
        log(f"UDP telemetry on {self.args.udp_host}:{self.udp_port}; "
            f"UI at http://{'localhost' if self.args.bind in ('127.0.0.1', '0.0.0.0') else self.args.bind}:{self.http_port}/")

    def stop(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.udp.stop()
        self.phone.close()
        self.logger.close()

    def command(self, req: dict) -> dict:
        reply = self.phone.send(req)
        entry = {"time": time.time(), "req": req, "reply": reply}
        with self.cmd_lock:
            self.cmd_log.append(entry)
        self.logger.command(req, reply)
        return reply

    def status(self) -> dict:
        return {
            "link": self.telem.stats(),
            "phone": self.phone.status(),
            "log_dir": str(self.logger.dir) if self.logger.dir else None,
            "udp_port": self.udp_port,
            "last_frame": self.telem.last_frame,
            "server_time": time.time(),
        }


def _make_handler(dash: Dashboard):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "gsdash/0.1"

        def log_message(self, fmt, *a):  # quiet
            pass

        def _send_json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self):
            n = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(n) if n else b"{}"
            obj = json.loads(raw or b"{}")
            if not isinstance(obj, dict):
                raise ValueError("body must be a JSON object")
            return obj

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/events":
                return self._sse()
            if path == "/api/status":
                return self._send_json(dash.status())
            if path == "/api/cmdlog":
                with dash.cmd_lock:
                    return self._send_json(list(dash.cmd_log))
            if path == "/":
                path = "/index.html"
            f = (STATIC_DIR / path.lstrip("/")).resolve()
            if STATIC_DIR not in f.parents or not f.is_file():
                return self._send_json({"error": "not found"}, 404)
            body = f.read_bytes()
            ctype = mimetypes.guess_type(f.name)[0] or "application/octet-stream"
            if ctype.startswith("text/") or ctype.endswith("javascript"):
                ctype += "; charset=utf-8"
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            path = self.path.split("?", 1)[0]
            try:
                body = self._read_json()
            except (ValueError, json.JSONDecodeError) as e:
                return self._send_json({"ok": False, "error": f"bad request: {e}"}, 400)
            if path == "/api/cmd":
                cmd = body.get("cmd")
                if not isinstance(cmd, str) or not cmd:
                    return self._send_json({"ok": False, "error": "missing cmd"}, 400)
                return self._send_json(dash.command(body))
            if path == "/api/phone":
                dash.phone.set_explicit(body.get("ip"))
                return self._send_json({"ok": True, "phone": dash.phone.status()})
            if path == "/api/reset_stats":
                dash.telem.reset_stats()
                return self._send_json({"ok": True})
            return self._send_json({"error": "not found"}, 404)

        def _sse(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            self.close_connection = True
            telem = dash.telem
            idx = telem.next_idx  # live stream starts with new frames
            last_stats = 0.0
            try:
                self.wfile.write(b"retry: 1000\n\n")
                # backfill recent frames (?history=seconds, default 60) so a reloaded page has charts
                now = time.time()
                try:
                    q = dict(p.split("=", 1) for p in self.path.split("?", 1)[1].split("&") if "=" in p)
                    hist_s = min(max(float(q.get("history", 60)), 0.0), 80.0)
                except (IndexError, ValueError):
                    hist_s = 60.0
                hist, _ = telem.frames_since(0)
                hist = [f for f in hist if now - f["recv_time"] <= hist_s]
                self.wfile.write(b"event: history\ndata: " + json.dumps(
                    {"now": now, "frames": hist}, separators=(",", ":")).encode() + b"\n\n")
                while True:
                    t0 = time.time()
                    with telem.cond:
                        if telem.next_idx == idx:
                            telem.cond.wait(timeout=0.2)
                    frames, idx = telem.frames_since(idx)
                    chunks = []
                    if frames:
                        frames = frames[-100:]
                        chunks.append(b"event: frames\ndata: " +
                                      json.dumps(frames, separators=(",", ":")).encode() +
                                      b"\n\n")
                    now = time.time()
                    if now - last_stats >= 0.2:
                        last_stats = now
                        st = dash.status()
                        st.pop("last_frame", None)
                        chunks.append(b"event: stats\ndata: " +
                                      json.dumps(st, separators=(",", ":")).encode() +
                                      b"\n\n")
                    if chunks:
                        self.wfile.write(b"".join(chunks))
                        self.wfile.flush()
                    # cap push rate at 50 Hz (frames arriving faster get batched)
                    dt = time.time() - t0
                    if dt < 0.02:
                        time.sleep(0.02 - dt)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return

    return Handler


def build_arg_parser():
    ap = argparse.ArgumentParser(prog="gsdash", description="GroundSpeed telemetry dashboard")
    ap.add_argument("--udp-port", type=int, default=9000, help="UDP telemetry port (9000)")
    ap.add_argument("--udp-host", default="0.0.0.0", help="UDP bind address (0.0.0.0)")
    ap.add_argument("--http-port", type=int, default=8080, help="web UI port (8080)")
    ap.add_argument("--bind", default="127.0.0.1",
                    help="web UI bind address (127.0.0.1; 0.0.0.0 to expose on the LAN)")
    ap.add_argument("--phone", default=None, help="phone IP for commands (default: auto)")
    ap.add_argument("--cmd-port", type=int, default=9001, help="phone TCP command port (9001)")
    ap.add_argument("--log-dir", default=str(DEFAULT_LOG_DIR), help="session log root")
    ap.add_argument("--no-log", action="store_true", help="do not write CSV logs")
    ap.add_argument("--no-bonjour", action="store_true", help="disable dns-sd discovery")
    return ap


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    try:
        dash = Dashboard(args)
    except OSError as e:
        log(f"cannot bind: {e} (is another gsdash already running?)")
        sys.exit(1)
    dash.start()
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        log("shutting down")
    finally:
        dash.stop()


if __name__ == "__main__":
    main()
