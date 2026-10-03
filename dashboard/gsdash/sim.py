"""Fake phone for testing the dashboard without the iOS app.

    python3 -m gsdash.sim [--host 127.0.0.1] [--json] [--loss 0.05]

Streams 50 Hz telemetry of a looping 10 m rest-to-rest run (with a 2 s "lens covered"
segment: FLOW_OK off, sigma growing) to UDP host:9000 and serves the TCP 9001 command
protocol. Like the real phone, it adopts the IP of a TCP command client as its UDP
destination.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import socket
import socketserver
import sys
import threading
import time

from . import protocol as P

REST_BEFORE = 3.0
RUN_T = 20.0          # cosine velocity profile: distance = vmax * RUN_T / 2
VMAX = 1.0            # -> 10 m
REST_AFTER = 4.0
CYCLE = REST_BEFORE + RUN_T + REST_AFTER
COVER_START = 8.0     # seconds into the run
COVER_LEN = 2.0
EST_SCALE = 1.008     # simulated estimator bias -> ~0.8 % distance error


class FakePhone:
    def __init__(self, host, port, rate=50.0, json_mode=False, loss=0.0, seed=1,
                 seq_start=0):
        self.dest = (host, port)
        self.rate = rate
        self.json_mode = json_mode
        self.loss = loss
        self.rng = random.Random(seed)
        self.lock = threading.Lock()
        self.seq = seq_start & 0xFFFFFFFF
        self.distance = 0.0
        self.recording = False
        self.run_id = None
        self.torch = 0.0
        self.calib_until = 0.0
        self._prev_tc = 0.0
        self.sent = 0
        self.dropped = 0
        self.running = True
        self.t0 = time.monotonic()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # ---- motion model
    def truth(self, tc):
        """True forward speed at cycle time tc."""
        tr = tc - REST_BEFORE
        if 0.0 <= tr <= RUN_T:
            return VMAX * 0.5 * (1.0 - math.cos(2 * math.pi * tr / RUN_T)), tr
        return 0.0, tr

    def step(self, t, dt):
        tc = t % CYCLE
        v_true, tr = self.truth(tc)
        covered = COVER_START <= tr < COVER_START + COVER_LEN
        moving = 0.0 <= tr <= RUN_T
        n = self.rng.gauss
        if covered:
            k = tr - COVER_START
            sx = 0.02 + 0.12 * k + 0.05 * k * k
            sy = 0.015 + 0.08 * k + 0.03 * k * k
            vx = v_true * EST_SCALE + 0.04 * k * k + n(0, 0.01)
            vy = 0.02 * k + n(0, 0.01)
            fq = 0.0
        else:
            sx, sy = 0.02 + n(0, 0.001), 0.015 + n(0, 0.001)
            sx, sy = abs(sx), abs(sy)
            vx = v_true * EST_SCALE + n(0, 0.008)
            vy = n(0, 0.006)
            fq = max(0.0, 12.0 + n(0, 1.5)) if moving else max(0.0, 9.0 + n(0, 1.0))
        if not moving:
            vx, vy = n(0, 0.002), n(0, 0.002)
        status = P.IMU_OK | P.LIDAR_OK | P.FILTER_INIT
        if not covered:
            status |= P.FLOW_OK
        if covered and int(tr * 10) % 4 == 0:
            status |= P.FLOW_GATED
        if moving and v_true > 0.6 and int(t) % 3 != 0:
            status |= P.GNSS_OK
        if not moving:
            status |= P.ZUPT
        with self.lock:
            wrapped = tc < self._prev_tc
            self._prev_tc = tc
            if wrapped and not self.recording:
                self.distance = 0.0   # new demo cycle starts from 0 unless a run is open
            self.distance += math.hypot(vx, vy) * dt if moving else 0.0
            if self.recording:
                status |= P.RECORDING
            if self.torch > 0:
                status |= P.TORCH_ON
            if time.monotonic() < self.calib_until:
                status |= P.CALIBRATING
            seq = self.seq
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            fields = dict(seq=seq, t=t, v_x=vx, v_y=vy, sigma_vx=sx, sigma_vy=sy,
                          distance=self.distance, flow_quality=fq,
                          h=0.30 + n(0, 0.0005), status=status,
                          battery=max(0, 87 - int(t / 120)), torch=int(round(self.torch * 100)))
            dest = self.dest
        enc = P.encode_json_frame if self.json_mode else P.encode_frame
        data = enc(**fields)
        if self.loss > 0 and self.rng.random() < self.loss:
            self.dropped += 1
            return
        try:
            self.sock.sendto(data, dest)
            self.sent += 1
        except OSError:
            pass

    def run(self):
        period = 1.0 / self.rate
        nxt = time.monotonic()
        last = None
        while self.running:
            now = time.monotonic()
            t = now - self.t0 + 1000.0   # pretend mach uptime
            self.step(t, period if last is None else min(now - last, 0.2))
            last = now
            nxt += period
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                nxt = time.monotonic()
        self.sock.close()

    # ---- commands
    def handle(self, req: dict, peer_ip: str) -> dict:
        cmd = req.get("cmd")
        with self.lock:
            if cmd == "ping":
                return {"ok": True, "cmd": "ping", "t": time.monotonic() - self.t0 + 1000.0}
            if cmd == "start_run":
                if self.recording:
                    return {"ok": False, "cmd": cmd, "error": "already recording"}
                label = req.get("label")
                rid = time.strftime("%Y-%m-%d_%H-%M-%S") + (f"_{label}" if label else "")
                self.recording, self.run_id, self.distance = True, rid, 0.0
                return {"ok": True, "cmd": cmd, "run": rid}
            if cmd == "stop_run":
                if not self.recording:
                    return {"ok": False, "cmd": cmd, "error": "not recording"}
                self.recording = False
                return {"ok": True, "cmd": cmd, "run": self.run_id,
                        "distance": round(self.distance, 4)}
            if cmd == "mark":
                return {"ok": True, "cmd": cmd}
            if cmd == "calibrate":
                self.calib_until = time.monotonic() + 2.0
                return {"ok": True, "cmd": cmd}
            if cmd == "set_torch":
                try:
                    lvl = float(req.get("level"))
                except (TypeError, ValueError):
                    return {"ok": False, "cmd": cmd, "error": "level must be 0..1"}
                if not 0.0 <= lvl <= 1.0:
                    return {"ok": False, "cmd": cmd, "error": "level must be 0..1"}
                self.torch = lvl
                return {"ok": True, "cmd": cmd, "level": lvl}
            if cmd == "reset_distance":
                self.distance = 0.0
                return {"ok": True, "cmd": cmd}
        return {"ok": False, "cmd": cmd if isinstance(cmd, str) else "?",
                "error": f"unknown cmd {cmd!r}"}

    def adopt(self, ip):
        with self.lock:
            if self.dest[0] != ip:
                print(f"[sim] command client {ip}: sending UDP there", file=sys.stderr)
                self.dest = (ip, self.dest[1])


def make_tcp_server(phone: FakePhone, host: str, port: int, adopt_peer: bool = True):
    class H(socketserver.StreamRequestHandler):
        def handle(self):
            ip = self.client_address[0]
            if adopt_peer:
                phone.adopt(ip)
            for line in self.rfile:
                line = line.strip()
                if not line:
                    continue
                try:
                    req = P.decode_line(line)
                    rep = phone.handle(req, ip)
                except ValueError as e:
                    rep = {"ok": False, "cmd": "?", "error": f"bad json: {e}"}
                self.wfile.write((json.dumps(rep) + "\n").encode())
                self.wfile.flush()

    class S(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True

    return S((host, port), H)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="gsdash.sim", description="fake GroundSpeed phone")
    ap.add_argument("--host", default="127.0.0.1", help="dashboard IP (UDP destination)")
    ap.add_argument("--port", type=int, default=9000, help="dashboard UDP port")
    ap.add_argument("--cmd-port", type=int, default=9001, help="TCP command port to serve")
    ap.add_argument("--cmd-host", default="0.0.0.0")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--json", action="store_true", help="send JSON debug frames")
    ap.add_argument("--loss", type=float, default=0.0, help="drop probability 0..1")
    ap.add_argument("--seq-start", type=int, default=0,
                    help="initial seq (e.g. 4294967000 to test u32 wrap)")
    ap.add_argument("--no-adopt", action="store_true",
                    help="keep UDP destination even when a command client connects")
    a = ap.parse_args(argv)
    phone = FakePhone(a.host, a.port, a.rate, a.json, a.loss, seq_start=a.seq_start)
    srv = make_tcp_server(phone, a.cmd_host, a.cmd_port, adopt_peer=not a.no_adopt)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[sim] {'JSON' if a.json else 'binary'} frames @ {a.rate:g} Hz -> "
          f"{a.host}:{a.port}; commands on TCP {a.cmd_port}; loss {a.loss:g}",
          file=sys.stderr)
    th = threading.Thread(target=phone.run, daemon=True)
    th.start()
    try:
        while True:
            time.sleep(5)
            print(f"[sim] sent {phone.sent} dropped {phone.dropped} "
                  f"dist {phone.distance:.2f} m", file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        phone.running = False
        srv.shutdown()


if __name__ == "__main__":
    main()
