"""Fake phone for testing the dashboard without the iOS app.

    python3 -m gsdash.sim [--host 127.0.0.1] [--json] [--v1] [--loss 0.05]

Streams 50 Hz v2 telemetry (or v1 with --v1) of a looping 10 m rest-to-rest push that
alternates forward and backward, so total distance grows while net_forward returns to ~0.
Each push has a 2 s "lens covered" segment (FLOW_OK off, sigma growing). Serves the
TCP 9001 command protocol, including "zero". Like the real phone, it adopts the IP of a
TCP command client as its UDP destination, and it accepts the Kalman estimates a dashboard
running with --fpga sends back on the telemetry socket (PROTOCOL.md §4).
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
                 seq_start=0, version=2):
        self.dest = (host, port)
        self.rate = rate
        self.json_mode = json_mode
        self.loss = loss
        self.rng = random.Random(seed)
        self.lock = threading.Lock()
        self.seq = seq_start & 0xFFFFFFFF
        self.version = version
        self.distance = 0.0
        self.net_forward = 0.0
        self._last_flow = (0.0, 0.0)
        self.recording = False
        self.run_id = None
        self.torch = 0.0
        self.calib_until = 0.0
        self.sent = 0
        self.dropped = 0
        self.running = True
        self.t0 = time.monotonic()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        # Estimates a dashboard running the Kalman filter on the FPGA sends back (PROTOCOL.md §4).
        self.fpga_est = None
        self.fpga_est_count = 0
        self.fpga_est_at = None

    def poll_estimates(self):
        """Read every FPGA estimate datagram waiting on the telemetry socket."""
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:          # e.g. Windows reports ICMP port-unreachable here
                return
            est = P.decode_fpga_estimate(data)
            if est is not None:
                with self.lock:
                    self.fpga_est = est
                    self.fpga_est_count += 1
                    self.fpga_est_at = time.monotonic()

    # ---- motion model
    def truth(self, t):
        """True forward speed and acceleration at time t. Odd cycles drive backwards, so
        net_forward returns to ~0 while total distance keeps growing."""
        tc = t % CYCLE
        direction = 1.0 if int(t // CYCLE) % 2 == 0 else -1.0
        tr = tc - REST_BEFORE
        if 0.0 <= tr <= RUN_T:
            w = 2 * math.pi / RUN_T
            v = direction * VMAX * 0.5 * (1.0 - math.cos(w * tr))
            a = direction * VMAX * 0.5 * w * math.sin(w * tr)
            return v, a, tr, direction
        return 0.0, 0.0, tr, direction

    def step(self, t, dt):
        v_true, a_true, tr, direction = self.truth(t)
        covered = COVER_START <= tr < COVER_START + COVER_LEN
        moving = 0.0 <= tr <= RUN_T
        n = self.rng.gauss
        if covered:
            k = tr - COVER_START
            sx = 0.02 + 0.12 * k + 0.05 * k * k
            sy = 0.015 + 0.08 * k + 0.03 * k * k
            vx = v_true * EST_SCALE + direction * 0.04 * k * k + n(0, 0.01)
            vy = 0.02 * k + n(0, 0.01)
            fq = 0.0
        else:
            sx, sy = abs(0.02 + n(0, 0.001)), abs(0.015 + n(0, 0.001))
            vx = v_true * EST_SCALE + n(0, 0.008)
            vy = n(0, 0.006)
            fq = max(0.0, 12.0 + n(0, 1.5)) if moving else max(0.0, 9.0 + n(0, 1.0))
        if not moving:
            vx, vy = n(0, 0.002), n(0, 0.002)
        ax = a_true + n(0, 0.04)
        ay = n(0, 0.03)
        gz = n(0, 0.01)
        if covered:
            fvx, fvy = self._last_flow
        else:
            fvx, fvy = v_true + n(0, 0.03), n(0, 0.02)
            self._last_flow = (fvx, fvy)
        status = P.IMU_OK | P.LIDAR_OK | P.FILTER_INIT
        if not covered:
            status |= P.FLOW_OK
        if covered and int(tr * 10) % 4 == 0:
            status |= P.FLOW_GATED
        if moving and abs(v_true) > 0.6 and int(t) % 3 != 0:
            status |= P.GNSS_OK
        if not moving:
            status |= P.ZUPT
        with self.lock:
            if moving:   # total distance excludes stationary periods
                self.distance += math.hypot(vx, vy) * dt
            self.net_forward += vx * dt
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
            if self.version == 2:
                fields.update(ax=ax, ay=ay, gz=gz, flow_vx=fvx, flow_vy=fvy,
                              net_forward=self.net_forward, version=2)
            else:
                fields["version"] = 1
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
            self.poll_estimates()
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
                self.recording, self.run_id = True, rid
                self.distance = self.net_forward = 0.0
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
            if cmd == "zero":
                self.distance = self.net_forward = 0.0
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
    ap.add_argument("--v1", action="store_true", help="send v1 (48-byte) frames instead of v2")
    ap.add_argument("--loss", type=float, default=0.0, help="drop probability 0..1")
    ap.add_argument("--seq-start", type=int, default=0,
                    help="initial seq (e.g. 4294967000 to test u32 wrap)")
    ap.add_argument("--no-adopt", action="store_true",
                    help="keep UDP destination even when a command client connects")
    a = ap.parse_args(argv)
    phone = FakePhone(a.host, a.port, a.rate, a.json, a.loss, seq_start=a.seq_start,
                      version=1 if a.v1 else 2)
    srv = make_tcp_server(phone, a.cmd_host, a.cmd_port, adopt_peer=not a.no_adopt)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"[sim] v{phone.version} {'JSON' if a.json else 'binary'} frames @ {a.rate:g} Hz -> "
          f"{a.host}:{a.port}; commands on TCP {a.cmd_port}; loss {a.loss:g}",
          file=sys.stderr)
    th = threading.Thread(target=phone.run, daemon=True)
    th.start()
    try:
        while True:
            time.sleep(5)
            line = f"[sim] sent {phone.sent} dropped {phone.dropped} dist {phone.distance:.2f} m"
            est = phone.fpga_est
            if est is not None and time.monotonic() - phone.fpga_est_at < 1.0:
                line += (f" | FPGA estimate back ({phone.fpga_est_count}): v_x {est['v_x']:+.3f} m/s, "
                         f"dist {est['distance']:.2f} m ({est.get('backend', '?')})")
            print(line, file=sys.stderr)
    except KeyboardInterrupt:
        pass
    finally:
        phone.running = False
        srv.shutdown()


if __name__ == "__main__":
    main()
