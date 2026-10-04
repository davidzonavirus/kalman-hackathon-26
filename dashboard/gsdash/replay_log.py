"""Replay a recorded dashboard log (data/dashboard_logs/<session>/telemetry.csv) to a running
dashboard over UDP, frame by frame at the original rate, like a phone would.

    python -m gsdash.replay_log ../data/dashboard_logs/2026-10-04_01-57-54 --start 5800 --seconds 20

Start the dashboard first (``./run.sh --fpga``). The frames carry the raw sensor columns (a_x, a_y,
gyro z, flow v_x / v_y / PSR) and the phone's own estimate; with ``--fpga`` the dashboard
replaces that estimate by the simulated FPGA's. Standard library only.
"""
from __future__ import annotations

import argparse
import csv
import socket
import time
from pathlib import Path

from . import protocol as P


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="session folder or telemetry.csv")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--start", type=int, default=0, help="first row to send")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this much log time (0 = to the end)")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed (1 = real time)")
    ap.add_argument("--loop", action="store_true", help="start over when the stretch ends, until Ctrl+C")
    a = ap.parse_args(argv)

    path = Path(a.log)
    path = path / "telemetry.csv" if path.is_dir() else path
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))[a.start:]
    if not rows:
        raise SystemExit("no rows")

    def num(r, k, default=0.0):
        v = r.get(k, "")
        return float(v) if v not in ("", None) else default

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t0_log = num(rows[0], "t")
    sent, seq = 0, 0
    try:
        while True:
            t0_wall = time.time()
            t = t0_log
            for r in rows:
                t = num(r, "t")
                if a.seconds and t - t0_log > a.seconds:
                    break
                wait = t0_wall + (t - t0_log) / a.speed - time.time()
                if wait > 0:
                    time.sleep(wait)
                frame = P.encode_frame(
                    seq=seq, t=t, v_x=num(r, "v_x"), v_y=num(r, "v_y"), sigma_vx=num(r, "sigma_vx"),
                    sigma_vy=num(r, "sigma_vy"), distance=num(r, "distance"),
                    flow_quality=num(r, "flow_quality"), h=num(r, "h"), status=int(num(r, "status")),
                    ax=num(r, "ax"), ay=num(r, "ay"), gz=num(r, "gz"), flow_vx=num(r, "flow_vx"),
                    flow_vy=num(r, "flow_vy"), net_forward=num(r, "net_forward"), version=2)
                sock.sendto(frame, (a.host, a.port))
                sent += 1
                seq += 1
            if not a.loop:
                break
    except KeyboardInterrupt:
        pass
    print(f"sent {sent} frames to {a.host}:{a.port}")


if __name__ == "__main__":
    main()
