"""Replay recorded runs from the session logs, looped, through the live pipeline.

Catalog: every RECORDING segment (phone status bit) in every <root>/<session>/telemetry.csv,
numbered oldest-first across all sessions (#1, #2, ...). Roots are the live log directory and
the archive in data/dashboard_logs; a session present in both is read once. Short blips
(< MIN_RUN_S) are skipped.

Playback: frames are re-timed by their original recv_time spacing and pushed into Telemetry
exactly like UDP frames (tagged replay=True, replay_loop=n), so the UI draws them with the
same charts. While a replay runs, live UDP frames are still logged but not displayed.
"""
import csv
import threading
import time
from pathlib import Path

PAD_S = 1.5          # show a little of the stillness before/after the run
MIN_RUN_S = 2.0
LOOP_GAP_S = 2.0     # pause on the final frame before looping
NUM = ("t", "v_x", "v_y", "sigma_vx", "sigma_vy", "distance", "flow_quality", "h", "ax", "ay", "gz",
       "flow_vx", "flow_vy", "net_forward")
REC_BIT = 16


def _num(v):
    if v in (None, "", "None"):
        return None
    try:
        return float(v)
    except ValueError:
        return None


def _read(path: Path):
    rows = []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            st, rt = _num(r.get("status")), _num(r.get("recv_time"))
            if st is None or rt is None or _num(r.get("t")) is None:
                continue
            f = {k: _num(r.get(k)) for k in NUM}
            f["status"] = int(st)
            f["seq"] = int(_num(r.get("seq")) or 0)
            f["version"] = int(_num(r.get("version")) or 1)
            f["_recv"] = rt
            rows.append(f)
    return rows


class Replayer:
    def __init__(self, telem, *roots):
        self.telem = telem
        self.roots = [Path(r) for r in roots if r]
        self.lock = threading.Lock()
        self.thread = None
        self.stop_ev = threading.Event()
        self.state = {"active": False}
        self._cache = {}              # path -> (mtime, rows)

    # ------------------------------------------------------------- catalog
    def _rows(self, path: Path):
        m = path.stat().st_mtime
        c = self._cache.get(path)
        if c and c[0] == m:
            return c[1]
        rows = _read(path)
        self._cache[path] = (m, rows)
        return rows

    def catalog(self):
        runs, sessions = [], {}
        for root in self.roots:
            if root.is_dir():
                for d in root.iterdir():
                    if d.is_dir() and (d / "telemetry.csv").is_file():
                        sessions.setdefault(d.name, d)
        for name in sorted(sessions):
            d = sessions[name]
            f = d / "telemetry.csv"
            if not f.is_file():
                continue
            rows = self._rows(f)
            i0 = None
            for i, r in enumerate(rows + [None]):
                rec = r is not None and r["status"] & REC_BIT
                if rec and i0 is None:
                    i0 = i
                elif not rec and i0 is not None:
                    a, b = rows[i0], rows[i - 1]
                    dur = b["t"] - a["t"]
                    if dur >= MIN_RUN_S:
                        seg = rows[i0:i]
                        sp = [((x["v_x"] or 0) ** 2 + (x["v_y"] or 0) ** 2) ** 0.5 for x in seg]
                        runs.append({"session": d.name, "path": str(f), "i0": i0, "i1": i - 1, "start_wall": a["_recv"],
                                     "duration": round(dur, 2),
                                     "distance": round(max((x["distance"] or 0) for x in seg), 3),
                                     "net_forward": round(b["net_forward"] or 0, 3) if b["net_forward"] is not None else None,
                                     "max_speed": round(max(sp), 3)})
                    i0 = None
        for n, r in enumerate(runs, 1):
            r["n"] = n
            r["id"] = f'{r["session"]}#{r["i0"]}'
        return runs

    # ------------------------------------------------------------- playback
    def start(self, run_id=None, n=None, loop=True, speed=1.0):
        cat = self.catalog()
        run = next((r for r in cat if (run_id and r["id"] == run_id) or (n and r["n"] == int(n))), None)
        if run is None:
            return {"ok": False, "error": f"run {run_id or n} not found ({len(cat)} runs in logs)"}
        rows = self._rows(Path(run["path"]))
        a, b = rows[run["i0"]], rows[run["i1"]]
        seg = [r for r in rows if a["_recv"] - PAD_S <= r["_recv"] <= b["_recv"] + PAD_S]
        self.stop()
        self.stop_ev = threading.Event()
        with self.lock:
            self.state = {"active": True, "n": run["n"], "id": run["id"], "session": run["session"],
                          "duration": run["duration"], "distance": run["distance"], "loop": 0,
                          "progress": 0.0, "loop_enabled": bool(loop), "speed": float(speed)}
        self.telem.replay_active = True
        self.thread = threading.Thread(target=self._run, args=(seg, bool(loop), max(0.1, float(speed)), self.stop_ev),
                                       daemon=True, name="replay")
        self.thread.start()
        return {"ok": True, "replay": self.status()}

    def _run(self, seg, loop, speed, stop_ev):
        n = 0
        while not stop_ev.is_set():
            n += 1
            with self.lock:
                self.state["loop"] = n
            t0w, r0 = time.time(), seg[0]["_recv"]
            span = max(seg[-1]["_recv"] - r0, 1e-6)
            for r in seg:
                due = t0w + (r["_recv"] - r0) / speed
                while not stop_ev.is_set():
                    dt = due - time.time()
                    if dt <= 0:
                        break
                    stop_ev.wait(min(dt, 0.05))
                if stop_ev.is_set():
                    return
                f = {k: v for k, v in r.items() if not k.startswith("_")}
                f.update(fmt="replay", replay=True, replay_loop=n, replay_n=self.state.get("n"))
                self.telem.add_replay_frame(f)
                with self.lock:
                    self.state["progress"] = round((r["_recv"] - r0) / span, 3)
            if not loop:
                break
            stop_ev.wait(LOOP_GAP_S)
        with self.lock:
            if self.stop_ev is stop_ev:
                self.state["active"] = False
                self.telem.replay_active = False

    def stop(self):
        self.stop_ev.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        with self.lock:
            was = self.state.get("active")
            self.state = {"active": False}
        self.telem.replay_active = False
        self.telem.purge_replay()
        return {"ok": True, "was_active": bool(was)}

    def status(self):
        with self.lock:
            return dict(self.state)
