import csv, tempfile, time, unittest
from pathlib import Path

from gsdash.replay import Replayer
from gsdash.server import Telemetry

HDR = "t,v_x,v_y,sigma_vx,sigma_vy,distance,status,flow_quality,h,ax,ay,gz,flow_vx,flow_vy,net_forward,seq,recv_time,version".split(",")


def write_log(root, name, segs):
    """segs: list of (n_frames, recording) at 50 Hz."""
    d = Path(root) / name; d.mkdir(parents=True)
    t, rt, dist, seq = 100.0, 1_000_000.0, 0.0, 0
    with open(d / "telemetry.csv", "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(HDR)
        for n, rec in segs:
            for _ in range(n):
                dist += 0.02 if rec else 0
                w.writerow([t, 1.0 if rec else 0, 0, .03, .03, dist, 3 | (16 if rec else 0), 100, .2,
                            0, 0, 0, 1.0, 0, dist, seq, rt, 2])
                t += .02; rt += .02; seq += 1


class ReplayTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        write_log(self.tmp.name, "2026-10-03_10-00-00", [(50, False), (150, True), (50, False), (20, True), (50, False)])
        write_log(self.tmp.name, "2026-10-03_11-00-00", [(25, False), (120, True), (25, False)])
        self.telem = Telemetry()
        self.rp = Replayer(self.telem, self.tmp.name)

    def tearDown(self):
        self.rp.stop(); self.tmp.cleanup()

    def test_catalog_numbers_runs_across_sessions(self):
        cat = self.rp.catalog()
        self.assertEqual([r["n"] for r in cat], [1, 2])          # 0.4 s blip skipped
        self.assertEqual(cat[1]["session"], "2026-10-03_11-00-00")
        self.assertAlmostEqual(cat[0]["duration"], 2.98, places=2)

    def test_loop_and_stop(self):
        live = {"seq": 1, "t": 1.0}
        self.assertTrue(self.rp.start(n=2, loop=True, speed=20)["ok"])
        self.telem.add_frame(dict(live), ("1.2.3.4", 1))         # live frame during replay: not shown
        deadline = time.time() + 5
        while self.rp.status().get("loop", 0) < 2 and time.time() < deadline:
            time.sleep(0.05)
        self.assertGreaterEqual(self.rp.status()["loop"], 2)
        frames, _ = self.telem.frames_since(0)
        self.assertTrue(frames and all(f.get("replay") for f in frames))
        self.assertEqual(self.telem.seq.received, 1)              # link stats untouched by replay
        self.rp.stop()
        self.assertFalse(self.rp.status()["active"])
        self.assertEqual(self.telem.frames_since(0)[0], [])       # history purged
        self.telem.add_frame(dict(live, seq=2, t=1.02), ("1.2.3.4", 1))
        self.assertEqual(len(self.telem.frames_since(0)[0]), 1)   # live again

    def test_unknown_run(self):
        self.assertFalse(self.rp.start(n=99)["ok"])


if __name__ == "__main__":
    unittest.main()
