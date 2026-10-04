"""The Kalman filter on the simulated FPGA, inside the dashboard."""
import json
import socket
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gsdash import fpga_filter, server, sim  # noqa: E402
from gsdash import protocol as P  # noqa: E402

HAVE_IVERILOG = "iverilog" in fpga_filter.kf_rtl.available()


def frame(t, vx_flow=1.0, quality=100.0, status=P.IMU_OK | P.FLOW_OK, ax=0.0):
    return dict(seq=int(t * 50), t=t, v_x=9.0, v_y=9.0, sigma_vx=9.0, sigma_vy=9.0, distance=0.0,
                flow_quality=quality, h=0.2, status=status, battery=80, torch=0, ax=ax, ay=0.0,
                gz=0.0, flow_vx=vx_flow, flow_vy=0.0, net_forward=0.0, fmt="bin", version=2)


class TestFpgaFilter(unittest.TestCase):
    def run_frames(self, backend, n=150):
        flt = fpga_filter.FpgaFilter(backend)
        out = []
        for k in range(n):
            f = frame(k * 0.02, vx_flow=1.0 + 0.001 * k)      # a new flow sample every frame
            out.append(flt.process(f))
        return flt, out

    def test_tracks_flow_and_replaces_phone_estimate(self):
        flt, out = self.run_frames(fpga_filter.kf_isa.SimBackend())
        last = out[-1]
        self.assertNotEqual(last["v_x"], 9.0)
        self.assertAlmostEqual(last["v_x"], 1.0 + 0.001 * 149, delta=0.02)
        self.assertLess(last["sigma_vx"], 0.1)
        self.assertEqual(last["phone"]["v_x"], 9.0)             # the phone's value is kept
        self.assertTrue(last["status"] & P.FILTER_INIT)
        self.assertGreater(last["distance"], 1.0)
        self.assertGreater(last["net_forward"], 1.0)

    def test_matches_float_reference(self):
        """Same frames through the float port of the Swift filter: the FPGA agrees to 1e-6."""
        flt, out = self.run_frames(fpga_filter.kf_isa.SimBackend())
        ref = fpga_filter.kf_ref.ReferenceKF4()
        for k in range(150):
            t = k * 0.02
            ref.predict(t, 0.0, 0.0, 0.0)
            ref.update_flow(t, 1.0 + 0.001 * k, 0.0, 100.0)
        self.assertAlmostEqual(out[-1]["v_x"], ref.state.vx, delta=1e-6)

    def test_outlier_is_gated_and_flagged(self):
        flt = fpga_filter.FpgaFilter(fpga_filter.kf_isa.SimBackend())
        for k in range(100):
            flt.process(frame(k * 0.02, vx_flow=1.0))
        f = flt.process(frame(2.0, vx_flow=9.0))                # 8 m/s jump: far outside 3 sigma
        self.assertTrue(f["status"] & P.FLOW_GATED)
        self.assertAlmostEqual(f["v_x"], 1.0, delta=0.05)

    def test_zupt_bit_pulls_velocity_to_zero(self):
        flt = fpga_filter.FpgaFilter(fpga_filter.kf_isa.SimBackend())
        for k in range(100):
            flt.process(frame(k * 0.02, vx_flow=1.0))
        for k in range(100, 130):
            f = flt.process(frame(k * 0.02, vx_flow=1.0, quality=0.0, status=P.IMU_OK | P.ZUPT))
        self.assertLess(abs(f["v_x"]), 0.01)

    def test_reset_distance_and_v1_frames(self):
        flt, out = self.run_frames(fpga_filter.kf_isa.SimBackend(), 60)
        self.assertGreater(out[-1]["distance"], 0.5)
        flt.reset_distance()
        f = flt.process(frame(1.2, vx_flow=1.0))
        self.assertLess(f["distance"], 0.05)
        v1 = dict(frame(1.3), ax=None)
        self.assertEqual(flt.process(v1)["v_x"], 9.0)           # no raw columns: left alone

    def test_distance_freezes_when_the_run_stops(self):
        flt = fpga_filter.FpgaFilter(fpga_filter.kf_isa.SimBackend())
        rec = P.IMU_OK | P.FLOW_OK | P.RECORDING
        for k in range(100):                                    # recording, moving at 1 m/s
            f = flt.process(frame(k * 0.02, vx_flow=1.0, status=rec))
        d_stop = f["distance"]
        self.assertGreater(d_stop, 0.5)
        for k in range(100, 200):                               # recording bit drops, still moving
            f = flt.process(frame(k * 0.02, vx_flow=1.0, status=P.IMU_OK | P.FLOW_OK))
        self.assertAlmostEqual(f["distance"], d_stop, delta=0.05)   # frozen (one frame of slack)
        self.assertGreater(f["v_x"], 0.9)                       # the estimate itself keeps running
        for k in range(200, 260):                               # next run starts: odometer from zero
            f = flt.process(frame(k * 0.02, vx_flow=1.0, status=rec))
        self.assertLess(f["distance"], 1.5)

    def test_clock_going_back_restarts_the_filter(self):
        flt = fpga_filter.FpgaFilter(fpga_filter.kf_isa.SimBackend())
        for k in range(200):
            f = flt.process(frame(100.0 + k * 0.02, vx_flow=2.0))
        self.assertAlmostEqual(f["v_x"], 2.0, delta=0.05)
        for k in range(200):                                    # a new session, earlier timestamps
            f = flt.process(frame(5.0 + k * 0.02, vx_flow=0.5))
        self.assertAlmostEqual(f["v_x"], 0.5, delta=0.05)       # tracking again, not stuck on the old time

    def test_stop_command_holds_and_start_resumes(self):
        flt = fpga_filter.FpgaFilter(fpga_filter.kf_isa.SimBackend())
        for k in range(100):
            f = flt.process(frame(k * 0.02, vx_flow=1.0))
        flt.set_hold(True)
        d = f["distance"]
        for k in range(100, 150):
            f = flt.process(frame(k * 0.02, vx_flow=1.0))
        self.assertEqual(f["distance"], d)
        flt.reset_distance(); flt.set_hold(False)
        for k in range(150, 200):
            f = flt.process(frame(k * 0.02, vx_flow=1.0))
        self.assertGreater(f["distance"], 0.5)
        self.assertLess(f["distance"], d)

    @unittest.skipUnless(HAVE_IVERILOG, "Icarus Verilog not installed")
    def test_rtl_backend_is_bit_exact_with_model(self):
        a, out_a = self.run_frames(fpga_filter.kf_isa.SimBackend(), 40)
        be = fpga_filter.kf_rtl.RtlBackend("iverilog")
        try:
            b, out_b = self.run_frames(be, 40)
            self.assertEqual([f["v_x"] for f in out_a], [f["v_x"] for f in out_b])
            self.assertEqual([f["sigma_vx"] for f in out_a], [f["sigma_vx"] for f in out_b])
            self.assertGreater(be.clocks, 1000)
        finally:
            be.close()


def free_port(kind=socket.SOCK_STREAM):
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class TestDashboardWithFpga(unittest.TestCase):
    def test_frames_go_through_the_fpga(self):
        tmp = tempfile.TemporaryDirectory()
        cmd_port = free_port()
        args = server.build_arg_parser().parse_args([
            "--udp-host", "127.0.0.1", "--udp-port", "0", "--http-port", "0", "--cmd-port", str(cmd_port),
            "--log-dir", tmp.name, "--no-bonjour", "--fpga", "--fpga-backend", "model"])
        dash = server.Dashboard(args)
        dash.start()
        phone = sim.FakePhone("127.0.0.1", dash.udp_port, rate=100.0)
        threading.Thread(target=phone.run, daemon=True).start()
        try:
            end = time.time() + 8
            st = None
            while time.time() < end:
                with urllib.request.urlopen(f"http://127.0.0.1:{dash.http_port}/api/status", timeout=5) as r:
                    st = json.loads(r.read())
                if st["fpga"] and st["fpga"]["frames"] > 50:
                    break
                time.sleep(0.2)
            self.assertIsNotNone(st["fpga"])
            self.assertEqual(st["fpga"]["backend"], "python-model")
            self.assertGreater(st["fpga"]["frames"], 50)
            lf = st["last_frame"]
            self.assertIn("phone", lf)                          # the phone's estimate is kept alongside
            self.assertTrue(lf["status"] & P.FILTER_INIT)
        finally:
            phone.running = False
            dash.stop()
            tmp.cleanup()

    def start(self, fpga: bool):
        tmp = tempfile.TemporaryDirectory()
        argv = ["--udp-host", "127.0.0.1", "--udp-port", "0", "--http-port", "0",
                "--cmd-port", str(free_port()), "--log-dir", tmp.name, "--no-bonjour"]
        if fpga:
            argv += ["--fpga", "--fpga-backend", "model"]
        dash = server.Dashboard(server.build_arg_parser().parse_args(argv))
        dash.start()
        phone = sim.FakePhone("127.0.0.1", dash.udp_port, rate=50.0)
        threading.Thread(target=phone.run, daemon=True).start()
        return tmp, dash, phone

    def test_phone_receives_the_fpga_estimate(self):
        """Phone -> dashboard -> FPGA -> back to the phone, over the telemetry socket."""
        tmp, dash, phone = self.start(fpga=True)
        try:
            end = time.time() + 8
            while time.time() < end and phone.fpga_est_count < 100:
                time.sleep(0.1)
            self.assertGreaterEqual(phone.fpga_est_count, 100)
            est = phone.fpga_est
            self.assertEqual(est["backend"], "python-model")
            self.assertEqual(dash.fpga.status()["estimates_returned"], dash.fpga.returned)
            # the estimate the phone got is exactly what the dashboard shows for that frame
            with dash.telem.lock:
                shown = {f["seq"]: f for _, f in dash.telem.frames}
            f = shown.get(est["seq"])
            self.assertIsNotNone(f, "frame for the returned estimate not in the dashboard ring")
            for k in ("v_x", "v_y", "sigma_vx", "distance", "net_forward"):
                self.assertEqual(est[k], f[k], k)
            self.assertNotEqual(est["v_x"], f["phone"]["v_x"])   # it is the FPGA's, not the phone's
        finally:
            phone.running = False
            dash.stop()
            tmp.cleanup()

    def test_no_estimates_without_fpga(self):
        tmp, dash, phone = self.start(fpga=False)
        try:
            time.sleep(1.5)
            self.assertGreater(dash.telem.next_idx, 30)        # frames arrive ...
            self.assertEqual(phone.fpga_est_count, 0)          # ... but nothing is sent back
        finally:
            phone.running = False
            dash.stop()
            tmp.cleanup()


class TestEstimateProtocol(unittest.TestCase):
    def test_roundtrip_and_rejects(self):
        f = dict(seq=7, t=12.5, v_x=1.25, v_y=-0.5, sigma_vx=0.02, sigma_vy=0.03, distance=3.0,
                 net_forward=2.5, status=P.IMU_OK | P.FILTER_INIT)
        data = P.encode_fpga_estimate(f, "rtl-iverilog")
        est = P.decode_fpga_estimate(data)
        self.assertEqual(est["type"], "fpga_est")
        self.assertEqual((est["seq"], est["v_x"], est["status"], est["backend"]),
                         (7, 1.25, P.IMU_OK | P.FILTER_INIT, "rtl-iverilog"))
        self.assertIsNone(P.encode_fpga_estimate(dict(f, v_x=float("nan")), "x"))   # never send NaN
        self.assertIsNone(P.decode_fpga_estimate(b'{"seq":1,"t":2}'))               # a JSON frame
        self.assertIsNone(P.decode_fpga_estimate(b"\xa5\x02rest"))                   # a binary frame
        self.assertIsNone(P.decode_fpga_estimate(b"{not json"))


REPO = Path(__file__).resolve().parents[2]


def csv_slice(path, t_max=None, max_rows=None):
    """Header + the rows of a recorded CSV up to time t_max (first column)."""
    lines = path.read_text().splitlines()
    out = [lines[0]]
    for ln in lines[1:]:
        if (t_max is not None and float(ln.split(",")[0]) > t_max) or (max_rows and len(out) > max_rows):
            break
        out.append(ln)
    return "\n".join(out) + "\n"


class TestUploadReplay(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        args = server.build_arg_parser().parse_args([
            "--udp-host", "127.0.0.1", "--udp-port", "0", "--http-port", "0", "--cmd-port", str(free_port()),
            "--log-dir", self.tmp.name, "--no-bonjour", "--fpga-backend", "model"])
        self.dash = server.Dashboard(args)
        self.dash.start()
        self.base = f"http://127.0.0.1:{self.dash.http_port}"

    def tearDown(self):
        self.dash.stop()
        self.tmp.cleanup()

    def post(self, path, obj):
        req = urllib.request.Request(self.base + path, data=json.dumps(obj).encode(),
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            return json.loads(e.read())

    def status(self):
        with urllib.request.urlopen(self.base + "/api/status", timeout=5) as r:
            return json.loads(r.read())

    def wait_done(self, timeout=60):
        end = time.time() + timeout
        while time.time() < end:
            rp = self.status()["replay"]
            if rp and rp["done"]:
                return rp
            time.sleep(0.2)
        self.fail("replay did not finish")

    def test_phone_run_upload_matches_the_float_filter(self):
        run = REPO / "data" / "phone_runs" / "2026-10-03_22-22-18"
        t_end = float(run.joinpath("imu.csv").read_text().splitlines()[301].split(",")[0])
        files = {"imu.csv": csv_slice(run / "imu.csv", t_end), "flow.csv": csv_slice(run / "flow.csv", t_end),
                 "gnss.csv": csv_slice(run / "gnss.csv"), "events.csv": csv_slice(run / "events.csv", t_end),
                 "meta.json": (run / "meta.json").read_text()}
        rep = self.post("/api/replay", {"name": "run1", "files": files, "speed": 1000})
        self.assertTrue(rep["ok"], rep)
        rp = self.wait_done()
        self.assertEqual(rp["error"], "")
        self.assertEqual(rp["progress"], 1.0)
        self.assertGreater(rp["flow_accepted"], 100)
        lf = self.status()["last_frame"]
        self.assertFalse(lf["status"] & P.RECORDING)             # the run ended
        # the same samples through the float Python filter
        r = fpga_filter.kf_ref.load_run(run)
        imu = [x for x in r["imu"] if x["t"] <= t_end]
        flow = [x for x in r["flow"] if x["t"] <= t_end]
        ev = [e for e in r["events"] if e["t"] <= t_end]
        ref = fpga_filter.kf_ref.ReferenceKF4(fpga_filter.kf_ref.FilterConfig.from_meta(r["meta"]))
        fpga_filter.kf_ref.replay(ref, imu, flow, [], ev, seed=fpga_filter.kf_ref.parse_filter_state(r["events"]))
        self.assertAlmostEqual(lf["v_x"], ref.state.vx, delta=1e-6)

    def test_dashboard_log_upload(self):
        log = REPO / "data" / "dashboard_logs" / "2026-10-04_01-57-54" / "telemetry.csv"
        rep = self.post("/api/replay", {"name": "log", "speed": 1000,
                                        "files": {"telemetry.csv": csv_slice(log, max_rows=250)}})
        self.assertTrue(rep["ok"], rep)
        rp = self.wait_done()
        self.assertEqual(rp["error"], "")
        self.assertEqual(rp["frames"], 250)
        self.assertIn("phone", self.status()["last_frame"])

    def test_bad_upload_is_refused_with_a_reason(self):
        rep = self.post("/api/replay", {"files": {"notes.txt": "hello"}})
        self.assertFalse(rep["ok"])
        self.assertIn("imu.csv", rep["error"])


if __name__ == "__main__":
    unittest.main()
