"""The Kalman filter on the simulated FPGA, inside the dashboard."""
import json
import socket
import sys
import tempfile
import threading
import time
import unittest
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


if __name__ == "__main__":
    unittest.main()
