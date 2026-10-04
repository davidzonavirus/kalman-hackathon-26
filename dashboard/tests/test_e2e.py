"""End-to-end: dashboard server + simulated phone on random local ports."""
import csv
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
from gsdash import protocol as P  # noqa: E402
from gsdash import server, sim  # noqa: E402


def free_port(kind=socket.SOCK_STREAM):
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def wait_for(pred, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        v = pred()
        if v:
            return v
        time.sleep(0.05)
    return pred()


class TestEndToEnd(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cmd_port = free_port()
        args = server.build_arg_parser().parse_args([
            "--udp-host", "127.0.0.1", "--udp-port", "0", "--http-port", "0",
            "--cmd-port", str(self.cmd_port), "--log-dir", self.tmp.name, "--no-bonjour"])
        self.dash = server.Dashboard(args)
        self.dash.start()
        self.base = f"http://127.0.0.1:{self.dash.http_port}"
        self.phone = sim.FakePhone("127.0.0.1", self.dash.udp_port, rate=100.0)
        self.tcp = sim.make_tcp_server(self.phone, "127.0.0.1", self.cmd_port)
        threading.Thread(target=self.tcp.serve_forever, daemon=True).start()
        threading.Thread(target=self.phone.run, daemon=True).start()

    def tearDown(self):
        self.phone.running = False
        self.tcp.shutdown()
        self.tcp.server_close()
        self.dash.stop()
        self.tmp.cleanup()

    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=5) as r:
            return r.status, r.headers, r.read()

    def post(self, path, obj):
        req = urllib.request.Request(self.base + path, data=json.dumps(obj).encode(),
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.loads(r.read())

    def test_flow(self):
        # frames arrive and are counted
        self.assertTrue(wait_for(lambda: self.dash.telem.seq.received >= 20))
        st = json.loads(self.get("/api/status")[2])
        self.assertEqual(st["link"]["source_ip"], "127.0.0.1")
        self.assertEqual(st["link"]["crc_errors"], 0)
        self.assertEqual(st["link"]["lost"], 0)

        # UI is served
        code, hdr, body = self.get("/")
        self.assertEqual(code, 200)
        self.assertIn(b"<canvas", body)

        # commands round-trip through the persistent TCP link (target = UDP source IP)
        rep = self.post("/api/cmd", {"cmd": "ping"})
        self.assertTrue(rep["ok"], rep)
        rep = self.post("/api/cmd", {"cmd": "start_run", "label": "e2e"})
        self.assertTrue(rep["ok"], rep)
        self.assertTrue(rep["run"].endswith("_e2e"))
        self.assertTrue(wait_for(lambda: self.dash.telem.last_frame["status"] & P.RECORDING))
        rep = self.post("/api/cmd", {"cmd": "set_torch", "level": 0.6})
        self.assertTrue(rep["ok"], rep)
        # v2 telemetry by default, and zero resets distance + net_forward
        f = self.dash.telem.last_frame
        self.assertEqual((f["version"], f["fmt"]), (2, "bin"))
        self.assertIsNotNone(f["net_forward"])
        rep = self.post("/api/cmd", {"cmd": "zero"})
        self.assertTrue(rep["ok"], rep)
        self.assertEqual(rep["cmd"], "zero")
        self.assertTrue(wait_for(lambda: self.dash.telem.last_frame["seq"] > f["seq"] + 2))
        self.assertLess(abs(self.dash.telem.last_frame["net_forward"]), 0.2)
        rep = self.post("/api/cmd", {"cmd": "bogus"})
        self.assertFalse(rep["ok"])
        self.assertNotIn("local", rep)  # error came from the phone, not the transport
        rep = self.post("/api/cmd", {"cmd": "stop_run"})
        self.assertTrue(rep["ok"], rep)

        # CRC errors / garbage are counted, not fatal
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        bad = bytearray(P.encode_frame(**dict(seq=1, t=1.0, v_x=0, v_y=0, sigma_vx=0,
                                              sigma_vy=0, distance=0, flow_quality=0,
                                              h=0.3, status=0)))
        bad[10] ^= 0xFF
        s.sendto(bytes(bad), ("127.0.0.1", self.dash.udp_port))
        s.sendto(b"garbage", ("127.0.0.1", self.dash.udp_port))
        s.close()
        self.assertTrue(wait_for(lambda: self.dash.telem.crc_errors == 1
                                 and self.dash.telem.decode_errors == 1))

        # SSE stream delivers frames and stats
        r = urllib.request.urlopen(self.base + "/events", timeout=5)
        got = set()
        end = time.time() + 3
        while time.time() < end and got != {"history", "frames", "stats"}:
            line = r.readline().decode()
            if line.startswith("event: "):
                got.add(line[7:].strip())
            elif line.startswith("data: ") and "frames" in got:
                pass
        r.close()
        self.assertEqual(got, {"history", "frames", "stats"})

        # session CSV
        self.dash.logger.close()
        path = self.dash.logger.dir / "telemetry.csv"
        with open(path) as f:
            rows = list(csv.reader(f))
        self.assertEqual(rows[0], list(P.LOG_CSV_FIELDS))
        self.assertEqual(rows[0][:9], "t,v_x,v_y,sigma_vx,sigma_vy,distance,status,flow_quality,h".split(","))
        self.assertGreater(len(rows), 20)
        hdr = rows[0]
        self.assertEqual(rows[1][hdr.index("version")], "2")
        self.assertNotEqual(rows[1][hdr.index("net_forward")], "")

    def test_no_phone_reply_is_local_error(self):
        self.tcp.shutdown()
        self.tcp.server_close()
        self.dash.phone.set_explicit("127.0.0.1")
        self.dash.phone.port = free_port()  # nothing listens here
        rep = self.post("/api/cmd", {"cmd": "ping"})
        self.assertFalse(rep["ok"])
        self.assertTrue(rep.get("local"))


class TestJsonAndLoss(unittest.TestCase):
    def test_json_frames_with_loss(self):
        tmp = tempfile.TemporaryDirectory()
        args = server.build_arg_parser().parse_args([
            "--udp-host", "127.0.0.1", "--udp-port", "0", "--http-port", "0",
            "--cmd-port", str(free_port()), "--no-log", "--no-bonjour"])
        dash = server.Dashboard(args)
        dash.start()
        phone = sim.FakePhone("127.0.0.1", dash.udp_port, rate=200.0, json_mode=True,
                              loss=0.2, seq_start=0xFFFFFFFF - 50, version=1)
        threading.Thread(target=phone.run, daemon=True).start()
        try:
            self.assertTrue(wait_for(lambda: dash.telem.seq.received >= 200, 8))
            phone.running = False
            time.sleep(0.1)
            st = dash.telem.stats()
            self.assertEqual(st["fmt"], "json")
            self.assertEqual(dash.telem.last_frame["version"], 1)
            self.assertIsNone(dash.telem.last_frame["net_forward"])
            self.assertEqual(st["seq_resets"], 0)          # crossed the u32 wrap cleanly
            # every drop the sim made between first and last received frame is a gap
            self.assertGreater(st["lost"], 0)
            self.assertLessEqual(abs(st["lost"] - phone.dropped), 2)
        finally:
            phone.running = False
            dash.stop()
            tmp.cleanup()


if __name__ == "__main__":
    unittest.main()
