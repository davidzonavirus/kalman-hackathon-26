import json
import os
import re
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gsdash import protocol as P  # noqa: E402

GOLDEN_MD = Path(__file__).resolve().parents[2] / "docs" / "golden_frame.md"
F32 = lambda x: struct.unpack("<f", struct.pack("<f", x))[0]  # noqa: E731

SAMPLE = dict(seq=42, t=1234.5, v_x=1.25, v_y=-0.03, sigma_vx=0.05, sigma_vy=0.06,
              distance=3.5, flow_quality=12.0, h=0.3,
              status=P.IMU_OK | P.FLOW_OK | P.FILTER_INIT, battery=87, torch=60)


def load_golden():
    if not GOLDEN_MD.exists():
        return None, None
    text = GOLDEN_MD.read_text()
    hexes = re.findall(r"\b([0-9a-fA-F]{96})\b", text)
    js = re.search(r"```json\s*\n(\{.*?\})\s*\n```", text, re.S)
    return (bytes.fromhex(hexes[0]) if hexes else None), (js.group(1) if js else None)


class TestCRC(unittest.TestCase):
    def test_check_value(self):
        self.assertEqual(P.crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_empty(self):
        self.assertEqual(P.crc16_ccitt_false(b""), 0xFFFF)


class TestFrames(unittest.TestCase):
    def test_layout(self):
        raw = P.encode_frame(**SAMPLE)
        self.assertEqual(len(raw), 48)
        self.assertEqual(raw[0], 0xA5)
        self.assertEqual(raw[1], 1)
        self.assertEqual(struct.unpack_from("<I", raw, 2)[0], 42)
        self.assertEqual(struct.unpack_from("<d", raw, 6)[0], 1234.5)
        self.assertEqual(struct.unpack_from("<H", raw, 42)[0], 0x0203)
        self.assertEqual(raw[44], 87)
        self.assertEqual(raw[45], 60)
        self.assertEqual(struct.unpack_from("<H", raw, 46)[0], P.crc16_ccitt_false(raw[:46]))

    def test_golden(self):
        raw, js = load_golden()
        if raw is None:
            self.skipTest("docs/golden_frame.md not present")
        # our encoder must reproduce the Swift golden bytes exactly
        self.assertEqual(P.encode_frame(**SAMPLE).hex(), raw.hex())
        f = P.decode_frame(raw)
        self.assertEqual(f["seq"], 42)
        self.assertEqual(f["t"], 1234.5)
        self.assertEqual(f["v_x"], 1.25)
        self.assertEqual(f["v_y"], F32(-0.03))
        self.assertEqual(f["h"], F32(0.3))
        self.assertEqual(f["status"], 515)
        self.assertEqual(f["battery"], 87)
        self.assertEqual(f["torch"], 60)
        self.assertEqual(f["fmt"], "bin")
        self.assertEqual(struct.unpack_from("<H", raw, 46)[0], 0xCD3C)
        if js:
            fj = P.decode_frame(js.encode())
            self.assertEqual(fj["fmt"], "json")
            for k in P.FIELDS:
                self.assertAlmostEqual(fj[k], f[k], places=6, msg=k)

    def test_roundtrip(self):
        raw = P.encode_frame(**SAMPLE)
        f = P.decode_frame(raw)
        for k, v in SAMPLE.items():
            if isinstance(v, float):
                self.assertEqual(f[k], F32(v) if k != "t" else v, k)
            else:
                self.assertEqual(f[k], v, k)

    def test_bad_crc(self):
        raw = bytearray(P.encode_frame(**SAMPLE))
        raw[20] ^= 0x01
        with self.assertRaises(P.FrameError) as cm:
            P.decode_frame(bytes(raw))
        self.assertEqual(cm.exception.kind, "crc")

    def test_bad_magic_version_length(self):
        raw = P.encode_frame(**SAMPLE)
        for bad in (b"\x00" + raw[1:], raw[:1] + b"\x02" + raw[2:], raw[:47], raw + b"\x00"):
            with self.assertRaises(P.FrameError) as cm:
                P.decode_frame(bad)
            self.assertEqual(cm.exception.kind, "decode")
        with self.assertRaises(P.FrameError):
            P.decode_frame(b"")

    def test_json(self):
        raw = P.encode_json_frame(**SAMPLE)
        self.assertEqual(raw[:1], b"{")
        f = P.decode_frame(raw)
        self.assertEqual(f["fmt"], "json")
        for k, v in SAMPLE.items():
            self.assertEqual(f[k], v, k)
        # any key order, extra whitespace
        obj = dict(reversed(list(SAMPLE.items())))
        self.assertEqual(P.decode_frame(json.dumps(obj, indent=1).encode())["seq"], 42)

    def test_json_bad(self):
        for bad in (b"{not json", b"{}", b'{"seq":1}', b"[1,2]"):
            with self.assertRaises(P.FrameError):
                P.decode_frame(bad)

    def test_status_names(self):
        self.assertEqual(P.status_names(515), ["IMU_OK", "FLOW_OK", "FILTER_INIT"])

    def test_command(self):
        line = P.encode_command("set_torch", level=0.6)
        self.assertTrue(line.endswith(b"\n"))
        self.assertEqual(json.loads(line), {"cmd": "set_torch", "level": 0.6})
        self.assertEqual(P.decode_line(b'{"ok":true,"cmd":"ping"}\n')["ok"], True)


class TestSeqTracker(unittest.TestCase):
    def feed(self, seqs, ts=None):
        tr = P.SeqTracker()
        ts = ts or [float(i) for i in range(len(seqs))]
        out = [tr.update(s, t) for s, t in zip(seqs, ts)]
        return tr, out

    def test_contiguous(self):
        tr, out = self.feed(list(range(100)))
        self.assertEqual((tr.lost, tr.out_of_order, tr.resets), (0, 0, 0))

    def test_gap(self):
        tr, out = self.feed([1, 2, 3, 7, 8])
        self.assertEqual(tr.lost, 3)
        self.assertEqual(out[3], "gap")

    def test_wrap(self):
        seqs = [0xFFFFFFFE, 0xFFFFFFFF, 0, 1, 3]
        tr, out = self.feed(seqs)
        self.assertEqual(tr.lost, 1)
        self.assertEqual(tr.resets, 0)

    def test_out_of_order(self):
        # 3 arrives after 4 with an earlier phone timestamp
        tr, out = self.feed([1, 2, 4, 3, 5], [1, 2, 4, 3, 5])
        self.assertEqual(out[3], "ooo")
        self.assertEqual(tr.out_of_order, 1)
        self.assertEqual(tr.lost, 0)

    def test_duplicate(self):
        tr, out = self.feed([1, 2, 2, 3])
        self.assertEqual(tr.duplicates, 1)

    def test_phone_restart_small_seq(self):
        # app restarts: seq drops back to 0 while phone mach time keeps increasing
        tr, out = self.feed([500, 501, 0, 1, 2], [10, 10.02, 15, 15.02, 15.04])
        self.assertEqual(out[2], "reset")
        self.assertEqual((tr.lost, tr.out_of_order, tr.resets), (0, 0, 1))

    def test_restart_with_clock_reset(self):
        # sender restarts within 20 s and its clock restarts too: seq and t both go back
        seqs = list(range(100, 110)) + list(range(0, 10))
        ts = [50 + i * .02 for i in range(10)] + [1 + i * .02 for i in range(10)]
        tr, out = self.feed(seqs, ts)
        self.assertEqual(tr.resets, 1)
        self.assertEqual(tr.out_of_order, 0)
        self.assertEqual(tr.lost, 0)
        self.assertEqual(out[-1], "ok")

    def test_huge_jump_is_reset(self):
        tr, out = self.feed([10, 11, 900000, 900001])
        self.assertEqual(out[2], "reset")
        self.assertEqual(tr.lost, 0)


if __name__ == "__main__":
    unittest.main()
