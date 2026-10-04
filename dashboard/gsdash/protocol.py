"""Wire protocol (docs/PROTOCOL.md): v1 (48 B) and v2 (72 B) binary frames, JSON debug
frames, commands.

Standard library only.
"""
from __future__ import annotations

import json
import math
import struct

MAGIC = 0xA5
VERSION = 2                 # what the phone sends by default
FRAME_LEN_V1 = 48
FRAME_LEN_V2 = 72
FRAME_LEN = FRAME_LEN_V1    # backwards-compatible alias
FRAME_LENS = {1: FRAME_LEN_V1, 2: FRAME_LEN_V2}

# < little-endian, no padding: magic u8, version u8, seq u32, t f64,
# v_x v_y sigma_vx sigma_vy distance flow_quality h  (7 x f32), status u16,
# battery u8, torch u8   -> 46 bytes; v2 then adds 6 x f32 -> 70 bytes; then crc u16.
_BODY_V1 = struct.Struct("<BBIdfffffffHBB")
_BODY_V2 = struct.Struct("<BBIdfffffffHBBffffff")
_BODY = _BODY_V1
_CRC = struct.Struct("<H")
assert _BODY_V1.size == 46 and _BODY_V2.size == 70

FIELDS_V1 = ("seq", "t", "v_x", "v_y", "sigma_vx", "sigma_vy", "distance",
             "flow_quality", "h", "status", "battery", "torch")
V2_EXTRA = ("ax", "ay", "gz", "flow_vx", "flow_vy", "net_forward")
FIELDS_V2 = FIELDS_V1 + V2_EXTRA
FIELDS = FIELDS_V1
_FLOAT_FIELDS = ("t", "v_x", "v_y", "sigma_vx", "sigma_vy", "distance", "flow_quality",
                 "h") + V2_EXTRA

# est.csv header (PROTOCOL.md section 3); the dashboard log adds the v2 columns
# (empty for v1 frames) and seq, recv_time, version.
EST_CSV_FIELDS = ("t", "v_x", "v_y", "sigma_vx", "sigma_vy", "distance", "status",
                  "flow_quality", "h")
LOG_CSV_FIELDS = EST_CSV_FIELDS + V2_EXTRA + ("seq", "recv_time", "version")

# Status bit flags
IMU_OK = 1 << 0
FLOW_OK = 1 << 1
GNSS_OK = 1 << 2
LIDAR_OK = 1 << 3
RECORDING = 1 << 4
ZUPT = 1 << 5
FLOW_GATED = 1 << 6
GNSS_GATED = 1 << 7
TORCH_ON = 1 << 8
FILTER_INIT = 1 << 9
CALIBRATING = 1 << 10

STATUS_BITS = {
    "IMU_OK": IMU_OK, "FLOW_OK": FLOW_OK, "GNSS_OK": GNSS_OK, "LIDAR_OK": LIDAR_OK,
    "RECORDING": RECORDING, "ZUPT": ZUPT, "FLOW_GATED": FLOW_GATED,
    "GNSS_GATED": GNSS_GATED, "TORCH_ON": TORCH_ON, "FILTER_INIT": FILTER_INIT,
    "CALIBRATING": CALIBRATING,
}

COMMANDS = ("ping", "start_run", "stop_run", "mark", "calibrate", "set_torch",
            "reset_distance", "zero")


class FrameError(ValueError):
    """Undecodable datagram. ``kind`` is 'crc' for checksum failures, else 'decode'."""

    def __init__(self, msg: str, kind: str = "decode"):
        super().__init__(msg)
        self.kind = kind


def crc16_ccitt_false(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def status_names(status: int) -> list[str]:
    return [name for name, bit in STATUS_BITS.items() if status & bit]


def _pick_version(version, extra) -> int:
    if version is None:
        return 2 if any(v is not None for v in extra) else 1
    if version not in (1, 2):
        raise ValueError(f"unsupported version {version}")
    return version


def encode_frame(seq: int, t: float, v_x: float, v_y: float, sigma_vx: float,
                 sigma_vy: float, distance: float, flow_quality: float, h: float,
                 status: int, battery: int = 0xFF, torch: int = 0,
                 ax: float | None = None, ay: float | None = None, gz: float | None = None,
                 flow_vx: float | None = None, flow_vy: float | None = None,
                 net_forward: float | None = None, version: int | None = None) -> bytes:
    """Encode a binary frame (tests and the simulator).

    version=None picks v2 when any v2 field is given, else v1 (48 B). Missing v2 fields
    encode as 0.
    """
    extra = (ax, ay, gz, flow_vx, flow_vy, net_forward)
    ver = _pick_version(version, extra)
    common = (MAGIC, ver, seq & 0xFFFFFFFF, t, v_x, v_y, sigma_vx, sigma_vy, distance,
              flow_quality, h, status & 0xFFFF, battery & 0xFF, torch & 0xFF)
    if ver == 1:
        body = _BODY_V1.pack(*common)
    else:
        body = _BODY_V2.pack(*common, *(0.0 if v is None else v for v in extra))
    return body + _CRC.pack(crc16_ccitt_false(body))


def encode_json_frame(seq: int, t: float, v_x: float, v_y: float, sigma_vx: float,
                      sigma_vy: float, distance: float, flow_quality: float, h: float,
                      status: int, battery: int = 0xFF, torch: int = 0,
                      ax: float | None = None, ay: float | None = None,
                      gz: float | None = None, flow_vx: float | None = None,
                      flow_vy: float | None = None, net_forward: float | None = None,
                      version: int | None = None) -> bytes:
    extra = (ax, ay, gz, flow_vx, flow_vy, net_forward)
    ver = _pick_version(version, extra)
    obj = {"seq": seq & 0xFFFFFFFF, "t": t, "v_x": v_x, "v_y": v_y,
           "sigma_vx": sigma_vx, "sigma_vy": sigma_vy, "distance": distance,
           "flow_quality": flow_quality, "h": h, "status": status & 0xFFFF,
           "battery": battery, "torch": torch}
    if ver == 2:
        obj["version"] = 2
        for k, v in zip(V2_EXTRA, extra):
            obj[k] = 0.0 if v is None else v
    return json.dumps(obj, separators=(",", ":")).encode()


def _decode_binary(data: bytes) -> dict:
    if len(data) < 2:
        raise FrameError(f"bad length {len(data)}")
    if data[0] != MAGIC:
        raise FrameError(f"bad magic 0x{data[0]:02X}")
    ver = data[1]
    if ver not in FRAME_LENS:
        raise FrameError(f"unsupported version {ver}")
    want_len = FRAME_LENS[ver]
    if len(data) != want_len:
        raise FrameError(f"bad length {len(data)} for v{ver} (want {want_len})")
    body = _BODY_V1 if ver == 1 else _BODY_V2
    (want,) = _CRC.unpack_from(data, body.size)
    got = crc16_ccitt_false(data[:body.size])
    if want != got:
        raise FrameError(f"crc mismatch: frame 0x{want:04X} computed 0x{got:04X}", "crc")
    vals = body.unpack_from(data, 0)[2:]
    frame = dict(zip(FIELDS_V1 if ver == 1 else FIELDS_V2, vals))
    if ver == 1:
        frame.update({k: None for k in V2_EXTRA})
    frame["version"] = ver
    frame["fmt"] = "bin"
    return frame


def _decode_json(data: bytes) -> dict:
    try:
        obj = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as e:
        raise FrameError(f"bad json: {e}") from None
    if not isinstance(obj, dict):
        raise FrameError("json frame is not an object")
    ver = obj.get("version")
    if ver is None:
        ver = 2 if all(k in obj for k in V2_EXTRA) else 1
    if ver not in (1, 2):
        raise FrameError(f"unsupported version {ver!r}")
    need = FIELDS_V1 if ver == 1 else FIELDS_V2
    missing = [k for k in need if k not in obj]
    if missing:
        raise FrameError(f"json v{ver} frame missing keys: {','.join(missing)}")
    try:
        frame = {
            "seq": int(obj["seq"]) & 0xFFFFFFFF,
            "t": float(obj["t"]),
            "status": int(obj["status"]) & 0xFFFF,
            "battery": int(obj["battery"]),
            "torch": int(round(float(obj["torch"]))),
        }
        for k in ("v_x", "v_y", "sigma_vx", "sigma_vy", "distance", "flow_quality", "h"):
            frame[k] = float(obj[k])
        for k in V2_EXTRA:
            frame[k] = float(obj[k]) if ver == 2 and obj[k] is not None else None
    except (TypeError, ValueError) as e:
        raise FrameError(f"bad json field: {e}") from None
    frame["version"] = ver
    frame["fmt"] = "json"
    return frame


def decode_frame(data: bytes) -> dict:
    """Decode a telemetry datagram: v1/v2 binary, or JSON (first byte '{').

    Returns a dict with keys FIELDS_V2 + 'version' + 'fmt' (v2-only fields are None for
    v1 frames). Raises FrameError.
    """
    if not data:
        raise FrameError("empty datagram")
    frame = _decode_json(data) if data[:1] == b"{" else _decode_binary(data)
    # NaN/inf are legal floats on the wire but break JSON for the browser.
    for k in _FLOAT_FIELDS:
        v = frame.get(k)
        if v is not None and not math.isfinite(v):
            frame[k] = None
    return frame


def encode_command(cmd: str, **kw) -> bytes:
    """One newline-terminated JSON command line, e.g. encode_command('set_torch', level=0.6)."""
    obj = {"cmd": cmd}
    obj.update({k: v for k, v in kw.items() if v is not None})
    return (json.dumps(obj, separators=(",", ":")) + "\n").encode()


def decode_line(line: bytes | str) -> dict:
    if isinstance(line, bytes):
        line = line.decode("utf-8")
    obj = json.loads(line)
    if not isinstance(obj, dict):
        raise ValueError("not a JSON object")
    return obj


class SeqTracker:
    """Sequence-number bookkeeping for a 50 Hz UDP stream with u32 wrap.

    * forward jump 1          -> normal
    * forward jump 2..MAX_GAP -> (jump-1) frames counted lost
    * backward, phone time t also went back (or no t) and within MAX_GAP
                              -> out-of-order (a late frame; one 'lost' is recovered)
    * duplicate               -> counted
    * anything else (huge jump, or seq went back while t moved forward)
                              -> phone restarted / reset its counter: resync, count reset
    """

    MAX_GAP = 1000  # 20 s at 50 Hz
    OOO_RESET_RUN = 5

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_seq = None
        self.last_t = None
        self.received = 0
        self.lost = 0
        self.out_of_order = 0
        self.duplicates = 0
        self.resets = 0
        self._ooo_run = 0
        self._ooo_recovered = 0

    def update(self, seq: int, t: float | None = None) -> str:
        self.received += 1
        seq &= 0xFFFFFFFF
        if self.last_seq is None:
            self.last_seq, self.last_t = seq, t
            return "first"
        fwd = (seq - self.last_seq) & 0xFFFFFFFF
        back = (self.last_seq - seq) & 0xFFFFFFFF
        if fwd == 0:
            self.duplicates += 1
            return "dup"
        if fwd <= self.MAX_GAP:
            self._ooo_run = self._ooo_recovered = 0
            self.lost += fwd - 1
            self.last_seq, self.last_t = seq, t
            return "ok" if fwd == 1 else "gap"
        time_went_back = (t is None or self.last_t is None or t <= self.last_t)
        if back <= self.MAX_GAP and time_went_back:
            # a run of consecutive "late" frames is really a sender restart whose
            # clock also restarted (e.g. a simulator): undo and resync.
            self._ooo_run += 1
            if self._ooo_run >= self.OOO_RESET_RUN:
                self.out_of_order -= self._ooo_run - 1
                self.lost += self._ooo_recovered
                self._ooo_run = self._ooo_recovered = 0
                self.resets += 1
                self.last_seq, self.last_t = seq, t
                return "reset"
            self.out_of_order += 1
            if self.lost > 0:
                self.lost -= 1
                self._ooo_recovered += 1
            return "ooo"
        self._ooo_run = self._ooo_recovered = 0
        self.resets += 1
        self.last_seq, self.last_t = seq, t
        return "reset"
