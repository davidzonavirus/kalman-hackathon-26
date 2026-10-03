# Golden telemetry frame (test vector)

Produced by `SpeedProtocol.TelemetryFrame.encodeBinary()` and independently
cross-checked with Python `struct.pack('<BBIdfffffffHBB', …)` + a bitwise
CRC-16/CCITT-FALSE. Use it to validate any decoder.

## Input

| Field | Value |
|---|---|
| seq | 42 |
| t | 1234.5 |
| v_x | 1.25 |
| v_y | -0.03 |
| sigma_vx | 0.05 |
| sigma_vy | 0.06 |
| distance | 3.5 |
| flow_quality | 12.0 |
| h | 0.3 |
| status | 0x0203 (515) = IMU_OK \| FLOW_OK \| FILTER_INIT |
| battery | 87 |
| torch | 60 |

(f32 fields are rounded to float32, so e.g. v_y decodes as -0.029999999329447746.)

## Binary (48 bytes, little-endian)

Contiguous hex:

```
a5012a00000000000000004a93400000a03f8fc2f5bccdcc4c3d8fc2753d00006040000040419a99993e0302573c3ccd
```

By field:

| Offset | Bytes | Field |
|---|---|---|
| 0 | `a5` | magic |
| 1 | `01` | version |
| 2 | `2a 00 00 00` | seq = 42 |
| 6 | `00 00 00 00 00 4a 93 40` | t = 1234.5 (f64) |
| 14 | `00 00 a0 3f` | v_x = 1.25 |
| 18 | `8f c2 f5 bc` | v_y = -0.03 |
| 22 | `cd cc 4c 3d` | sigma_vx = 0.05 |
| 26 | `8f c2 75 3d` | sigma_vy = 0.06 |
| 30 | `00 00 60 40` | distance = 3.5 |
| 34 | `00 00 40 41` | flow_quality = 12.0 |
| 38 | `9a 99 99 3e` | h = 0.3 |
| 42 | `03 02` | status = 0x0203 |
| 44 | `57` | battery = 87 |
| 45 | `3c` | torch = 60 |
| 46 | `3c cd` | crc = 0xCD3C (CRC-16/CCITT-FALSE over bytes 0..45) |

CRC sanity: CRC-16/CCITT-FALSE("123456789") = 0x29B1.

Python:

```python
import struct
raw = bytes.fromhex("a5012a00000000000000004a93400000a03f8fc2f5bccdcc4c3d8fc2753d00006040000040419a99993e0302573c3ccd")
fields = struct.unpack('<BBIdfffffffHBBH', raw)
```

## JSON debug form of the same frame

Swift `encodeJSON()` output (keys sorted; Python should accept any key order):

```json
{"battery":87,"distance":3.5,"flow_quality":12,"h":0.3,"seq":42,"sigma_vx":0.05,"sigma_vy":0.06,"status":515,"t":1234.5,"torch":60,"v_x":1.25,"v_y":-0.03}
```
