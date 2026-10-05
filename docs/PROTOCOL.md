# Wire protocol and file formats

This document is the contract between the phone (`GroundSpeedKit`, iOS app, `phone-sim`) and
the dashboard (`dashboard/gsdash`). Field names are identical in UDP frames, CSV headers,
Swift and Python. Times are seconds on the phone's monotonic host clock (mach time). SI units
throughout: m, m/s, m/s², rad/s, unless stated otherwise.

Vehicle frame: **x forward, y left, z up**. `r`/`gz` is the yaw rate about z, counter-clockwise
positive.

## 1. Telemetry: phone to dashboard (UDP 9000, 50 Hz)

The dashboard listens on UDP 9000 on all interfaces (IPv4 and IPv6). The phone sends to its
configured dashboard address. When a dashboard connects on the command port (section 2), the
phone switches its UDP destination to that dashboard.

### Binary frame, version 2 (default): 72 bytes, little-endian

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | u8 | magic | `0xA5` |
| 1 | u8 | version | `2` |
| 2 | u32 | seq | 0 at app launch, +1 per frame, wraps at 2³². A backwards jump means the app restarted |
| 6 | f64 | t | phone host clock, s |
| 14 | f32 | v_x | forward velocity, m/s |
| 18 | f32 | v_y | lateral velocity, m/s |
| 22 | f32 | sigma_vx | 1σ of v_x, m/s |
| 26 | f32 | sigma_vy | 1σ of v_y, m/s |
| 30 | f32 | distance | total distance travelled, ∫\|v\|dt since the last zero or run start, m |
| 34 | f32 | flow_quality | peak-to-sidelobe ratio (PSR) of the last flow sample, 0 if none |
| 38 | f32 | h | camera height above the ground in use, m |
| 42 | u16 | status | bit flags, below |
| 44 | u8 | battery | percent 0–100, `0xFF` unknown |
| 45 | u8 | torch | torch level × 100 (0–100) |
| 46 | f32 | ax | forward acceleration, gravity removed, bias-corrected, m/s² |
| 50 | f32 | ay | lateral acceleration, m/s² |
| 54 | f32 | gz | yaw rate, rad/s |
| 58 | f32 | flow_vx | last raw optical-flow velocity, x, m/s |
| 62 | f32 | flow_vy | last raw optical-flow velocity, y, m/s |
| 66 | f32 | net_forward | signed ∫v_x dt since the last zero, m |
| 70 | u16 | crc | CRC-16/CCITT-FALSE over bytes 0–69 |

CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, no reflection, no final XOR. Check value:
`"123456789"` gives `0x29B1`.

**Version 1** (48 bytes) is the same layout up to offset 45, with the CRC over bytes 0–45 at
offset 46 and no fields after it. Receivers accept both; in a v1 frame the v2 fields are absent.

### Status flags (u16)

| Bit | Name | Set when |
|---|---|---|
| 0 | IMU_OK | IMU sample less than 50 ms old |
| 1 | FLOW_OK | flow sample less than 100 ms old, quality above threshold, and accepted by the filter |
| 2 | GNSS_OK | GNSS fix less than 2 s old with a valid speed accuracy |
| 3 | LIDAR_OK | LiDAR depth less than 500 ms old |
| 4 | RECORDING | a run is being recorded |
| 5 | ZUPT | a zero-velocity update was applied in the last 100 ms |
| 6 | FLOW_GATED | the last flow sample was rejected by the innovation gate |
| 7 | GNSS_GATED | the last GNSS sample was rejected by the innovation gate |
| 8 | TORCH_ON | torch is on |
| 9 | FILTER_INIT | filter initialised and running |
| 10 | CALIBRATING | IMU bias calibration in progress |
| 11–15 | reserved | 0 |

### JSON frame (debug)

A datagram whose first byte is `{` is a UTF-8 JSON object with the same field names:
`seq, t, v_x, v_y, sigma_vx, sigma_vy, distance, flow_quality, h, status, battery, torch`,
and for version 2 also `version, ax, ay, gz, flow_vx, flow_vy, net_forward`. `battery` and
`torch` are integers.

## 2. Commands: dashboard to phone (TCP 9001)

The phone listens on TCP 9001 and advertises Bonjour service `_groundspeed._tcp`. Both
directions carry newline-delimited JSON, one object per line, with exactly one reply per request.

| Request | Effect |
|---|---|
| `{"cmd":"ping"}` | liveness check |
| `{"cmd":"start_run","label":"…"}` | start recording a run (label optional) |
| `{"cmd":"stop_run"}` | stop recording |
| `{"cmd":"mark","label":"…"}` | write a marker to the run's `events.csv` |
| `{"cmd":"zero"}` | zero distance and net_forward, then re-estimate IMU bias (hold still ~1 s) |
| `{"cmd":"measure_height"}` | measure camera height with LiDAR (hold still ~2 s; flow pauses) |
| `{"cmd":"calibrate"}` | re-estimate IMU bias (hold still) |
| `{"cmd":"reset_distance"}` | zero distance only |
| `{"cmd":"set_torch","level":0.6}` | torch level, 0 (off) to 1 |

Replies: `{"ok":true,"cmd":"…"}` or `{"ok":false,"cmd":"…","error":"…"}`. Extra keys:
`start_run` and `stop_run` carry `run` (the run id); `stop_run` carries `distance` (m);
`measure_height` carries `h` (m) and a human-readable `message`. Receivers ignore unknown keys.

The phone sends telemetry to the most recent command client, so connect one dashboard at a time.

## 3. Run files on the phone

Each recorded run is a folder `Documents/runs/<run_id>/`, where run_id is
`yyyy-MM-dd_HH-mm-ss[_label]`. The folder is visible in the Files app and can be exported as a
zip from the app.

| File | Header | Rate and meaning |
|---|---|---|
| `imu.csv` | `t,ax,ay,az,gx,gy,gz` | 100 Hz, vehicle frame, gravity removed, bias-corrected |
| `flow.csv` | `t,vx_cam,vy_cam,quality,h` | every camera frame (up to 240 Hz). Vehicle-frame velocity after gyro de-rotation: exactly what the filter consumed |
| `gnss.csv` | `t,speed,speed_acc,course` | ~1 Hz; course in degrees |
| `depth.csv` | `t,h` | LiDAR height when available |
| `est.csv` | `t,v_x,v_y,sigma_vx,sigma_vy,distance,status,flow_quality,h` | every filter step (100 Hz) |
| `events.csv` | `t,event,value` | start, stop, mark, zero, zupt, calibrate, torch, camera events |
| `meta.json` | | run id, label, app version, device, filter name and config, mount height and mapping, focal length, scale factor, battery, start/end time |

`t` is printed with microsecond resolution; other values with 9 significant digits. The phone
rounds every input to its printed value before the filter sees it, so replaying `imu.csv`,
`flow.csv`, `gnss.csv` and the `zupt` events in timestamp order (ties: IMU, flow, GNSS, ZUPT)
reproduces `est.csv`. `kfreplay` does this.

The dashboard writes its own log per session: `dashboard/logs/<session>/telemetry.csv` (the
`est.csv` columns plus `ax,ay,gz,flow_vx,flow_vy,net_forward,seq,recv_time,version`, where
`recv_time` is the laptop's wall clock) and `commands.csv` (`recv_time,request,reply`).

## 4. Test vectors

Both implementations must produce these bytes for the frame
`seq=42, t=1234.5, v_x=1.25, v_y=-0.03, sigma_vx=0.05, sigma_vy=0.06, distance=3.5,
flow_quality=12.0, h=0.3, status=0x0203 (IMU_OK|FLOW_OK|FILTER_INIT), battery=87, torch=60`.
They are checked by `swift run gsk-checks` and `python3 -m unittest discover dashboard/tests`.

Version 1 (48 bytes, CRC `0xCD3C`):

```
a5012a00000000000000004a93400000a03f8fc2f5bccdcc4c3d8fc2753d00006040000040419a99993e0302573c3ccd
```

Version 2 (72 bytes), adding `ax=0.5, ay=-0.25, gz=0.01, flow_vx=1.2, flow_vy=-0.02,
net_forward=3.25`:

```
a5022a00000000000000004a93400000a03f8fc2f5bccdcc4c3d8fc2753d00006040000040419a99993e0302573c0000003f000080be0ad7233c9a99993f0ad7a3bc000050408aaf
```

JSON form of the version 1 frame:

```json
{"battery":87,"distance":3.5,"flow_quality":12,"h":0.3,"seq":42,"sigma_vx":0.05,"sigma_vy":0.06,"status":515,"t":1234.5,"torch":60,"v_x":1.25,"v_y":-0.03}
```
