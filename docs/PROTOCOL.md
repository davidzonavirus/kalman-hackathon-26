# Interface contract (FROZEN — hour 1)

Same field names in UDP packets, CSV headers, and Swift/Python code. All times are
seconds on the phone's mach clock (`CACurrentMediaTime()` / `ProcessInfo.systemUptime`
domain). All velocities m/s, distances/heights m, angles rad unless stated.

Vehicle frame: **x = forward, y = left, z = up.** `r` = yaw rate about z (rad/s, CCW +).

## 1. Telemetry frame: phone → dashboard (UDP, 50 Hz)

Dashboard listens on **UDP 9000** (all interfaces). The phone sends to a configured
host; when a dashboard connects on the TCP command port, the phone automatically
adopts that peer's IP as its UDP destination.

### Binary frame (default) — 48 bytes, little-endian

| Offset | Type | Field | Notes |
|---|---|---|---|
| 0 | u8 | magic | `0xA5` |
| 1 | u8 | version | `1` |
| 2 | u32 | seq | starts at 0 on app launch, +1 per frame, wraps at 2³²; a backwards jump = phone restart |
| 6 | f64 | t | s, phone mach clock |
| 14 | f32 | v_x | m/s |
| 18 | f32 | v_y | m/s |
| 22 | f32 | sigma_vx | m/s (1σ) |
| 26 | f32 | sigma_vy | m/s (1σ) |
| 30 | f32 | distance | m, SIGNED forward odometer ∫v_x dt (jostling cancels; vehicle frame turns with the cart so curves still count) since run start (or last reset_distance / app start if not recording); never reset mid-run except by reset_distance |
| 34 | f32 | flow_quality | peak-to-sidelobe ratio of last flow sample (0 if none) |
| 38 | f32 | h | m, camera height used |
| 42 | u16 | status | bit flags, see below |
| 44 | u8 | battery | percent 0–100, `0xFF` unknown |
| 45 | u8 | torch | torch level ×100 (0–100) |
| 46 | u16 | crc | CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect, xorout 0) over bytes 0..45 |

Check value: CRC-16/CCITT-FALSE of ASCII `"123456789"` = `0x29B1`.

### Status flags (u16)

| Bit | Name | Meaning |
|---|---|---|
| 0 | IMU_OK | IMU samples arriving (< 50 ms old) |
| 1 | FLOW_OK | flow sample < 100 ms old AND quality ≥ threshold AND not gated |
| 2 | GNSS_OK | GNSS fix < 2 s old with speed_acc ≥ 0 |
| 3 | LIDAR_OK | depth sample < 500 ms old (else h is the fixed/manual value) |
| 4 | RECORDING | a run is being recorded |
| 5 | ZUPT | zero-velocity update applied in last 100 ms |
| 6 | FLOW_GATED | last flow update rejected by innovation gate |
| 7 | GNSS_GATED | last GNSS update rejected by innovation gate |
| 8 | TORCH_ON | torch is on |
| 9 | FILTER_INIT | filter initialized and running |
| 10 | CALIBRATING | mount/height calibration in progress |
| 11–15 | reserved | 0 |

### JSON debug frame

Same datagram port. A datagram whose first byte is `{` is a UTF-8 JSON object with
exactly these keys:
`{"seq":…, "t":…, "v_x":…, "v_y":…, "sigma_vx":…, "sigma_vy":…, "distance":…, "flow_quality":…, "h":…, "status":…, "battery":…, "torch":…}`
(`torch` is 0–100 int, `battery` 0–100 or 255). The dashboard accepts both formats.

## 2. Commands: dashboard → phone (TCP 9001)

Phone listens on **TCP 9001** and advertises Bonjour `_groundspeed._tcp`.
Newline-delimited JSON, one object per line, both directions.

Requests:
```
{"cmd":"ping"}
{"cmd":"start_run"}             // optional "label":"steady_carpet_03"
{"cmd":"stop_run"}
{"cmd":"mark"}                  // optional "label":"lens_covered"
{"cmd":"calibrate"}             // re-zero mount/bias calibration (cart must be still)
{"cmd":"set_torch","level":0.6} // 0.0 = off .. 1.0
{"cmd":"reset_distance"}
```
Reply (one per request): `{"ok":true,"cmd":"start_run","run":"2026-10-03_18-40-12"}` or
`{"ok":false,"cmd":"…","error":"…"}`.
`stop_run` replies also carry the run id and `"distance"` (m, run distance at stop):
`{"ok":true,"cmd":"stop_run","run":"…","distance":10.04}`. Receivers ignore unknown keys.

Coupling note: the phone sends UDP telemetry to the most recent TCP command client, so
only one dashboard at a time should be connected for commands.

## 3. Per-run storage on the phone

`Documents/runs/<run_id>/` (run_id = `yyyy-MM-dd_HH-mm-ss[_label]`), visible in Files
(`UIFileSharingEnabled` + `LSSupportsOpeningDocumentsInPlace`), exported as a zip via
the share sheet / AirDrop.

| File | Header (exact) | Rate |
|---|---|---|
| `imu.csv` | `t,ax,ay,az,gx,gy,gz` | 100 Hz — vehicle frame, gravity removed, m/s², rad/s |
| `flow.csv` | `t,vx_cam,vy_cam,quality,h` | 30–120 Hz — vehicle frame m/s, AFTER gyro de-rotation (v_x − h·ω_y, v_y + h·ω_x; ω = mean gyro since previous flow sample) — exactly what the filter consumed |
| `gnss.csv` | `t,speed,speed_acc,course` | ~1 Hz (course in deg) |
| `depth.csv` | `t,h` | 15–30 Hz (if LiDAR live) |
| `est.csv` | `t,v_x,v_y,sigma_vx,sigma_vy,distance,status,flow_quality,h` | every filter step (100 Hz) |
| `events.csv` | `t,event,value` | start/stop/mark/torch/calibrate |
| `meta.json` | see below | once |

`meta.json`: `{"run_id","label","app_version","device","filter_name","mount_height_m",
"torch_level","scale_factor","focal_px","mount_config":{...},"start_t","end_t",
"battery_start","battery_end","q":{...},"r":{...}}`

Replay rule: feeding `imu.csv` (predict), `flow.csv`, `gnss.csv`, and ZUPT events in
timestamp order through the filter must reproduce `est.csv` (that is how Swift and
Python implementations are cross-checked to 1e-6 m/s).

---

## v2 frame (REPLACES v1 as the phone's default; dashboards must accept both)

72 bytes, little-endian. Bytes 0–45 identical to v1 except `version = 2` and the meaning of
`distance`, then:

| Offset | Type | Field | Notes |
|---|---|---|---|
| 30 | f32 | distance | m, **total distance travelled** = ∫\|v\| dt since last zero/run start (stationary periods excluded) |
| 46 | f32 | ax | m/s², vehicle frame, gravity removed, bias-corrected (as fed to the filter) |
| 50 | f32 | ay | m/s² |
| 54 | f32 | gz | rad/s yaw rate |
| 58 | f32 | flow_vx | m/s, last raw flow measurement (vehicle frame, de-rotated) |
| 62 | f32 | flow_vy | m/s |
| 66 | f32 | net_forward | m, signed ∫v_x dt since last zero (displacement along the vehicle axis) |
| 70 | u16 | crc | CRC-16/CCITT-FALSE over bytes 0..69 |

JSON debug frame v2 adds keys `ax, ay, gz, flow_vx, flow_vy, net_forward` and `"version":2`.
Speed `|v| = hypot(v_x, v_y)` is derived by receivers.

New command: `{"cmd":"zero"}`: zero distance + net_forward and re-estimate IMU bias (phone
must be still ~1 s). Reply `{"ok":true,"cmd":"zero"}`.

---

## 4. FPGA estimate return: dashboard → phone (UDP, same socket)

When the dashboard runs the Kalman filter on the simulated FPGA (`dashboard/run.sh --fpga`, see
`fpga/README.md`), it answers every v2 telemetry frame it processed with the FPGA's estimate.
The datagram is sent **from the dashboard's telemetry socket (UDP 9000) to the source address
of that frame**, so it arrives on the UDP connection the phone already sends telemetry from: no
new port, no firewall rule, nothing to configure on the phone. One UTF-8 JSON object per
datagram:

```
{"type":"fpga_est","seq":1234,"t":5012.31,"v_x":1.02,"v_y":-0.01,"sigma_vx":0.02,"sigma_vy":0.02,
 "distance":4.81,"net_forward":4.79,"status":547,"backend":"rtl-iverilog"}
```

| Key | Meaning |
|---|---|
| `type` | always `"fpga_est"` (anything else on this socket is ignored) |
| `seq`, `t` | the phone frame this answers (its `seq` and `t`) |
| `v_x`, `v_y`, `sigma_vx`, `sigma_vy` | the FPGA's estimate after that frame (m/s, 1σ) |
| `distance`, `net_forward` | m, integrated by the dashboard from the FPGA's velocity; reset on start_run / zero / reset_distance, frozen on stop_run |
| `status` | the frame's status bits with FLOW_GATED / FILTER_INIT from the FPGA's own outcomes |
| `backend` | `rtl-iverilog` (clock-level Verilog) or `python-model` (Python model of the same processor) |

Rate: one per processed frame (≤ 50 Hz). Values are never NaN (such frames are not answered).
The phone shows these numbers while they keep arriving and falls back to its own filter when
none has arrived for 0.5 s (`GroundSpeedRuntime.fpgaEstimate(maxAge:)`). Receivers ignore
unknown keys. A dashboard without `--fpga` sends nothing back.
