# Hackathon data archive

Every log recorded and used during MHacks 2026 (Oct 3 19:30 to Oct 4 02:15) is in this folder. The story behind the runs is in `docs/HACKATHON_WRITEUP.md`.

```
data/
├── phone_runs/<run_id>/     55 runs recorded on the iPhone (full rate)
├── dashboard_logs/<session>/ 21 dashboard sessions (50 Hz telemetry as received on the Mac)
├── lidar/<time>.csv          32 LiDAR height measurements, every frame of each attempt
├── ios_diagnostics/          iOS crash / resource reports for GroundSpeed and the camera daemon
└── tools/summarize_runs.py   index + GPS-vs-flow analysis
```

Runs were pulled from the phone several times during the night, so the same run often appeared in more than one pull. This folder keeps one copy of each, the most complete one. All times are local (EDT).

## File formats

The full schema is in `docs/PROTOCOL.md` (section "Run CSV schema"). `t` is the phone's monotonic host clock in seconds, the same for every file in a run.

| File | Columns | Rate |
|---|---|---|
| `flow.csv` | `t, vx_cam, vy_cam, quality, h` | 240 Hz (120 Hz if hot); velocity from the camera, quality = PSR, h = height used |
| `imu.csv` | `t, ax, ay, az, gx, gy, gz` | 100 Hz, gravity removed, m/s² and rad/s |
| `gnss.csv` | `t, speed, speed_acc, course` | ~1 Hz; speed and its accuracy in m/s |
| `est.csv` | `t, v_x, v_y, sigma_vx, sigma_vy, distance, status, flow_quality, h` | 100 Hz Kalman filter output; `distance` is the run total |
| `depth.csv` | `t, h` | LiDAR during the run (usually empty: LiDAR can't run alongside the 240 fps camera) |
| `events.csv` | `t, event, value` | start/stop, camera format and frame stats, zupt, calibrate, measure h, interruptions |
| `meta.json` | | phone, filter config, mount mapping, calibration at record time |
| `dashboard_logs/*/telemetry.csv` | est fields plus accel, flow, `net_forward`, `seq`, `recv_time` | 50 Hz over Wi-Fi |
| `dashboard_logs/*/commands.csv` | `recv_time, request, reply` | every button press and the phone's reply; `start_run`/`stop_run` replies carry the phone run ID and final distance |

## Which run is which

Runs not listed were short checks, stationary tests or aborted starts. The dashboard's run numbers restarted when the runs list was cleared, so the numbers here are the ones used in conversation at the time.

### Cart, indoor floor, taped course

The calibration runs. Phone distances are as recorded; calibration was applied *after* runs 5–7 and refined after 9–11, so the early runs read high in the raw data.

| Dash run | Phone run ID | h (m) | Course | Phone read | Notes |
|---|---|---|---|---|---|
| – | `2026-10-03_21-34-31` | 0.183 | | 19.39 m | Camera daemon crashed after Measure h; IMU-only drift (writeup 3.6) |
| 3 | `2026-10-03_22-18-56` | 0.187 | | 4.637 m | |
| 4 | `2026-10-03_22-20-43` | 0.186 | | 4.583 m | Already moving at Start, stopped partway |
| 5 | `2026-10-03_22-22-18` | 0.186 | 16 ft (4.877 m) | 5.235 m | +7 %, before the focal correction |
| 6 | `2026-10-03_22-23-45` | 0.187 | 16 ft | 5.246 m | +7.6 % |
| 7 | `2026-10-03_22-25-09` | 0.187 | 16 ft | 5.214 m | +6.9 %, about 1 m/s |
| 9 | `2026-10-03_22-37-35` | 0.236 | 16 ft | 4.830 m | −1.0 % with the first correction |
| 10 | `2026-10-03_22-38-37` | 0.235 | 16 ft | 4.849 m | −0.6 % |
| 11 | `2026-10-03_22-39-51` | 0.235 | 16 ft | 4.814 m | −1.3 %, different lighting |
| 12–14 | `2026-10-03_22-54-25`, `_22-55-58`, `_22-57-27` | 0.25 | | 4.84 / 6.05 / 6.02 m | Not discussed |
| 15 | `2026-10-03_22-59-53` | 0.798 | 20 ft (6.096 m) | 5.692 m | 31 in mount; faster push |
| 16 | `2026-10-03_23-01-53` | 0.792 | 20 ft | 5.078 m | −16.7 % |
| 17 | `2026-10-03_23-03-20` | 0.793 | 20 ft | 5.181 m | −15.0 % |
| 18 | `2026-10-03_23-23-07` | 0.794 | 20 ft | 4.185 m | −31 % with static-pattern subtraction (reverted) |

### Cart, variable ride height (20 ft)

| Dash run | Phone run ID | h at start | Phone read | Error |
|---|---|---|---|---|
| 1 | `2026-10-03_23-39-53` | 0.263 | 5.719 m | −6.2 % |
| 2 | `2026-10-03_23-40-27` | 0.165 | 5.259 m | −13.7 % |
| 3 | `2026-10-03_23-41-51` | 0.167 | 4.596 m | −24.6 % |
| 4 | `2026-10-04_00-00-16` | 0.257 | 5.891 m | −3.4 % with image-expansion height tracking |
| 5 | `2026-10-04_00-01-22` | 0.260 | 4.022 m | Tracked height swung 0.14–0.39 m; dashboard +4 m jump bug (writeup 3.11) |

### Car (Mazda CX-5 AWD, night, compared with GPS)

| Phone run ID | Mount h | What happened |
|---|---|---|
| `2026-10-04_01-14-56` | 0.173 (tracked h collapsed) | First drive, 12 s |
| `2026-10-04_01-19-31` | 0.173 | **Car test 1.** Height tracker collapsed to 0.058 m (speed 0.31× GPS); window wrap at about 6 m/s; mount knocked at t ≈ 123 s |
| `2026-10-04_01-45-16` | 0.165 | **Car test 2**, with motion prediction. GPS match ±10 % to 8 m/s for the first 40 s; tracker drift to 0.110 m after that; zero lock at t = 163 s |
| `2026-10-04_01-59-42` | 0.171 | **Car test 3** (final). Height fixed: 10–15 mph 0.97× GPS, 15–20 mph 1.01×, consistent to about 25 mph; blur-limited and zero lock near 30 mph (t ≈ 264–272 s). Distance was reset mid-run at t ≈ 231 s |

### Other

- `*_focal_check`, `*_bg_check`, `*_fps_check`: 2 s diagnostic recordings made over USB.
- `2026-10-03_19-*` to `20-31-*`: early runs at the 0.3 m default height (no LiDAR yet, 92–118 Hz Debug build).
- `2026-10-03_21-20-*` to `21-21-37`: jostle/wiggle investigation.
- `ios_diagnostics/cameracaptured-*.ips`: the camera daemon crashes after LiDAR measurements (writeup 3.6). `GroundSpeed-*.ips`, `*.cpu_resource`, `JetsamEvent-*`: app crash, CPU and memory-kill reports from the same period.

## Using the data

**List every run** (duration, distance, max speed, max GPS speed, height, flow quality, flow rate):
```bash
python3 data/tools/summarize_runs.py            # add --markdown for a table
```

**Speed vs. GPS over time for one run** (4 s bins: GPS, raw flow, filter estimate, ratio, quality, h):
```bash
python3 data/tools/summarize_runs.py 2026-10-04_01-59-42 --bin 4
```

**Re-run the Kalman filter on a recorded run** (try other filters or configs):
```bash
cd GroundSpeedKit
swift run -c release kfreplay ../data/phone_runs/2026-10-03_22-37-35
swift run -c release kfreplay ../data/phone_runs/2026-10-03_22-37-35 --all
swift run -c release kfreplay ../data/phone_runs/2026-10-03_22-37-35 --config my_cfg.json --out /tmp/est.csv
```
It prints distance, flow accepted/gated/skipped and the max difference from the phone's `est.csv`. By default it writes `est_replay_<filter>.csv` next to the run; use `--out` to keep the archive clean. The phone's `distance` column uses lag-compensated smoothing (`DistanceIntegrator`), so plain filter distance can differ slightly.

**Flow benchmark** (synthetic floor, real correlator):
```bash
cd GroundSpeedKit
swift run -c release flowbench --h 0.171 --focal 884 --predict --predict-err 4 --step 16 --max 400
```

**Pulling new runs from the phone** (USB, Xcode installed):
```bash
xcrun devicectl device copy from --device <UDID> --domain-type appDataContainer \
  --domain-identifier com.mhacks26.groundspeed --source Documents/runs --destination device_data/pullN/runs
```
LiDAR logs are under `Documents/lidar` in the same way.
