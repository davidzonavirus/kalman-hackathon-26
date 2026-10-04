# Forensic analysis: Open Ground Speed Sensor (MHacks 2026)

An independent reconstruction of what this repository measured, what went wrong, what was fixed, and
how accurate the final system is. All raw data under `data/` is read-only; everything generated
lives in `analysis/out/`.

**Deliverables**

| What | Where |
|---|---|
| Slideshow (16 slides) | `out/GroundSpeed_Forensics.pptx` (PNG renders in `out/slides_png/`) |
| Machine-readable headline metrics | `out/final_metrics.json` |
| Monte Carlo results | `out/mc_distance_summary.json`, `out/mc_velocity_summary.json`, `out/mc_*_samples.npz` |
| Figures | `out/figures/*.png` |
| Per-run tables | `out/inventory_*.csv`, `out/replay_validation.csv`, `out/distance_runs.csv`, `out/velocity_samples_*.csv` |

## Reproduce

```bash
python -m venv .venv && .venv/bin/pip install -r analysis/requirements.txt   # Windows: .venv\Scripts\pip
python analysis/run_all.py          # full run, ~1-2 h (Monte Carlo dominates)
python analysis/run_all.py --fast   # smoke test with small Monte Carlo (numbers NOT for reporting)
powershell -File analysis/render_slides.ps1   # optional: PNG renders (Windows + PowerPoint)
```

Random seeds are fixed (`common.SEED = 20261004`; velocity MC uses `SEED+7`). Swift is not needed:
`kf.py` is a line-by-line Python port of `ReferenceKF4` + `DistanceIntegrator`. On Windows the scripts
use `\\?\` paths because the data paths exceed the 260-character limit (the repo's own
`data/tools/summarize_runs.py` fails there for that reason).

| Step | Script | Does |
|---|---|---|
| 1 | `01_inventory.py` | per-run stream statistics, timestamps, NaN/dup/gap checks, LiDAR and dashboard inventories |
| 2 | `02_replay_validation.py` | replays every run through the Python filter port; infers which distance integrator each run used |
| 3 | `03_distance_analysis.py` | taped-course errors; re-fits the 2-parameter calibration; re-applies it by full replay; leave-one-out |
| 4 | `04_velocity_analysis.py` | car runs vs GPS speed: lag, GPS noise, envelope, segments, bootstrap CIs |
| 5 | `05_noise_characterization.py` | flow frame noise by PSR, tails, autocorrelation; IMU at rest; LiDAR repeatability |
| 6 | `06_mc_distance.py` | distance Monte Carlo (100,000 trials through the vectorised filter + odometer) |
| 7 | `07_mc_velocity.py` | velocity Monte Carlo (100,000 trials, 95 s of the final drive) |
| 8 | `08_story_figures.py` | chronology and before/after evidence figures |
| 9 | `09_summary.py` | collects `final_metrics.json` |
| 10 | `10_build_slides.py` | builds the PPTX from `final_metrics.json` and the figures |

`mc_kf.py` holds the vectorised filter; its `selftest()` feeds real measurements as identical trials and
matches the scalar port to 1e-16 m/s, including GPS fusion.

## 1. The actual pipeline (from the code, not assumed)

```
camera 240 fps ─ 256² crop → 2× downsample → 128² phase correlation ─► shift (px/frame), PSR
   v = −shift · 2 · h / (f · Δt)  ·  tilt: h/cosθ  ·  mount mapping  ·  gyro de-rotation v −= h·ω
LiDAR (once, cart still) ─► h = depth·cosθ − 7.9 mm          f = FOV pinhole focal × 1.027
IMU 100 Hz ─► ReferenceKF4 predict   x=[vx, vy, bx, by]
flow ─► scalar updates, R = 1.6e-3·(20/PSR)², PSR<8 skipped, NIS gate 9, lockout reset after 12
GPS speed (>1 m/s) ─► update with R = speed_acc²      ZUPT (IMU still) ─► v = 0, R = 1e-4
filter v ─► odometer ∫|v̄|dt  (deadband 0.05 m/s, low-pass τ = 0.5 s, + τ·|v̄| lead) ─► est.csv / UDP
```

Velocity is the primary measurement; distance is integrated from the filtered velocity. Truth
available in the archive: taped courses (16 ft = 4.8768 m, 20 ft = 6.096 m) for distance on the cart;
phone GPS Doppler speed for speed in the car. There is no wheel encoder or RTK reference.

## 2. Data inventory (summary; full tables in `out/inventory_*.csv`)

| Path | Type | Rate / size | Role |
|---|---|---|---|
| `data/phone_runs/<id>/flow.csv` | raw (de-rotated) camera velocity, PSR, h | 240 Hz (60–120 Hz before 20:36) | filter input |
| `…/imu.csv` | accel (gravity removed, bias-corrected), gyro | 100 Hz | filter predict |
| `…/gnss.csv` | speed, speed_acc, course | ~1 Hz | filter input; our speed reference |
| `…/est.csv` | filter output v, σ, distance, status | 100 Hz | phone output (reproduced by replay) |
| `…/events.csv` | start/stop, ZUPT, calibrate, camera, reset | events | replay order, chronology |
| `…/depth.csv` | LiDAR during run | empty in all runs | – |
| `…/meta.json` | focal_px, mount height, filter config | once | **chronology of calibration changes** |
| `data/lidar/*.csv` | per-frame LiDAR depth, cos tilt, h; `# result` line | ~30 Hz × 1.5 s | height measurement, repeatability |
| `data/dashboard_logs/*/telemetry.csv` | 50 Hz telemetry as received | Wi-Fi | cross-check (stop_run distances match est.csv exactly) |
| `data/ios_diagnostics/*.ips` | crash / resource reports | 21 files | camera-daemon crash evidence |

Health: 58 runs (36 LiDAR logs), 0 NaN, 0 infinite, 0 backwards timestamps, 1 duplicate flow timestamp, 3 duplicate GPS
timestamps (dropped). Dashboard telemetry lost 2,293 packets in a car session (Wi-Fi), not used for accuracy.

## 3. Chronology and issue ledger

Git has only one code commit for the whole night (`379dcfd` 19:41 → `aa0d32a` 02:12), so the
sequence below comes from per-run `meta.json`, frame rates, `events.csv`, LiDAR logs and crash-report
times, cross-checked against the code diff and `docs/HACKATHON_WRITEUP.md`.

Evidence levels: **CONFIRMED** (code and data both show it), **SUPPORTED** (data consistent, mechanism
plausible, not isolated), **HYPOTHESIS** (not tested by the data), **CORRECTION** (the write-up and the data disagree).

| # | Issue | Evidence | Fix | Verified effect | Level |
|---|---|---|---|---|---|
| 1 | Debug build processed 60–120 frames/s | flow.csv rate 69–118 Hz before 20:36, 232–240 Hz after | Release build, 240 fps | Within one setup, PSR falls from 396 to 61 as shift grows 1.4 → 14 px/frame, so 2.7× less shift per frame at 240 fps raises quality. The Debug era's own curve is flatter (different h, floor, exposure), so there is no controlled before/after. | SUPPORTED |
| 2 | Early filter config | meta q_accel 0.02 in the 19:34/19:39 runs, 0.1 after | q_accel raised | Those two runs don't replay even with 0.02, so that build differed in other ways too | CONFIRMED (config) |
| 3 | Odometer changed three times | replaying logged velocities: plain ∫\|v\| (19:34), +0.05 m/s deadband (20:06), + τ = 0.5 s smoothing (21:20); at stop the 22:xx runs exclude the lead term (old cutoff) | smoothing + lead fade | On real jostle logs smoothing removed 21–47 % of distance; true distance unknown | CONFIRMED (behaviour) |
| 4 | LiDAR tilt counted twice | 379dcfd: depth used as h, FlowConverter ÷cosθ again; 20:26 log: depth 0.132 m at 38°, vertical 0.104 m | h = depth·cosθ once; reject bad readings | +61 % speed error at 38° removed. No recorded run used a buggy LiDAR h, so there is no run-level before/after | CONFIRMED |
| 5 | Camera daemon crash after Measure h → IMU-only runaway | 14 `cameracaptured` reports 20:26–21:43; runs 21:34 (19.4 m in 31 s) and 21:37 (15.0 m in 3 s) with zero flow frames | 30→240 fps restart, watchdog, ZUPT after 2 s flow loss | 21:46/21:48 runs: 2.1 s camera gap then recovery, distance at rest 0.07/0.00 m; no crash reports archived after 21:43 | CONFIRMED |
| 6 | +7 % scale on every push | runs 5–7 +7.34/+7.57/+6.92 %; focal_px 860.66 → 923.48 (×1.073) → 883.89 (×1.027) | focal ×1.027, LiDAR −7.9 mm | Our refit gives ×1.0268 ± 0.012, −8.0 ± 2.3 mm (r = 0.99); in-sample max 0.32 %, leave-one-out max 0.54 % | CONFIRMED (fix); cause = SUPPORTED |
| 7 | 79 cm mount reads low | runs 15/16/17: −6.6/−16.7/−15.0 % | static-pattern subtraction (run 18: −31.3 %) → reverted | unresolved; cause untested | HYPOTHESIS (cause) |
| 7b | Write-up says runs 15–17 read "15–17 % low" | run 15 = 5.692 m on 6.096 m = **−6.6 %** | – | – | CORRECTION |
| 8 | Ride height changes mid-run | ride runs 1–3: −6.2/−13.7/−24.6 %; expansion tracking −3.4 then −34.0 % | tracking turned off | unresolved | CONFIRMED (failure) |
| 9 | Dashboard +4 m jump | est.csv of 00:01:22 dips 2.49 cm at stop (lead-term cutoff) | fade lead; dashboard reset rule | dip reproduced from the log | CONFIRMED |
| 10 | Car 1: height tracker collapsed, window wrap | tracked h 0.17 → 0.06 m; flow ÷ GPS median 0.31 at 10–15 mph | motion prediction | car 2 ratio 0.69–0.80 | CONFIRMED |
| 11 | Car 2: tracker drift, zero-lock | h → 0.11 m; flow 0 while GPS 4–7 m/s | tracking off; coasting; jump rejection | car 3 steady cruise ratio 0.984 | CONFIRMED |
| 12 | Car 3: blur above ~25 mph, zero-lock 273–304 s | PSR ≈ 11 near 10 m/s; 30 GPS epochs at >3 m/s read <0.5 m/s | fake-stop rule (ef48eaa); prediction only above 32 px/frame (5295780) | **not road-tested**: the only later runs are three short indoor pushes at 02:26–02:27 (h 0.171/0.32/0.506 m, no GPS, no course length) | UNTESTED |
| 13 | GPS accuracy | iOS speed_acc median 2.2 m/s while moving (final drive), not "±0.3 m/s"; empirical white noise 0.16 m/s; lag 0.65 s behind the camera | – | GPS barely affects the filter: removing it changes the estimate by at most 4 mm/s | CORRECTION (write-up figure) |
| 14 | Sign convention | every calibration push moved along the phone's −x axis (∫vₓ ≈ −4.9 m) | none needed: odometer is ∫\|v\| | the signed `net_forward` field would read negative | CONFIRMED (harmless) |

## 4. Target velocity: definition and limits

- **Cart:** no time-resolved speed truth exists. Only the course length is known, so the mean-speed error
  over a run equals the distance error (same duration). No other cart velocity number is claimed.
- **Car:** TARGET = GPS Doppler ground speed (`gnss.csv speed`), the only independent, time-resolved
  speed in the archive and the reference the team used. It is shifted by the fitted lag (GPS trails
  the camera by 0.65 s in drives 2 and 3; drive 1's fit hits the search edge and is unreliable).
  Its white noise (0.16 m/s) is estimated from the second difference of the 1 Hz series. Its reported 1σ
  (median 2.2 m/s) is far larger, so iOS's accuracy figure looks very conservative, but that is unproven.
  The driver's mph targets (10/15/20/25/30) are visible as plateaus but were never logged, so they are not used.
- **ESTIMATED** speed is the filter replayed **without GPS updates**, so it is independent of the target.
  (The phone's logged estimate differs from it by at most 4 mm/s.)
- `velocity_error = estimated − target`, evaluated at each GPS epoch as a time series
  (`out/velocity_samples_*.csv`, `figures/vel_final_timeseries.png`).

## 5. Results

TBD_RESULTS

## 6. Assumptions (all of them)

TBD_ASSUMPTIONS
