# Open Ground Speed Sensor

An iPhone pointed at the ground becomes an optical ground-speed sensor. The camera watches the
surface move, LiDAR measures the camera's height, and the IMU and GNSS fill the gaps. A Kalman
filter running on the phone fuses them into velocity and distance, records every run, and
streams live telemetry to a dashboard on a laptop.

Commercial optical ground-speed sensors (Kistler Correvit, Datron) are standard equipment in
vehicle-dynamics testing and cost thousands of dollars. This project does the core job with a
phone and a small estimator.

## Results

| Test | Setup | Result |
|---|---|---|
| Taped 16 ft (4.877 m) course, cart, 6 runs | Camera height 18.6 cm and 23.5 cm, indoor floor | All six within ±0.4 % of tape after calibration |
| Car, compared with GPS speed | 17 cm mount, 240 fps, 1/1000 s exposure, night | 0.97× GPS at 10–15 mph, 1.01× at 15–20 mph, consistent to about 25 mph |
| Repeatability | Same height, three runs | Within 1 % of each other before calibration |

Every run behind these numbers is in `data/` (see [Data](#data)).

## Repository layout

| Path | Contents |
|---|---|
| `GroundSpeedKit/` | Swift package with all platform-independent code: wire protocol, Kalman filters, optical flow, the phone runtime, and the command-line tools |
| `ios/GroundSpeed/` | The iPhone app: sensor capture (camera, LiDAR, IMU, GNSS) and the SwiftUI interface |
| `dashboard/` | Laptop dashboard: Python standard-library server plus a browser UI |
| `docs/PROTOCOL.md` | Wire protocol, commands, run file formats and test vectors |
| `data/` | Recorded runs, dashboard sessions, LiDAR logs and the run index `data/runs.csv` |
| `hardware/MR26_GSS_2/` | Altium project for a dedicated sensor board |

## Quick start without a phone

The dashboard and a simulated phone run on any Mac. The simulator uses the same runtime code as
the iPhone app, fed with a synthetic 10 m push.

```bash
dashboard/run.sh
```

Open http://localhost:8080, then in a second terminal:

```bash
cd GroundSpeedKit && swift run phone-sim --host 127.0.0.1
```

To loop a recorded run instead, start the dashboard with `dashboard/run.sh --replay 11`, or
press **Replay run** in the dashboard and pick a run from the list.

Requirements: macOS 14+, Swift 5.9+ (Xcode or Command Line Tools), Python 3.10+. The dashboard
has no third-party dependencies and works offline.

## Architecture

```
iPhone                                                          Laptop
+----------------------------------------------------------+    +---------------------------+
| Camera 1280x720 @ 240 fps --> PhaseCorrelator -> flow v  |    | gsdash server (Python)    |
| LiDAR (on request) -------> camera height h              |    |   UDP receiver, seq/CRC   |
| CoreMotion 100 Hz --------> accel, gyro                  |    |   session CSV logger      |
| CoreLocation ~1 Hz -------> GNSS speed                   |    |   replay of logged runs   |
|                                                          |    |   HTTP + Server-Sent      |
| SensorFusionEngine (GroundSpeedKit/PhoneRuntime)         |    |   Events to the browser   |
|   reorder buffer -> gyro de-rotation -> Kalman filter    |    |   command link (TCP)      |
|   -> ZUPT -> distance integrators -> status flags        |    |                           |
| RunRecorder -> Documents/runs/<run_id>/*.csv             |    | Browser UI                |
| TelemetrySender  ---- UDP 9000, 50 Hz binary frames ---->|--->|   live readouts, charts,  |
| CommandServer    <--- TCP 9001, JSON commands ---------- |<---|   runs table, controls    |
+----------------------------------------------------------+    +---------------------------+
```

All estimation runs on the phone. The laptop only displays, logs and sends commands, so the
phone works without it: runs are recorded on the device and can be exported later.

## How the back end works

### Time base and sensors

Every sample is stamped on one clock: the phone's monotonic host clock in seconds (`Clock` in
`PhoneRuntime`). Camera frames use their capture timestamps, CoreMotion uses its own
timestamps (same clock), and GNSS fixes are converted from wall time. Misaligned timestamps
cost more accuracy than any filter choice, so this is enforced everywhere.

The iOS sensor sources (`ios/GroundSpeed/GroundSpeed/Sensors/`):

- `CameraFlowSource`: back wide camera at about 1280x720, up to 240 fps (120 fps under thermal
  pressure), exposure capped at 1/1000 s, torch on, focus and exposure locked after a short
  settle, video stabilisation off. Each frame goes through the phase correlator and becomes a
  velocity over the ground. A watchdog restarts the capture session if frames stop for 1.5 s.
- `DepthSource`: LiDAR height measurement in a separate short capture session (see below).
- `MotionSource`: CoreMotion device motion at 100 Hz, gravity removed, rotated from the phone's
  axes into the vehicle frame (x forward, y left, z up) by a configurable mount mapping.
- `LocationSource`: GNSS speed and speed accuracy; invalid and repeated fixes are dropped.

`AppModel` wires these into `GroundSpeedRuntime`. Everything after this point is
platform-independent Swift in `GroundSpeedKit` and is tested on the Mac.

### Optical flow (`GroundSpeedKit/Sources/OpticalFlow`)

`PhaseCorrelator` measures how far the ground texture moved between two frames:

1. Take a 256x256 pixel patch from the centre of the luma plane and box-downsample it to 128x128.
2. Remove the mean, apply a Hann window, and take the 2-D FFT (Accelerate/vDSP).
3. Multiply by the conjugate spectrum of the previous frame, normalise to unit magnitude
   (keeping only phase), apply a Gaussian low-pass, and inverse-FFT. The result has a sharp
   peak at the shift between the frames.
4. Locate the peak with sub-pixel accuracy (log-parabola fit). The quality score is the
   peak-to-sidelobe ratio (PSR): about 3–7 for noise or a covered lens, 20–600 on textured
   surfaces.

All buffers are preallocated; a frame takes under 0.5 ms on the phone.

`FlowConverter` turns the pixel shift into velocity with the pinhole model:

```
v = (shift_px × downsample / dt) × (h / f_px)
```

where `h` is the camera height and `f_px` the focal length in pixels, from the camera's
per-frame intrinsics or its field of view. Gravity from the IMU corrects for a tilted mount.
`MountMapping` maps camera axes onto vehicle axes; the app can learn it from one forward push.

**Motion prediction.** A 128-pixel window can only measure shifts up to ±64 downsampled pixels
per frame, about 6 m/s at a 17 cm mount and 240 fps. Above 32 full-resolution pixels per frame,
the current frame's crop is offset by the last confident shift, so the window only has to cover
the change in motion. The tracker coasts through brief dropouts (24 frames), re-acquires at
fractions and multiples of the last motion, rejects implausible jumps, and requires PSR 30 to
accept a sudden zero while moving, because the sensor's fixed pattern produces a weak peak at
zero. Below 32 pixels per frame the plain correlation is used, which is the path the cart runs
were calibrated on.

**Calibration constants.** `CameraFlowSource.fovFocalCorrection = 1.027` corrects lens
distortion at the image centre, and `DepthSource.heightOffset = -7.9 mm` corrects the offset
between the LiDAR origin and the camera's projection centre. Both were fitted jointly from six
16 ft runs at two heights.

### Camera height

Speed is proportional to height, so height is the calibration that matters most: 1 cm of error
at 30 cm is 3 % of speed.

**Measure height** (from the phone or the dashboard) pauses the flow camera for about 2 s, runs
a LiDAR session, takes the median depth of the image centre over all frames, and converts the
line-of-sight depth to vertical height with the gravity vector. A measurement is rejected if the
frames disagree by more than max(8 %, 1 cm), the depth is not metric, or the height is outside
5 cm–2 m. The same still window zeroes the distance and re-learns the IMU bias. iOS cannot run
LiDAR alongside the 240 fps camera, so height is measured, not streamed.

`HeightTracker` estimates height changes between LiDAR fixes from image expansion: four extra
correlation patches measure the flow gradient, and `Δh/h = A∥ − 2·A⊥` cancels the apparent
expansion caused by a tilted mount. It improved variable-height cart runs but drifted on the
car, so it is disabled (`CameraFlowSource.tracksHeight = false`) and the measured height is
used.

### Fusion engine (`PhoneRuntime/SensorFusionEngine.swift`)

The engine owns the filter, the zero-velocity detector and the distance integrators on one
serial queue. For each input it does the following:

1. **Reorder.** Samples from different sensors arrive out of order. They wait in a 50 ms buffer
   keyed to IMU time, then are processed strictly by timestamp (ties: IMU, flow, GNSS, ZUPT).
2. **Quantise.** Every input is rounded to the value written to the CSV before the filter sees
   it, so replaying the files reproduces the phone's estimate exactly.
3. **IMU sample.** Subtract the calibrated bias, run the filter's predict step with the yaw rate,
   update the distance integrators, write an `est.csv` row, and publish a snapshot for the UI and
   telemetry.
4. **Flow sample.** Remove rotation-induced motion first. Tilting the phone sweeps the image
   across the ground at `h × ω` without any translation, so the engine applies
   `v_x -= h·ω_y` and `v_y += h·ω_x` using the gyro rate averaged over the frame interval. Then
   it runs the flow update.
5. **GNSS sample.** Run the speed update (used above 1 m/s only).
6. **ZUPT.** `ZuptDetector` declares the phone stationary when accelerometer variance over
   0.3 s is low and flow speed is near zero (or the estimate is below 0.1 m/s with no flow).
   The engine then applies a zero-velocity update, which pins the velocity and lets the filter
   learn the accelerometer bias.

**Calibrate / Zero** average the IMU over a still window to estimate the bias. If the phone
moved during the window, the result is rejected and the previous bias kept.

**Distance.** Two `DistanceIntegrator`s run on the filter output:

- *Total distance* integrates speed `|v|` with a 0.05 m/s deadband, so noise at rest doesn't
  accumulate. Velocity is low-passed (τ = 0.5 s) first, so hand wobble doesn't count as
  travel, and a lead term `τ·|v̄|` is added to the readout to remove the low-pass lag while
  moving.
- *Net forward* integrates the signed forward velocity `v_x`, so back-and-forth motion cancels.

**Status flags** (IMU_OK, FLOW_OK, FLOW_GATED, ZUPT, RECORDING and others) are derived from
sample ages and the filter's last outcomes. They drive the status lights in the app and the
dashboard.

### Kalman filter (`GroundSpeedKit/Sources/KalmanCore`)

The app talks to the `GroundSpeedFilter` protocol, not to a concrete filter. `ReferenceKF4` is
the default implementation.

**State and model.** State `x = [v_x, v_y, b_x, b_y]`: velocity in the vehicle frame and the two
accelerometer biases. Inputs are the measured accelerations `a_x, a_y` and the yaw rate `r`.
With `Δt` from the timestamps (nominally 10 ms):

```
v_x' = v_x + Δt·(a_x − b_x + r·v_y)
v_y' = v_y + Δt·(a_y − b_y − r·v_x)
b'   = b                              (random walk)
```

Because the yaw rate is measured, the model is linear time-varying: no Jacobians and no matrix
inverse. The covariance prediction `F P Fᵀ + Q` is written out by hand to exploit the sparsity
of `F`.

**Measurements**, each applied as a scalar update with the Joseph-form covariance update and
forced symmetry:

| Measurement | Model | Noise |
|---|---|---|
| Optical flow | `z = v_x`, then `z = v_y` | `R = r_flow_base · (psr_ref / PSR)²`; skipped below `psr_min` |
| GNSS speed | `z = |v|`, `H = [v_x/|v|, v_y/|v|, 0, 0]` | `R = speed_accuracy²`; only above 1 m/s |
| Zero velocity | `z = 0` on both axes | `r_zupt` |

**Outlier rejection.** A flow or GNSS sample is rejected when its normalised innovation
`ν²/S` exceeds the gate (9, about 3σ). The gate is disarmed after a reset until the first
accepted measurement, so a filter started while moving can lock on. If 12 consecutive flow
samples are rejected, the filter concludes that its own estimate has diverged, reopens the
velocity covariance and accepts the measurement (lockout recovery).

**Defaults** (`FilterConfig`, editable in the app and saved with every run):

| Parameter | Value | Meaning |
|---|---|---|
| `q_accel` | 0.1 | acceleration process noise, (m/s²)²·s |
| `q_bias` | 1e-5 | bias random walk, (m/s²)²/s |
| `r_flow_base` | 1.6e-3 | flow variance at `psr_ref`, (m/s)² |
| `psr_ref`, `psr_min` | 20, 8 | flow quality reference and floor |
| `r_zupt` | 1e-4 | zero-velocity variance, (m/s)² |
| `gate` | 9 | innovation gate |
| `p0_v`, `p0_b` | 0.25, 0.01 | initial velocity and bias variance |
| `gate_reset_count` | 12 | lockout recovery threshold |

The exact step-by-step algorithm is documented at the top of `ReferenceKF4.swift`, so a port to
another language (or to fixed-point hardware) can reproduce it to 1e-6.

**Variants and adding a filter.** `DecoupledKF2x2` runs two independent 2-state filters (one
per axis) as a cheaper alternative. To add a filter, implement `GroundSpeedFilter` in
`KalmanCore` and register it in `FilterRegistry.all`. It then appears in the app's filter
picker, is recorded in `meta.json`, and can be compared with the others on any recorded run
with `kfreplay --all`.

### Recording (`PhoneRuntime/RunRecorder.swift`)

Each run is a folder `Documents/runs/<run_id>/` with one CSV per sensor (`imu`, `flow`, `gnss`,
`depth`), the filter output (`est.csv`), an event log and `meta.json`. Rows are formatted on the
calling thread, buffered on a private queue and flushed about once a second, so recording never
blocks a sensor callback. Runs are visible in the Files app and exportable as a zip. Formats are
in `docs/PROTOCOL.md`.

### Telemetry and commands (`PhoneRuntime`)

- `TelemetrySender` sends the latest snapshot as a 72-byte binary frame (or JSON for debugging)
  over UDP at 50 Hz, with a sequence number and CRC-16.
- `CommandServer` listens on TCP 9001 and advertises the Bonjour service `_groundspeed._tcp`.
  Commands are newline-delimited JSON with one reply each: start/stop run, mark, zero, measure
  height, calibrate, reset distance and torch level. When a dashboard connects, the phone
  retargets its UDP stream to that address, so no IP has to be typed in.
- `GroundSpeedRuntime` ties the engine, recorder, sender and server together and implements the
  command set. The iOS app and `phone-sim` both use it; only the sensor sources differ.

### Dashboard server (`dashboard/gsdash`)

A single Python process using only the standard library:

- **UDP receiver** on port 9000 (IPv4 and IPv6). Decodes v1 and v2 binary frames and JSON,
  validates magic, length and CRC, and tracks sequence numbers to count lost, duplicate and
  out-of-order frames and to detect phone restarts.
- **Telemetry store**: a ring buffer of the last 80 s of frames plus link statistics (rate,
  gaps, CRC errors, age of the last frame).
- **Session logger**: every frame to `dashboard/logs/<session>/telemetry.csv` and every
  command with its reply to `commands.csv`.
- **Command link** (`PhoneLink`): one persistent TCP connection to the phone with a 5 s
  keepalive and automatic reconnect. The phone's address comes from `--phone`, the UI, the
  source address of incoming telemetry, or Bonjour discovery (`dns-sd`), in that order.
- **HTTP server** on port 8080: serves the UI, streams frames and statistics to the browser as
  Server-Sent Events (`/events`, with the last 60 s replayed on connect), and exposes
  `/api/cmd`, `/api/status`, `/api/phone` and the replay endpoints.
- **Replay** (`replay.py`): finds every recorded run (phone RECORDING segments) in the session
  logs, both `dashboard/logs/` and the archive `data/dashboard_logs/`, numbers them oldest
  first, and plays one back at its original pace through the same pipeline as live data,
  optionally on a loop. Live telemetry keeps being logged during a replay but isn't shown.

The browser UI (`dashboard/gsdash/static/`) shows distance, speed and velocity with ±2σ,
stacked charts of velocity, acceleration, distance and flow quality, sensor status, link health,
a runs table with the error against a target distance (metres or feet, exportable as CSV), and
controls for Zero, Start/Stop tracking, Measure height, Mark, Calibrate and the torch.

Command-line options: `python3 -m gsdash --help` (`dashboard/run.sh` passes them through and
stops any running instance first, since two instances would compete for UDP 9000).

## Tools

All run from `GroundSpeedKit/` (`swift run -c release <tool>` for speed):

| Tool | Purpose |
|---|---|
| `gsk-checks` | The test suite: protocol, filters, optical flow, height tracking, runtime, recorder, UDP and TCP. Exits non-zero on failure |
| `kfreplay <run_dir> [--filter NAME \| --all] [--out FILE]` | Replays a recorded run through any filter and compares with the phone's `est.csv`. Output defaults to `est_replay_<filter>.csv` inside the run folder; `--all` always writes there. `--synth <dir>` writes a synthetic 10 m run with a flow dropout |
| `phone-sim` | Simulated phone on the production runtime: streams to the dashboard and answers commands |
| `flowbench` | Speed-range benchmark of the correlator on a synthetic floor with motion blur and noise |

`data/tools/summarize_runs.py` lists every recorded run (duration, distance, max speed, GNSS,
height, flow quality) and can compare flow speed with GNSS speed over time for one run.

## Running on an iPhone

Requires an iPhone with LiDAR (Pro models), iOS 17+, and Xcode 15+.

1. Open `ios/GroundSpeed/GroundSpeed.xcodeproj`. The project is generated from
   `ios/GroundSpeed/project.yml`; after adding files, run `xcodegen generate` in that folder.
2. Under the GroundSpeed target's **Signing & Capabilities**, set your team (a free Apple ID
   works) and, if needed, a unique bundle identifier.
3. On the phone, enable **Settings > Privacy & Security > Developer Mode**.
4. Build and run in the **Release** configuration (Product > Scheme > Edit Scheme > Run >
   Build Configuration); a Debug build cannot keep up with 240 fps. On first
   launch, trust the developer under **Settings > General > VPN & Device Management**, then
   allow Camera, Motion, Location and Local Network access.

**Connecting the dashboard.** Turn on the phone's Personal Hotspot and join it from the laptop,
then start `dashboard/run.sh`. The dashboard finds the phone over Bonjour and the phone starts
streaming to it; the LINK light turns on. If discovery fails, enter the phone's IP in the
dashboard's Setup panel (or pass `--phone <ip>`), usually `172.20.10.1`.

**Before the first run:**

1. Mount the phone flat, screen up, camera looking straight down, top of the phone pointing
   forward, about 17–25 cm above the surface (the validated range; below 20 cm the app warns
   that the camera may not focus).
2. With the mount still, press **Measure height**. This measures the height, zeroes the
   distance and learns the IMU bias.
3. Open **Console > Setup > Orientation check** and push forward: flow v_x and IMU a_x must go
   positive. If not, press **Learn** and push forward for 3 s, or use the swap/flip toggles.
4. Push a taped distance and compare. A consistent error of k % is corrected by setting the
   scale factor to 1/(1 + k/100), after re-measuring the height.

## Data

| Path | Contents |
|---|---|
| `data/runs.csv` | Index of the notable runs: setup, course, height, distance read, error and notes |
| `data/phone_runs/<run_id>/` | Full-rate run folders pulled from the phone |
| `data/dashboard_logs/<session>/` | 50 Hz telemetry and command logs as received by the dashboard |
| `data/lidar/` | Every LiDAR height measurement, frame by frame |
| `data/ios_diagnostics/` | iOS crash and resource reports |

Examples:

```bash
python3 data/tools/summarize_runs.py
cd GroundSpeedKit && swift run -c release kfreplay ../data/phone_runs/2026-10-03_22-37-35 --out /tmp/est.csv
```

## Testing

```bash
cd GroundSpeedKit && swift run gsk-checks
python3 -m unittest discover dashboard/tests
xcodebuild -project ios/GroundSpeed/GroundSpeed.xcodeproj -scheme GroundSpeed -destination 'generic/platform=iOS' CODE_SIGNING_ALLOWED=NO build
```

The Swift checks and the Python tests both verify the same binary test vectors from
`docs/PROTOCOL.md`, so the two protocol implementations cannot drift apart.

## Limits

- **Speed.** Without prediction the window allows about `128 · h · fps / f` (6 m/s at 17 cm,
  240 fps). With prediction the limit is motion blur, `v · f · t_exp / h` pixels per frame:
  about 8–11 m/s at 17 cm with 1/1000 s exposure on asphalt at night.
- **Height.** Validated at 17–25 cm. At a 79 cm mount, slow pushes read 15–17 % low: the floor
  moves only about a pixel per frame and static image features bias the peak toward zero.
- **Varying ride height.** Height is measured while stationary, not tracked during a run, so a
  run whose ride height changes reads in proportion to the height error.
- **GNSS.** Phone GNSS speed is good to about ±0.3–0.5 m/s, so it only helps at higher speeds.
