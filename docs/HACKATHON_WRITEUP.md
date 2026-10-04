# Open Ground Speed Sensor: MHacks 2026 write-up

**An iPhone pointed at the ground becomes an optical ground-speed sensor.** The camera watches the road or floor move. LiDAR measures the camera's height. The IMU and GPS fill the gaps. A Kalman filter running on the phone fuses all of it into speed and distance, which streams live to a dashboard on a MacBook.

Commercial optical ground-speed sensors (Kistler Correvit, Datron) cost thousands of dollars. They're standard kit for vehicle dynamics testing and FSAE: slip angle, wheel slip, true distance. This project does the core job with a phone you already own.

---

## 1. Headline results

| Test | Setup | Result |
|---|---|---|
| Cart, 16 ft (4.877 m) taped course, 6 runs | Mount heights 18.6 cm and 23.5 cm, indoor floor, including one run in different lighting | **All six within ±0.4 %** of tape after calibration (+0.05, +0.30, −0.33, −0.02, +0.36, −0.36 %) |
| Repeatability | Runs 5–7, same height | Three runs within 1 % of each other *before* calibration |
| Car, Mazda CX-5 AWD, night, compared with GPS speed | 17 cm mount, 240 fps, 1/1000 s exposure | **10–15 mph: 0.97× GPS (±3 %). 15–20 mph: 1.01× GPS.** Consistent readings up to about 25 mph |
| Top speed | Same | Readings hold to **about 25 mph (11 m/s)**. Around 30 mph the image blurs too much at night and flow drops out |
| Before the car-speed work | Same | Capped at about 6 m/s (13 mph), and read 3× low because of a height-tracking bug |

Low-speed car comparisons (under 10 mph) scatter by ±10–30 %, but that's mostly the GPS, not us: phone GPS speed is only good to about ±0.3–0.5 m/s, which is a big fraction of 2–4 m/s.

---

## 2. How it works

```
 iPhone (all processing on-device, ~0.5–0.8 ms per frame)
 ┌───────────────────────────────────────────────────────────────────────────┐
 │ Wide camera 1280×720 @ 240 fps, exposure ≤ 1/1000 s, torch, focus locked   │
 │   └─ 256×256 centre crop → 2× downsample → 128×128 phase correlation       │
 │        → image shift (px/frame) + quality score (PSR)                       │
 │        → v = shift · ds · h / (f · dt)   (tilt-corrected with gravity)     │
 │ LiDAR (before a run) → camera height h above the ground                     │
 │ IMU 100 Hz (accel + gyro)   GNSS ~1 Hz (speed + speed accuracy)             │
 │                                                                             │
 │ 4-state Kalman filter  x = [v_x, v_y, b_x, b_y]                             │
 │   predict: IMU accel (minus bias) + gyro yaw-rate coupling                  │
 │   update:  flow velocity (R scales with 1/PSR²), GNSS speed > 1 m/s,        │
 │            zero-velocity updates (ZUPT) when still                          │
 │   → distance integrator (lag-compensated smoothing)                        │
 │   → run recorder: flow.csv, est.csv, imu.csv, gnss.csv, depth.csv, events   │
 └──────────────┬──────────────────────────────────────────────┬──────────────┘
                │ UDP 9000, 50 Hz binary telemetry (CRC)        │ TCP 9001 JSON commands
                ▼                                               ▼
 MacBook dashboard (Python stdlib + browser): live speed, distance, quality, h,
 runs table (distance, max, mode speed, error vs. tape), CSV export,
 Measure h / Zero / Start / Stop / torch buttons, Bonjour auto-discovery
```

### Optical flow: phase correlation

- Each frame, a 256×256 px patch at the image centre is box-downsampled to 128×128, Hann-windowed and FFT'd. The cross-power spectrum with the previous frame is whitened and inverse-FFT'd. The result is a sharp peak at the shift between the two frames.
- **Sub-pixel accuracy** comes from a Gaussian fit around the peak. The quality score is the **PSR** (peak-to-sidelobe ratio): about 3–6 for noise, tens to hundreds for good texture.
- **Velocity:** `v = shift × downsample × h / (f × dt)`. Here `h` is the camera height, `f` the focal length in pixels and `dt` the frame interval. Gravity from the IMU corrects for a tilted mount.
- **Motion prediction (added for the car):**
  - The current frame's crop is offset by the last confident shift, so the 128 px window only has to cover the *change* in motion between frames.
  - Without prediction, the window wraps at ±64 downsampled px per frame, about 6 m/s at a 17 cm mount.
  - The sensor's own fixed pattern (which sits at zero motion) is masked out of the correlation surface when the prediction is large.

### Height: LiDAR

- A **Measure h** button (on the phone or the dashboard) pauses the camera for about 2 s and samples LiDAR depth across the centre of the frame.
- It converts the line-of-sight depth to vertical height using the gravity vector, rejects inconsistent readings, and zeroes the IMU bias at the same still moment.

### Fusion: Kalman filter

- The filter is a 4-state Kalman filter (Joseph's design, `ReferenceKF4`; see `docs/KALMAN_INTEGRATION.md`). The algorithm is specified step by step in the source, so a port in another language can reproduce it to 1e-6. Replaying a run's logged sensors through it reproduces the phone's `est.csv`.
- **Flow measurements** are weighted by quality, and outliers are rejected with an innovation (NIS) gate. A lockout recovery reopens the gate after 12 straight rejections.
- **GNSS speed** is only used above 1 m/s.
- **ZUPT:** when the IMU says the phone is still, velocity is pinned to zero, which kills drift.

---

## 3. The debugging story

These are in the order we hit them, over about 8 hours of testing (Oct 3, 19:30 → Oct 4, 02:15). Each one was found from logged data: the phone records every sensor stream to CSV, and we pulled the runs over USB with `xcrun devicectl` to analyse them in Python.

### 3.1 Ride height gave "inconceivable" readings: LiDAR tilt counted twice
- **Symptom:** distances were wildly off, and none of the first seven runs had a LiDAR reading at all, so they all used the 0.3 m default.
- **Root cause:** LiDAR measures distance *along the camera's line of sight*. On a tilted phone that's longer than the vertical height: 0.133 m at 39° when the real height was about 0.104 m. The app stored the slant distance as the height, and the flow math corrected for tilt *again*. Speed read about 64 % high, and the error changed with the mount angle.
- **Fix:** convert depth to vertical height once. Reject readings where frames disagree by more than 8 % / 1 cm, depth isn't metric, or the result is outside 5 cm–2 m. Log every attempt.

### 3.2 GPS "±5 m"
- This was a misunderstanding, not a bug. ±5 m is *position* accuracy, which phones can't beat. The filter uses GNSS *speed*, which was about ±0.3 m/s. The dashboard now shows speed accuracy instead.

### 3.3 Frame rate: the app was dropping a quarter of the frames
- **Symptom:** quality fell from about 200 at rest to about 50 at 2 m/s.
- **Root cause:** the app was an unoptimised Debug build processing about 89 of 120 frames a second. Fewer frames means more motion between frames and less overlap.
- **Fix:** build in Release and move the camera to 240 fps. The result was 238 frames a second processed at under 0.5 ms each, with 1 % dropped. That doubled the speed range.

### 3.4 Dashboard couldn't see the phone: IPv6-only hotspot
- The T-Mobile hotspot gave the Mac no IPv4 address, and the dashboard only listened on IPv4. We made the UDP receiver dual-stack and added Bonjour discovery that tries every address the phone advertises.

### 3.5 Jostling the phone added 10–15 % distance
- When the phone wobbles on its tape mount, the lens really does swing over the floor, and the odometer counted every swing.
- **Fix:**
  - Distance integrates velocity smoothed over about 0.5 s, so oscillations cancel. In the test, 10 s of simulated wobble went from adding 3.38 m to 0.013 m, while a clean push still read within 0.5 %.
  - The filter trusts flow less while the gyro shows rotation.

### 3.6 "Flow lost" and runaway distance after Measure h: iOS camera service crash
- **Symptom:** after pressing Measure h, flow never came back. The IMU-only estimate drifted, reading 15 m in 3 s with the phone untouched.
- **Root cause, from iOS crash logs pulled over USB:** `cameracaptured`, iOS's camera service, crashed every time. LiDAR leaves the camera in a 30 fps mode, and asking for 240 fps immediately afterwards aborts the camera daemon. The phone was also at thermal state "serious".
- **Fix:**
  - Restart the camera at 30 fps and step up to 240 once it's running.
  - A watchdog restarts the camera if no flow arrives for 1.5 s.
  - Drop to 120 fps under thermal pressure.
  - Once flow has been gone for 2 s, any still IMU moment forces velocity to zero, so a dead camera can't add distance.
  - Measure h now also does the Zero, so the procedure became one button.

### 3.7 Reading 7 % high on every run: focal length from the wide lens
- **Runs 5–7** (16 ft at 18.6 cm) read 5.235, 5.246 and 5.214 m: +7 %, very repeatable.
- **Not wobble, speed or the filter:** path length and net forward distance agreed, a 1 m/s run had the same error, and raw flow integrated to the same 5.27 m.
- **Root cause:** at 240 fps iOS doesn't deliver per-frame lens intrinsics, so the focal length came from the field-of-view figure. Barrel distortion magnifies the image centre, where the flow patch sits.
- **Two-parameter calibration**, after runs 9–11 at 23.5 cm read −1.0, −0.6 and −1.3 % with the first fix (run 11 was in different lighting and no worse):
  - focal correction ×1.027
  - LiDAR height offset −7.9 mm (LiDAR sits higher than the camera's optical centre)
- **Result:** all six runs within ±0.4 %. A lens-only correction would have drifted to −2 % at 50 cm.

### 3.8 The extreme mount (79 cm / 31 in): a fix that made things worse
- **Runs 15–17** read 15–17 % low. LiDAR read 0.792–0.798 m against the tape's 0.787 m, so height wasn't the problem.
- **Theory:** at walking pace and 79 cm the floor moves only about 1 px per frame, and anything static in the image (sensor pattern, vignetting) pulls the reading toward zero.
- We added static-pattern subtraction. It helped in simulation (−8.4 % → −0.4 %), but **run 18 read −31 %** on the real cart.
- **Decision:** revert, and accept that the sensor is specified for low mounts (validated at 19–25 cm, used at 17 cm on the car). It's a real lesson: synthetic benchmarks don't capture everything about real sensors.

### 3.9 Dashboard hijacked by the iOS Simulator
- A copy of the app running in Xcode's simulator advertised itself on Bonjour, and the dashboard connected to it: all zeros and the default 30 cm height. We killed the simulator, and now we check for this first.

### 3.10 Variable ride height: image-expansion height tracking
- **Ride-height runs 1–3** (height changed mid-run) read −6.2, −13.7 and −24.6 %, because height was fixed at the starting value.
- **The idea:** four extra correlation patches (left, right, top, bottom) measure image *expansion*. A closer floor looks bigger.
- **The maths:** a tilted camera moving along the floor also sees apparent expansion, so we used `Δh/h = A∥ − 2·A⊥` (expansion along the motion minus twice the expansion across it), which cancels tilt exactly.
- **Results:**
  - Synthetic rendered floors: within 0.5 % with tilt, and tracked 0.20 → 0.24 m height changes to 1 %.
  - Ride-height run 4 read −3.4 %. Run 5 swung wildly.
  - On the car it **failed badly** (see 3.13 and 3.15), so it's **off** in the final build (`CameraFlowSource.tracksHeight = false`). A car's ride height barely changes, so LiDAR at the start is enough.

### 3.11 Dashboard distance jumped +4 m at the end of a run
- When the cart stopped, the phone's distance dipped by 2.5 cm in one step. A lag-compensation term (`0.5 s × v`) switched off abruptly below 0.05 m/s.
- The dashboard treated any drop over 5 mm as "phone was zeroed" and re-added the phone's whole distance. Replaying run 5 through the old logic gave 8.07 m; the new logic gives 4.05 m.
- **Fixes:**
  - The phone fades the term out smoothly.
  - The dashboard only treats a drop to near zero as a reset.
  - The big readout holds the finished run's distance after Stop.

### 3.12 "No signal" after 1 AM: a regex and a timestamp
- The dashboard parsed `dns-sd` output, where each line starts with a timestamp. Before 10 AM the hour is one digit, so the line starts with a **space**, and the regex required a non-space first character.
- Discovery silently failed from 1:00 to 9:59 AM. It worked at 23:36 and died after midnight.
- **Fixes:**
  - Fixed the regex.
  - Stale UDP-source addresses expire after 3 s.
  - Bonjour targets are dropped after 3 failed connections.
  - Added regression tests.

### 3.13 First car test: 10 mph read 1.5 m/s, 20 mph stuck at 6 m/s
- **Run `2026-10-04_01-19-31`**, compared with GPS in 2 s windows:
  - The **height tracker collapsed** from 0.173 m to 0.058 m (it was clamped at 1/3 of the anchor) within 4 s, so speed read 0.31× GPS.
  - **Window wrap:** at GPS ≈ 6.1 m/s quality fell to the floor. At 8.5–8.8 m/s flow was lost, because the shift exceeded the ±64 px correlation window and wrapped.
  - **Mount knocked at t ≈ 123 s:** 2.4 rad/s gyro and 4 g spikes. Afterwards flow read 0 with good quality while GPS said 3–7 m/s, because the camera was no longer seeing the road.
- **Fix:** motion prediction (section 2). On the bench, the reliable speed went from **3.0 m/s to 9.0 m/s** at a 17 cm mount, and PSR at 6 m/s went from 7 to 42.
- **A bug the bench caught:** the fixed-pattern mask erased the true peak whenever the predicted motion was an exact multiple of the window (128 px ≈ 12 m/s), because both peaks wrap to the same place. It now skips masking when they coincide.

### 3.14 Exposure trade-off
- Benchmarks with a realistic prediction error (±4 px) at 17 cm:
  - 1/1000 s: about 7.5 m/s fully reliable.
  - 1/2000 s: about 11 m/s, with 95 % of frames good at 15 m/s.
  - 1/4000 s: about 21 m/s.
- At night the camera was already at ISO 2200. Halving the exposure halves the light, and the bench showed no net gain once darkness is modelled. Daylight shortens the exposure automatically, because the cap only limits how long it can get.

### 3.15 Second car test: 20 mph read 10–11 mph, 30 mph read zero
- **Run `2026-10-04_01-45-16`:**
  - **The first 40 s were great:** flow matched GPS within ±10 % up to 8 m/s, which confirmed motion prediction works.
  - **Height tracker drift, again:** at 1.5–2.5 m/s (below its speed gate) it drifted down to its new 1/1.5 clamp (0.110 m). Everything after read 0.67× GPS, which matches "20 mph reads 10–11".
  - **Zero lock:** at t = 163.00 s one frame dipped to PSR 7.8 (the mount vibrates at about 3 g). The re-acquisition logic tried "zero motion" on the very next frame, locked onto the fixed pattern at PSR 13.7, and overwrote the remembered speed with 0. Every candidate after that was a multiple of zero, so it read 0 from 7.8 to 12.8 m/s.
- **Fix:**
  - Height tracking turned off.
  - Coast on the last prediction for 24 frames (0.1 s) before trying other guesses.
  - Reject weak results (PSR < 30) that jump far from the last good motion.

### 3.16 Third car test (final): consistent to 25 mph
- **Run `2026-10-04_01-59-42`:** height steady at 0.171 m.
  - 10–15 mph: est/GPS = 0.971 ± 0.032.
  - 15–20 mph: 1.015.
  - You saw consistent readings up to 25 mph.
- **Around 30 mph (12.5 m/s):**
  - Quality sat at the floor (PSR about 11) from 7.6 m/s up. That's motion blur: 12.5 m/s at 17 cm with a 1/1000 s exposure smears about 65 px.
  - Once the track was lost, the coasting allowance from 3.15 kept widening. It eventually accepted a weak, exactly-zero result, and then stayed locked at 0 even after the car slowed to 5–6 m/s.
- **Final fix (in the build on the phone, not yet driven):** an exactly-zero result (under 1.5 px) while the last tracked motion was fast (over 8 px/frame) needs PSR ≥ 30. A car can't stop during a lost-track interval, a real stop reads PSR 100–500, and the fixed pattern peaks at 10–25. Braking to a stop still works: tracking follows the shift down smoothly, and the rule stops applying once the motion is under 8 px/frame.

---

## 4. Calibration constants (final)

| Constant | Value | Where | From |
|---|---|---|---|
| `fovFocalCorrection` | 1.027 | `CameraFlowSource.swift` | Joint fit, six 16 ft runs at 0.186 and 0.235 m |
| LiDAR height offset | −7.9 mm (likely range 5–11 mm) | `DepthSource.swift` | Same fit |
| Max exposure | 1/1000 s | `CameraFlowSource.maxExposure` | Blur vs. noise at ISO 2200 |
| Correlator | 128², downsample 2, PSR min 8 | `PhaseCorrelator.swift` | Bench + cart runs |
| Tracking | prediction only above 32 px/frame; coast 24 frames, relock PSR 30, fake-stop rule | `CameraFlowSource.swift` | Car runs 2 and 3; slow pushes keep the calibrated un-predicted path |
| Filter | q_accel 0.1, q_bias 1e-5, r_flow_base 1.6e-3, gate 9 | `GroundSpeedFilter.swift` | Joseph's reference filter |

---

## 5. Speed limits: the physics

- **Window limit (without prediction):** ±64 downsampled px per frame, so `v_max ≈ 128 · h · fps / f`. That's about 6 m/s at h = 0.17 m and 240 fps. Prediction removes this limit.
- **Blur limit (with prediction):** blur in px = `v · f · t_exp / h`. Tracking degrades past about 40–50 px of blur on asphalt at night, so about 8–11 m/s at 17 cm with 1/1000 s.
- **Everything scales with h:** a 35 cm mount doubles the range. Under thermal throttling (120 fps) the window limit halves.
- **Ways to go faster:** daylight (shorter exposure), a higher and stiffer mount, or a brighter light.

---

## 6. What didn't work (and what we learned)

| Idea | Simulated | Real | Verdict |
|---|---|---|---|
| Static-pattern subtraction for high mounts | −8.4 % → −0.4 % | −31 % | Reverted; the sensor is specified for low mounts |
| Image-expansion height tracking | Within 0.5–1 % | Cart OK-ish; car collapsed (twice) | Off by default; LiDAR at start instead |
| Shorter exposure at night | +40 % range at equal brightness | Image too dark, no gain | Rely on daylight |

**The lesson:** every fix was checked against logged real data, not just the simulator. The bench (`flowbench`) was right about window wrap and blur, but wrong about two physical effects. Those only showed up because every run is recorded in full.

---

## 7. Tooling we built along the way

| Tool | What it does |
|---|---|
| `swift run -c release gsk-checks` | 166 automated checks: filter maths, flow accuracy, tilt, prediction, height tracker, protocol, TCP server, recorder |
| `swift run -c release flowbench [--predict] [--exposure s] [--h m]` | Runs the real correlator on synthetic textured floors at known speeds with blur and noise; prints the max reliable speed |
| `swift run kfreplay <run_dir>` | Re-runs the Kalman filter on a recorded run (any filter or config) and compares with the phone's `est.csv` |
| `swift run phone-sim` | Simulated phone using the real Swift pipeline, for dashboard work without hardware |
| `python3 data/tools/summarize_runs.py [RUN_ID]` | Index of every recorded run, or GPS-vs-flow speed over time for one run |
| Dashboard | Live telemetry, Measure h / Zero / Start / Stop, runs table with distance, max, **mode** (cruise) speed, error vs. tape, CSV export |
| Launch flag `-gskAutoMeasure` | Takes a LiDAR height reading when launched over USB with `xcrun devicectl`, so the Mac can test the phone hands-off |

---

## 8. Running it

1. **Phone:** build `ios/GroundSpeed` in Xcode (Release scheme), or use `xcodebuild` (see `ios/README.md`).
2. **Network:** join the Mac to the phone's hotspot, then run `dashboard/run.sh` and open http://localhost:8080. The phone is found automatically.
3. **Mount:** mount the phone rigidly 15–30 cm above the ground, camera down.
4. **Measure h:** press **Measure h** with the vehicle still. This also zeroes the IMU.
5. **Record:** wait for **Flow** to turn teal, then press **Start**. Drive or push, then press **Stop**.
6. **Pull the logs:** plug in USB and pull the full-rate logs (see `data/README.md`).

---

## 9. Known limitations / future work

- **Night top speed is about 25 mph** at a 17 cm mount, limited by blur. Daylight and a higher mount should roughly double it. The final fake-stop rule should stop the 30 mph reading from sticking at zero, but it hasn't been driven yet.
- **Mount vibration** (about 3 g peaks) causes quality dips. A stiffer mount helps directly.
- **High mounts** (about 80 cm) read 15–17 % low at walking pace; unsolved.
- **Ride height** is measured once per run. Dynamic height (suspension travel) would need a second rangefinder or a better expansion estimator.
- **Per-frame lens intrinsics** aren't available at 240 fps, so we calibrate the focal length per phone model.

---

## 10. Data

Every run recorded during the hackathon is in **`data/`**: 55 phone runs, 21 dashboard sessions, 32 LiDAR height logs and the relevant iOS crash reports. `data/README.md` explains which run is which (cart 16 ft runs, the extreme-height runs, the ride-height runs and the three car tests) and how to replay them.
