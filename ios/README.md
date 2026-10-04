# GroundSpeed iOS app

This is the on-phone half of the Open Ground Speed Sensor. It runs on an iPhone (Pro, with LiDAR) mounted on a cart, flat with the screen up, so the back camera looks at the floor.

The app fuses three inputs with the KalmanCore filter on the phone:

- IMU at 100 Hz.
- Optical flow from the back camera at about 120 fps, with the torch on.
- GNSS speed.

The camera height comes from LiDAR or from a manual setting.

It logs every run to CSV under `Documents/runs/<run_id>/`. It streams telemetry over UDP to the dashboard on port 9000 and accepts commands on TCP port 9001 (Bonjour `_groundspeed._tcp`). See `docs/PROTOCOL.md` for the wire format.

The app works without a laptop. Record, Mark and export (share sheet / AirDrop) are all on the phone.

## Layout

```
ios/GroundSpeed/
  project.yml                 XcodeGen spec (source of truth for the Xcode project)
  GroundSpeed.xcodeproj       generated: `xcodegen generate`
  GroundSpeed/
    GroundSpeedApp.swift      @main, keeps the screen awake
    AppModel.swift            wires the iOS sensors into PhoneRuntime.GroundSpeedRuntime
    Theme.swift               warm-neutral datasheet palette + type
    Sensors/                  MotionSource (CoreMotion), LocationSource (CoreLocation),
                              CameraFlowSource (AVFoundation → PhaseCorrelator),
                              DepthSource (LiDAR height measurement)
    Views/                    MainView, ConsoleSheet (controls / setup), RunsView
```

Everything that isn't platform-specific lives in `GroundSpeedKit/Sources/PhoneRuntime`:

- The fusion engine.
- The recorder.
- UDP telemetry and the TCP command server.
- Settings.

That code is tested on the Mac with `swift run gsk-checks` and `swift run phone-sim`.

## Build and run on the phone

1. **Install Xcode** (version 15 or newer, with the iOS 17+ SDK). Then accept the license once in Terminal:
   `sudo xcodebuild -license accept`
   and run `xcodebuild -runFirstLaunch` to install the platform components.
2. **Generate the project.** Do this again whenever files are added or `project.yml` changes:
   ```bash
   brew install xcodegen         # once
   cd ios/GroundSpeed
   xcodegen generate
   open GroundSpeed.xcodeproj
   ```
3. **Set up signing.** In Xcode, select the **GroundSpeed** target, then **Signing & Capabilities**. Set **Team** to your Apple ID ("Add Account…" if needed). A free personal team works.
   - If the bundle id `com.mhacks26.groundspeed` is taken, change it to something unique, for example `com.<you>.groundspeed`.
   - You can also put your team id in `project.yml` under `DEVELOPMENT_TEAM:` so regenerating keeps it.
4. **Prepare the iPhone.** Connect it by USB. Open **Settings › Privacy & Security › Developer Mode**, turn it on, and reboot.
5. **Build and run.** Choose your iPhone as the run destination and press **⌘R**.
6. **Trust the developer.** The first launch with a free team fails with "Untrusted Developer". On the phone, go to **Settings › General › VPN & Device Management**, select your Apple ID, then **Trust**. Launch again.
7. **Allow permissions** when asked: Camera, Motion & Fitness, Location (While Using), and Local Network. All of them are needed.

Command-line build for the simulator (UI only, since the simulator has no camera or IMU):
```bash
xcodebuild -project ios/GroundSpeed/GroundSpeed.xcodeproj -scheme GroundSpeed \
  -destination 'generic/platform=iOS Simulator' build
```

## Network: phone hotspot + laptop dashboard

1. On the iPhone, open **Settings › Personal Hotspot** and turn on **Allow Others to Join**. Keep that screen open while the laptop joins.
2. Join the laptop to the phone's hotspot over Wi-Fi. The laptop usually gets an address like `172.20.10.2`.
3. Start the dashboard on the laptop. It listens on UDP 9000 and connects to the phone on TCP 9001, finding it via Bonjour or the phone IP, which is usually `172.20.10.1`.
4. When the dashboard connects over TCP, the phone automatically sends its UDP telemetry to that laptop. **LINK** turns teal.
5. Without a TCP connection, set the laptop's IP in **Console › Setup › Dashboard IP** and press **Apply**. To find it, run `ipconfig getifaddr en0` on the laptop.
6. The first time, iOS asks for **Local Network** permission. It must be allowed. You can change it later in **Settings › GroundSpeed**.

Without a laptop, everything still works. Press Record and Stop, then use **Console › Runs** to export a run as a zip (AirDrop, Files). Runs are also visible in the Files app under **On My iPhone › GroundSpeed › runs**.

## On-site setup and orientation check (do this before the first real run)

1. **Mount the phone.** Mount it flat with the screen up and the back camera looking straight down at the floor. The top of the phone should point in the cart's forward direction, which is the default mapping. Keep the camera at least about 25 cm above the floor, because the wide camera can't focus closer.
2. **Set the height.** With the cart still, open **Console › Controls › Measure height**, or press **Measure h** on the dashboard.
   - This pauses the flow camera for about 2 s and measures height with LiDAR. It also zeroes the distance and re-learns the IMU bias over the same still window, so step 3 is only needed if you skip this one.
   - The result is the **vertical** height: the LiDAR range along the lens axis times cos(tilt) from gravity. It is rejected (manual h kept) if the frames disagree by more than max(8%, 1 cm), or if the height is outside 5 cm–2 m.
   - Every attempt is logged frame by frame to `Documents/lidar/<stamp>.csv`. To measure from the Mac over USB: `xcrun devicectl device process launch --device <udid> --terminate-existing com.mhacks26.groundspeed -- -gskAutoMeasure`, then copy `Documents/lidar` off with `xcrun devicectl device copy from … --domain-type appDataContainer`.
   - If LiDAR is unavailable, measure with a tape and enter it under **Setup › Camera height h**.
   - Flow speed scales linearly with h, so a 1 cm error at 30 cm is a 3% speed error.
3. **Calibrate.** With the cart still for 2 s, open **Console › Controls › Calibrate**. This averages the IMU bias.
4. **Check the orientation.** Open **Console › Setup › Orientation check** and push the cart straight **forward**:
   - **flow v_x** must go **positive** (teal). If it goes negative, or the motion shows up in v_y, press **Learn** and push forward for 3 s, or toggle Swap / Flip x / Flip y until it does.
   - **imu a_x** must spike **positive** as you start pushing. If it doesn't, toggle the IMU mapping. Swap x/y means the phone's side edge points forward. Flip x means the phone's top points backwards.
   - Push to the **left**: v_y should be positive (vehicle y = left).
5. **Check the main screen.** At rest, v_x should read about 0.00 and **IMU, FLOW** should be teal. Push about 1 m/s and check that v_x is about +1.0.
6. **Run a sanity check.** Push exactly 10.00 m (tape on the floor), stop, and compare the distance. If it is consistently off by k%, set **Setup › Flow › Scale factor** to 1/(1+k/100). Better still, re-measure h first. Two corrections are already fitted from six 16 ft pushes at LiDAR h = 0.186 m and 0.235 m, with all runs within ±0.4%. `CameraFlowSource.fovFocalCorrection` = 1.027 is the lens distortion at the centre crop. `DepthSource.heightOffset` = −7.9 mm is the LiDAR origin vs. the camera's projection centre. If a new mount height reads consistently off, refit both with that height's runs included.

**Ride-height changes.** LiDAR can't run alongside the 240 fps camera, so Measure h only gives a starting height. Between measurements, `HeightTracker` follows the height from image expansion: four extra patches 448 px left/right and 224 px up/down of centre give the flow gradient. Δh/h = A∥ − 2·A⊥ (along and across the direction of motion) cancels the apparent expansion that a tilted camera sees while moving, which would otherwise drift tens of % per metre. It costs about 0.8 ms/frame. The live h appears in telemetry (dashboard h trace), is re-anchored by every Measure h, and random-walks about 1–2% per metre on rendered floors.

**Mount height.** Distance was within ±1% at 19–25 cm. At 79 cm (31 in) it read 15–17% low: the floor moves only about 1 px per frame at walking pace, and anything fixed in the camera frame (sensor pattern noise, vignetting) pulls the measured shift toward zero. Subtracting a learned static image made it worse on device (−31%), so it was dropped. `flowbench --fpn 3 --contrast 0.25 --h 0.79` reproduces the bias.

**Speed range.** The correlator offsets the current crop by the last confident shift (motion prediction), so the 128 px window only has to cover the frame-to-frame *change* in motion. When the track is lost it keeps the last good prediction for 24 frames, then cycles through 0.5×, 1.5×, 0.75×, 1.25×, 0 and 1× of it. A weak peak (PSR < 30) far from the last good motion is rejected as the sensor's fixed pattern. Motion blur sets the limit (`swift run -c release flowbench --predict --predict-err 4`). At h = 0.173 m and 240 fps it's about 11 m/s (25 mph) fully reliable at 1/2000 s exposure, 95 % of frames at 15 m/s, and about 7.5 m/s at 1/1000 s. The phone shortens the exposure on its own in daylight; at night it sits at the 1/1000 s cap. The range scales with h × fps/240, so a higher mount raises it proportionally and thermal throttling (120 fps) halves it. Ride-height tracking (`CameraFlowSource.tracksHeight`) is off: on the car it drifted to its clamp and made speed read 0.67× low.

## Status lights

| Light | Teal | Vermilion | Hollow |
|---|---|---|---|
| IMU | samples < 50 ms old | not arriving | — |
| FLOW | fresh, PSR ≥ psr_min, not gated | stale / low texture / gated | — |
| GNSS | fix < 2 s with speed accuracy | — | no fix (normal indoors). The filter only uses GNSS speed above 1 m/s; the "pos ±N m" on the main screen is position accuracy, which a phone can't get below ~3–5 m and which the speed estimate never uses |
| LiDAR | depth < 500 ms (during Measure height) | — | using manual/measured h |
| LINK | dashboard connected on TCP | — | no dashboard |

## Known limitations

- **Camera restarts slowly after LiDAR.** If the flow session restarts straight at 240 fps after the LiDAR session, iOS's camera daemon (`cameracaptured`) crashes. The flow camera therefore restarts at 30 fps and steps up once it's running. A watchdog restarts the session if no flow arrives for 1.5 s (`camera_restart` event). Each restart logs a `camera_started` event with frame and flow counts.
- **Heat.** 240 fps plus the torch heats a taped-up phone to thermal "serious". When iOS reports serious or critical camera pressure, the camera drops to 120 fps and logs a `camera_pressure` event. It goes back to 240 fps when the pressure clears.
- **No flow means no speed.** IMU-only dead reckoning drifts. Once flow has been gone for 2 s, any still IMU window zeroes the velocity, so a dead camera can't keep adding distance.
- **LiDAR height is measured, not streamed during runs.** iOS can't run the LiDAR depth session and the 240 fps flow session at the same time. LIDAR_OK is therefore only on during **Measure height**. The measured value becomes the manual h used for flow.
- **Background stops the run.** When the app goes to the background, the camera stops, so the app stops the active run cleanly.
- **Focus locks after start.** Focus and exposure lock about 1.5 s after the camera starts, with the torch on. If you change the mount height, the lock is redone automatically when you apply a new h.
