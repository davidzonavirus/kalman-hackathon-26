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
2. **Set the height.** With the cart still, open **Console › Controls › Measure height**.
   - This pauses the flow camera for about 2 s and measures height with LiDAR.
   - If LiDAR is unavailable, measure with a tape and enter it under **Setup › Camera height h**.
   - Flow speed scales linearly with h, so a 1 cm error at 30 cm is a 3% speed error.
3. **Calibrate.** With the cart still for 2 s, open **Console › Controls › Calibrate**. This averages the IMU bias.
4. **Check the orientation.** Open **Console › Setup › Orientation check** and push the cart straight **forward**:
   - **flow v_x** must go **positive** (teal). If it goes negative, or the motion shows up in v_y, press **Learn** and push forward for 3 s, or toggle Swap / Flip x / Flip y until it does.
   - **imu a_x** must spike **positive** as you start pushing. If it doesn't, toggle the IMU mapping. Swap x/y means the phone's side edge points forward. Flip x means the phone's top points backwards.
   - Push to the **left**: v_y should be positive (vehicle y = left).
5. **Check the main screen.** At rest, v_x should read about 0.00 and **IMU, FLOW** should be teal. Push about 1 m/s and check that v_x is about +1.0.
6. **Run a sanity check.** Push exactly 10.00 m (tape on the floor), stop, and compare the distance. If it is consistently off by k%, set **Setup › Flow › Scale factor** to 1/(1+k/100). Better still, re-measure h first.

## Status lights

| Light | Teal | Vermilion | Hollow |
|---|---|---|---|
| IMU | samples < 50 ms old | not arriving | — |
| FLOW | fresh, PSR ≥ psr_min, not gated | stale / low texture / gated | — |
| GNSS | fix < 2 s with speed accuracy | — | no fix (normal indoors) |
| LiDAR | depth < 500 ms (during Measure height) | — | using manual/measured h |
| LINK | dashboard connected on TCP | — | no dashboard |

## Known limitations

- **LiDAR height is measured, not streamed during runs.** iOS can't run the LiDAR depth session and the 120 fps flow session at the same time. LIDAR_OK is therefore only on during **Measure height**. The measured value becomes the manual h used for flow.
- **Background stops the run.** When the app goes to the background, the camera stops, so the app stops the active run cleanly.
- **Focus locks after start.** Focus and exposure lock about 1.5 s after the camera starts, with the torch on. If you change the mount height, the lock is redone automatically when you apply a new h.
