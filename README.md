# Open Ground Speed Sensor — MHacks 2026

An iPhone on a cart becomes an optical ground-speed sensor: camera + torch on the
floor (phase-correlation optical flow), IMU, GNSS and LiDAR height fused by a small
Kalman filter **on the phone**, streamed live to a MacBook dashboard.

| Path | What | Owner |
|---|---|---|
| `docs/PROTOCOL.md` | Frozen contract: UDP frame, TCP commands, run CSV schema | all |
| `docs/KALMAN_INTEGRATION.md` | **How Joseph's Kalman filter plugs in** | Joseph |
| `GroundSpeedKit/` | Swift package: `SpeedProtocol`, `KalmanCore`, `OpticalFlow`, `PhoneRuntime`, tools `kfreplay`, `phone-sim`, `gsk-checks` | David / Joseph |
| `ios/` | SwiftUI iPhone app (XcodeGen project) | David |
| `dashboard/` | Mac telemetry dashboard (Python stdlib + browser UI) | Sean |

## Quick start (no phone needed)

```bash
cd dashboard && python3 -m gsdash          # dashboard → http://localhost:8080
```
```bash
cd GroundSpeedKit && swift run phone-sim --host 127.0.0.1   # simulated phone, real Swift pipeline
```

Checks:
```bash
cd GroundSpeedKit && swift run gsk-checks
```
```bash
python3 -m unittest discover dashboard/tests
```

## On the iPhone

Requires full **Xcode** (not just Command Line Tools). See `ios/README.md`.
