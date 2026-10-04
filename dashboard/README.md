# Ground speed dashboard

Live telemetry screen for the MacBook. Python 3 standard library only: no pip, no
internet, no CDN. It receives the phone's 50 Hz UDP frames, logs them to CSV, serves a
web UI, and forwards button presses to the phone's TCP command port.
The wire format is in `docs/PROTOCOL.md`.

```
iPhone ──UDP 9000 (telemetry)──▶ gsdash ──HTTP/SSE 8080──▶ browser
       ◀─TCP 9001 (commands)──── gsdash ◀─POST /api/cmd──
```

## Quick start

```sh
dashboard/run.sh                  # stops any running gsdash first
open http://localhost:8080
```

Use `run.sh` rather than `python3 -m gsdash` directly: it kills any dashboard that's
already running, since two instances fight over UDP 9000 and the phone's command link.

## Doing a test run

1. **Measure h**, with the cart parked and nobody touching it for about 3 s. This one
   button does three things: measures ride height with LiDAR, zeroes the distance,
   and re-learns the IMU bias. You don't need to press Zero or Calibrate separately.
   The button shows `h 18.2 cm` when it worked. If it shows `h failed`, hover over it
   to see why (usually the cart moved), then press it again.
2. **Wait for the Flow dot to go teal.** The camera pauses during the measurement and
   comes back about 2 s later.
3. **Start tracking**, push the cart, stop it, wait about 1 s, then **Stop tracking**.
   Start resets the distance itself. If Flow isn't OK when you press Start, the
   dashboard asks first, because the distance would come from the IMU alone.

Measure h again whenever the mount changes. Zero is only for resetting the distance
in the middle of a session, with the cart still.

To try it without a phone, run the fake phone in a second terminal:

```sh
cd dashboard
python3 -m gsdash.sim             # binary frames, 10 m run on a loop
python3 -m gsdash.sim --json      # JSON debug frames
python3 -m gsdash.sim --v1        # legacy 48-byte v1 frames (default is v2)
python3 -m gsdash.sim --loss 0.05 # drop 5 % of packets to exercise the gap counter
./run.sh --sim                    # dashboard and sim together
```

The Swift simulator should also work: `cd GroundSpeedKit && swift run phone-sim --host 127.0.0.1`.

## At the venue

1. **Network.** On the iPhone, turn on Settings → Personal Hotspot ("Allow Others to
   Join"). Join that Wi-Fi from the MacBook. The phone is usually `172.20.10.1` and
   the laptop gets `172.20.10.x`. Venue Wi-Fi often blocks client-to-client traffic,
   so use the hotspot.
2. **Start the dashboard:** `cd dashboard && ./run.sh`. The first time, macOS asks
   *"Do you want the application python3 to accept incoming network connections?"*.
   Click **Allow**. Without it no UDP arrives. If you clicked Deny, fix it under
   System Settings → Network → Firewall → Options.
3. **Open http://localhost:8080** and press **F** for fullscreen on the projector.
4. **Connect the phone.** The phone sends UDP to the host configured in the app. When
   the dashboard connects to the phone's TCP 9001, the phone sends UDP to that laptop
   from then on. The dashboard chooses the phone IP in this order:
   * `--phone IP` on the command line, or the **phone IP** field in the UI (Enter to
     set; leave it empty and press Set to go back to auto)
   * otherwise the source IP of incoming UDP telemetry
   * otherwise Bonjour (`dns-sd -B _groundspeed._tcp`, best effort)

   If nothing shows up, type `172.20.10.1` into the phone IP field. You can also start
   with `./run.sh --phone 172.20.10.1`. The dashboard then connects to the phone,
   which starts streaming to the laptop.
5. **USB-C fallback.** If Wi-Fi is bad, plug the iPhone into the Mac with the hotspot
   on. It shows up as a wired "iPhone USB" network interface. The addresses are the
   same (`172.20.10.x`), so nothing else changes. Switch Wi-Fi off on the Mac if it
   picks the wrong route.
6. To view the UI from another device (a second laptop, for example), use
   `./run.sh --bind 0.0.0.0`. The UI is then served on all interfaces.

## The screen

The layout is a quiet instrument datasheet: warm neutral colours, one teal accent for
"live/ok" and one vermilion for "something is wrong". Light is the default; the
**Dark** link in the header switches theme and the choice is remembered. Fonts are
macOS-installed only: Iowan Old Style or Charter for labels, PT Mono or Menlo for
every number.

* **Hero row**:
  * **Distance travelled** (total ∫|v| dt since the last zero or run start, in m or ft
    per the m/ft toggle), with **net forward** (signed displacement along x) below it.
  * **Speed |v|** in m/s, with km/h and mph underneath.
  * v_x / v_y with ±2σ. The ± figure turns vermilion when 2σ ≥ 0.15 m/s.
* **Charts** (the main body): four stacked panels on a shared time axis. Zoom with
  **20 s / 60 s / Run**.
  * (a) velocity: v_x with its ±2σ band, plus v_y, |v| and raw flow v_x as faint dots.
  * (b) acceleration: a_x, a_y.
  * (c) distance travelled and net forward.
  * (d) flow quality (PSR) and h.

  Hover for a crosshair readout of every value at that instant. While FLOW_OK is off,
  the band turns vermilion and widens and the column is tinted; the header reads
  "flow lost — dead-reckoning on IMU".
* **Runs**: Start tracking → Stop tracking makes a row with #, label, duration,
  distance, net, avg and max speed, and error vs the target. The target is editable in
  m or ft, e.g. 45 ft = 13.72 m.
  * Runs are tracked from the telemetry itself, so a slow phone reply doesn't matter.
    Runs started or stopped on the phone (the RECORDING bit) are picked up too.
  * Rows persist in the browser (localStorage). They can be deleted or exported as CSV.
  * Clicking a row freezes the charts on that run for inspection. Chart data is kept
    only for this page session; **back to live** resumes.
* **Zero** zeroes the display instantly with a client-side baseline, then sends
  `{"cmd":"zero"}`. When the phone's own reset arrives, it is absorbed seamlessly.
* **Status row**: IMU · Flow · GNSS · LiDAR · ZUPT · Rec, each a dot and a label
  (teal = ok, vermilion = fault, hollow = inactive). "gated" appears next to Flow when
  the innovation gate rejects a flow update. Calibrating or an uninitialised filter
  shows as a short note. On the right, one mono line gives link health:
  `Hz · gaps · crc · age · phone IP`.
* **Controls**: Zero · Start/Stop tracking · Mark (key **M**) · Calibrate · Measure h
  (LiDAR height + zero + IMU bias), plus the torch slider. A mono line in the status row
  shows flow quality, h, yaw rate, battery, torch, seq and frame version/format.
* **Setup** (collapsed): run label, phone IP override, Ping and command-link state.
  **Command log** (collapsed): every request with the phone's reply.
* **No signal**: the numbers and plot fade to grey with "No signal, last frame 3.1 s
  ago". Nothing blinks.
* Press **F** for fullscreen. Reloading the page refills the last 60 s from the
  server.

## Options

```
python3 -m gsdash [--udp-port 9000] [--http-port 8080] [--bind 127.0.0.1]
                  [--phone IP] [--cmd-port 9001] [--log-dir dashboard/logs]
                  [--no-log] [--no-bonjour]
```

## Logs

Each dashboard session writes to `dashboard/logs/<yyyy-mm-dd_HH-MM-SS>/`. The folder
is gitignored.

* `telemetry.csv`: every received frame. Header
  `t,v_x,v_y,sigma_vx,sigma_vy,distance,status,flow_quality,h,ax,ay,gz,flow_vx,flow_vy,net_forward,seq,recv_time,version`.
  The first 9 columns match the phone's `est.csv`. The v2 columns are empty for v1
  frames. `recv_time` is laptop Unix time.
* `commands.csv`: every command sent and the phone's reply.

## HTTP API

| Method | Path | |
|---|---|---|
| GET | `/events` | Server-Sent Events: `history` (last `?history=` seconds on connect, default 60, max 80), `frames` (array, ≤ 50 pushes/s), `stats` (5 Hz) |
| GET | `/api/status` | link stats, phone link state, last frame |
| POST | `/api/cmd` | `{"cmd":"start_run","label":"x"}` returns the phone's JSON reply, or `{"ok":false,"local":true,"error":…}` if the phone could not be reached |
| POST | `/api/phone` | `{"ip":"172.20.10.1"}` (empty means auto) |
| POST | `/api/reset_stats` | zero the link counters |

## Tests

```sh
python3 -m unittest discover dashboard/tests     # from the repo root
```

The tests cover the CRC check value, the golden frame from `docs/golden_frame.md` (both
binary and JSON), encode/decode roundtrip, rejection of bad CRC/magic/version/length,
v2 (72-byte) layout, roundtrip, JSON and version/length mismatch (plus a v2 golden
frame once one is added to `docs/golden_frame.md`), and the sequence-gap/wrap/restart
logic. An end-to-end test runs the server and the
simulator on random ports and checks frames, SSE, commands and CSV.
