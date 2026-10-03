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
cd dashboard
python3 -m gsdash                 # or: ./run.sh
open http://localhost:8080
```

To try it without a phone, run the fake phone in a second terminal:

```sh
cd dashboard
python3 -m gsdash.sim             # binary frames, 10 m run on a loop
python3 -m gsdash.sim --json      # JSON debug frames
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

* **Distance**: the hero number, with an editable target (default 10.00 m) and a thin
  progress rule. After **Stop**, it shows `run X m · error ±Y %`: teal ≤ 2 %, ink
  ≤ 5 %, vermilion otherwise.
* **v_x / v_y** with ±2σ. The ± figure turns vermilion when 2σ ≥ 0.15 m/s.
* **Plot**: the last 20 s, as a thin graphite v_x line inside a soft warm-grey ±2σ
  band, with v_y as a thinner muted line. While FLOW_OK is off, the band turns faint
  vermilion and widens, its column is lightly tinted, and the header reads "flow lost —
  dead-reckoning on IMU". This is the demo moment.
* **Status row**: IMU · Flow · GNSS · LiDAR · ZUPT · Rec, each a dot and a label
  (teal = ok, vermilion = fault, hollow = inactive). "gated" appears next to Flow when
  the innovation gate rejects a flow update. Calibrating or an uninitialised filter
  shows as a short note. On the right, one mono line gives link health:
  `Hz · gaps · crc · age · phone IP`.
* **Controls**: Start · Stop · Mark (key **M**) · Calibrate · Reset, plus the torch
  slider. A mono line on the right shows flow quality, h, battery, torch, seq and
  format.
* **Setup** (collapsed): run label, phone IP override, Ping and command-link state.
  **Command log** (collapsed): every request with the phone's reply.
* **No signal**: the numbers and plot fade to grey with "No signal, last frame 3.1 s
  ago". Nothing blinks.
* Press **F** for fullscreen. Reloading the page refills the last 20 s from the
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
  `t,v_x,v_y,sigma_vx,sigma_vy,distance,status,flow_quality,h,seq,recv_time`. The
  first 9 columns match the phone's `est.csv`. `recv_time` is laptop Unix time.
* `commands.csv`: every command sent and the phone's reply.

## HTTP API

| Method | Path | |
|---|---|---|
| GET | `/events` | Server-Sent Events: `history` (last 20 s on connect), `frames` (array, ≤ 50 pushes/s), `stats` (5 Hz) |
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
and the sequence-gap/wrap/restart logic. An end-to-end test runs the server and the
simulator on random ports and checks frames, SSE, commands and CSV.
