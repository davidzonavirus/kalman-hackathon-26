# The Kalman filter on an FPGA

The 4-state filter from `GroundSpeedKit/Sources/KalmanCore/ReferenceKF4.swift` (state
`[v_x, v_y, b_x, b_y]`, IMU predict, optical-flow / GNSS / zero-velocity updates, Joseph-form
covariance, 3-sigma gate with arming and lockout recovery) as hardware, built with Quartus and
simulated at clock level. The dashboard can run its estimate on it instead of on the phone
(`dashboard/run.sh --fpga`, see below).

```
 host (Python)                          FPGA design (fpga/rtl)
 ───────────────                        ──────────────────────────────────────────
 dt, clamping, init        write regs   kf_cpu: 56-bit fixed-point coprocessor
 ZUPT decision        ───────────────▶  ┌────────────────────────────────────────┐
 distance integrator       start cmd    │ kf_rom: the filter, as microcode       │
 sensor / frame I/O   ◀───────────────  │ 1024 x 56 register file, MAC, divider, │
                          read regs     │ square root, branch                    │
                                        └────────────────────────────────────────┘
```

The filter is not a block of hard-wired arithmetic. `kf_cpu.v` is a small processor and the
filter is a program for it (`model/kf_isa.py` writes both the program and a bit-exact Python
model of the processor). Changing the filter is editing Python and running
`python model/kf_isa.py --emit rtl`. The same split the Swift app has between
`SensorFusionEngine` (time, ZUPT, distance) and the filter (`ReferenceKF4`) is kept: the host
does time handling, the FPGA does the filter.

## Files

| Path | What |
|---|---|
| `rtl/kf_cpu.v` | The coprocessor: register file, multiplier/accumulator, divider, square root, sequencer |
| `rtl/kf_rom.v`, `rtl/kf_isa.vh` | **Generated** by `model/kf_isa.py --emit rtl`: the microprogram and opcode constants |
| `model/kf_ref.py` | Double-precision Python port of `ReferenceKF4`, `ZuptDetector`, `DistanceIntegrator`, `Replay` (golden model) |
| `model/kf_isa.py` | Instruction set, the filter microprogram, `Cpu` (bit-exact model), `FixedKF4` (host driver) |
| `model/kf_rtl.py` | Runs the Verilog in ModelSim (Quartus) or Icarus Verilog as a backend for `FixedKF4` |
| `model/replay.py` | Replays a recorded run through the simulated FPGA and compares with float and `est.csv` |
| `model/check_model.py`, `model/check_rtl.py` | The two verification steps below |
| `sim/tb_stream.v` | The testbench that is the "board": talks to Python over stdin/stdout |
| `quartus/` | Quartus Prime project (`kf_cpu.qpf/qsf/sdc`), `build.ps1` / `build.sh` |

## Number format

Signed Q16.40 in 56 bits: `integer = round(value * 2^40)`, range ±32768, step 9e-13. Every
operation saturates instead of wrapping. The width is not arbitrary: 40 fractional bits because
the filter adds `q_bias * dt = 1e-7` to a covariance of ~1e-2 every tick, and 16 integer bits
because the flow quality (PSR) reaches 3000+ on a textured floor (a first version with Q12.36
clipped it and drifted by 5e-5 m/s; Q8.40 clipped it and drifted by 1e-2 m/s).

## The processor

| | |
|---|---|
| Registers | 1024 x 56 bit, two read ports (two RAM copies written together) |
| Arithmetic | `ADD SUB MUL MAX AVG MOV`, multiply-accumulate `CLR MAC MACN STA` (120-bit accumulator, one rounding per dot product), `DIV` (2 quotient bits/clock), `SQRT` |
| Control | `JMP BLT BGE CALL RET HALT` |
| Timing | 3 clocks per instruction (2 for branches / `CLR`); `DIV`/`SQRT` +48 |
| Program | 857 instructions |

A command is started by index: `0 RESET`, `1 PREDICT`, `2 FLOW`, `3 GNSS`, `4 ZUPT`. The host
writes the inputs (`DT AX AY RZ` / `FVX FVY FQ` / `GS GA`) and the constants (`C_QA ... C_GRC`,
all of `FilterConfig`) into registers, pulses `start`, waits for `done`, and reads `X0..X3`,
`P00..P33` and `STATUS` (0 accepted, 1 gated, 2 skipped). Register addresses:
`python model/kf_isa.py --map`.

Cost of one command at 50 MHz: predict 360 clocks (7 µs), flow update 1250 clocks (25 µs),
ZUPT 1180 clocks. The app needs one predict per 10 ms.

Differences from the Swift code, none of which change the result beyond rounding:

* flow / ZUPT use a Joseph form written out for `H = e_x, e_y` (about half the instructions),
  and only the upper triangle of `P` is computed, then mirrored, instead of averaging
* the gate is evaluated as `nu^2 > gate * S` (no division), so the NIS value is not reported
* time handling (`dt`, clamp to 0.1 s, "first sample initialises") is in the host

## Verification

The chain, each link checked against the previous:

```
phone's est.csv  ==  kf_ref.ReferenceKF4 (float port)  ~=  kf_isa.FixedKF4 (Q16.40 model)  ==  Verilog (simulated)
      replay.py              replay.py / check_model.py                    check_rtl.py
```

1. **Float port vs the phone.** The runs in `data/phone_runs` store the raw sensors and the
   phone's own `est.csv`. For runs made with the final filter (from `2026-10-03_22-18-56`) the
   float port reproduces `est.csv` to 1e-7 m/s, and the distance to 3 decimals. (Earlier runs
   were made with older builds of the Swift filter and do not match; the port is of the final one.)
2. **Fixed-point model vs float.** `python model/check_model.py` replays all 53 recorded runs
   (up to 3000 IMU ticks each, 4000 flow updates, ZUPTs) through both: worst difference
   **2.6e-9 m/s** in velocity, 6e-10 in sigma, 5e-9 m in distance.
3. **Verilog vs the model, bit for bit.** `python model/check_rtl.py` runs random stress
   sequences (saturating inputs, x/0, GNSS gating, lockout recovery) and slices of real runs
   through `kf_cpu.v` in ModelSim and requires every outcome, every published value and the whole
   register file to be identical.

```sh
python fpga/model/check_model.py
python fpga/model/check_rtl.py                          # ModelSim from Quartus if found, else Icarus
python fpga/model/replay.py data/phone_runs/2026-10-03_22-22-18 --ticks 600
```

## Quartus

```powershell
fpga\quartus\build.ps1          # or fpga/quartus/build.sh
```

Project `quartus/kf_cpu.qpf`, Cyclone V `5CEBA4F23C7` (change `DEVICE` in the `.qsf` for your
board), 50 MHz clock. The host port is wider than a board connector, so it is routed as virtual
pins (area/timing study); a UART or SPI bridge in front of it is the missing board piece.

Result of the Quartus Prime 25.1 Lite compile (synthesis, fit, timing analysis all pass):

| | |
|---|---|
| Logic | 1,733 ALMs of 18,480 (9 %) |
| Memory | 12 M10K blocks, 114,688 bits (4 %): the two register-file copies |
| DSP blocks | 10 of 66 (15 %): the 56 x 56 multiplier |
| Timing at 50 MHz | met: worst setup slack +2.4 ns (slow corner, 0 C), hold slack +0.12 ns |

A predict is 360 clocks and a flow update 1250 clocks, i.e. 7 us and 25 us at 50 MHz.


## Using it from the dashboard

```sh
dashboard/run.sh --fpga                       # live: needs Icarus Verilog (iverilog) on PATH
dashboard/run.sh --fpga --fpga-backend model  # same program on the Python model, no simulator
```

Each 72-byte v2 frame carries the raw sensors (a_x, a_y, gyro z, flow v_x / v_y / PSR). The
dashboard writes them into the simulated FPGA, runs predict (every frame), flow update (when the
frame has a new flow sample) and ZUPT (while the phone's ZUPT bit is set), and shows the FPGA's
`v_x, v_y, sigma, distance` in place of the phone's (the phone's are kept in `frame["phone"]`).
`/api/status` has an `fpga` block (backend, frames, load, simulated clocks) and the header shows
the load. If the simulation falls behind, frames queued behind the newest are only predicted.
At about 2000 simulated clocks per frame, Icarus Verilog runs it at ~60 frames/s on a laptop;
a 50 Hz stream uses roughly 80% of one core.

ModelSim cannot be driven interactively through a pipe, so live mode uses Icarus Verilog.
Everything offline (`replay.py`, `check_rtl.py`) uses ModelSim when Quartus is installed.

## Not done

* The ZUPT *detector* stays on the phone (the dashboard uses the phone's ZUPT bit); only the
  zero-velocity *update* runs on the FPGA. GNSS updates are implemented and verified but the
  dashboard frame has no GNSS speed, so live mode does not use them (the replay tool does).
* No board top level: no UART/SPI bridge or pin assignments, so it has not been run on hardware.
* `ReferenceKF4`'s tuned config is loaded by the host (`FilterConfig`); there is no run-time
  config command beyond writing the constant registers.
