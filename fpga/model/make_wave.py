"""
make_wave.py - open the simulated FPGA in the ModelSim waveform viewer.

Takes a stretch of a recorded run, turns it into the register writes / commands the host would
send (the same ones kf_rtl.py uses), and opens ModelSim (the simulator that comes with Quartus)
with the inputs and outputs of the filter as analog traces:

    python make_wave.py                                   # default: a push in 2026-10-03_22-22-18
    python make_wave.py --run <run_dir> --start 250 --ticks 80
    python make_wave.py --batch                           # run without the GUI (checks it works)
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import kf_isa  # noqa: E402
import kf_ref  # noqa: E402
import kf_rtl  # noqa: E402

FPGA = Path(__file__).resolve().parents[1]
OUT = FPGA / "sim" / "build" / "wave"

WAVE_DO = """quietly set sig /tb_wave
add wave -divider {host command}
add wave -color Gold -radix ascii $sig/cmd_name
add wave $sig/clk
add wave $sig/busy
add wave $sig/done
add wave -divider {inputs (written by the host)}
add wave -format Analog-Step -height 60 -min @AX@ -color Cyan $sig/in_ax
add wave -format Analog-Step -height 60 -min @AY@ -color Cyan $sig/in_ay
add wave -format Analog-Step -height 60 -min @GZ@ -color Cyan $sig/in_gyro_z
add wave -format Analog-Step -height 60 -min @FVX@ -color Orange $sig/in_flow_vx
add wave -format Analog-Step -height 40 -min @PSR@ -color Orange $sig/in_flow_psr
add wave -divider {outputs (read back from the filter)}
add wave -format Analog-Step -height 80 -min @VX@ -color Green $sig/out_vx
add wave -format Analog-Step -height 50 -min @VY@ -color Green $sig/out_vy
add wave -format Analog-Step -height 50 -min @SIG@ -color Magenta $sig/out_sigma_vx
add wave -format Analog-Step -height 50 -min @BX@ -color Yellow $sig/out_bias_x
add wave -format Analog-Step -height 30 -min 0 -max 2 -color Red $sig/out_status
add wave -divider {Python reference (kf_ref.py) on top of the FPGA outputs}
add wave -format Analog-Step -height 80 -min @VX@ -color White $sig/ref_vx
add wave -format Analog-Step -height 50 -min @SIG@ -color White $sig/ref_sigma_vx
add wave -divider {error = FPGA minus Python}
add wave -format Analog-Step -height 60 -min -1e-8 -max 1e-8 -color Red $sig/err_vx
add wave -format Analog-Step -height 60 -min -1e-8 -max 1e-8 -color Red $sig/err_vy
add wave -format Analog-Step -height 60 -min 0 -max 1e-7 -color Red $sig/err_pct_vx
add wave -divider {processor internals}
add wave -radix unsigned $sig/pc
add wave -radix unsigned $sig/opcode
add wave $sig/dut/state
"""

COLS = ("n", "cmd", "ax", "ay", "gz", "fvx", "psr", "vx", "vy", "bx", "by", "sigx", "sigy")


def scaled_wave_do(csv_lines):
    """WAVE_DO with every analog trace's -min/-max taken from the data (10 % padding)."""
    rows = [dict(zip(COLS, l.split(","))) for l in csv_lines]

    def rng(key, floor0=False):
        v = [float(r[key]) for r in rows]
        lo, hi = (0.0 if floor0 else min(v)), max(v)
        pad = (hi - lo) * 0.1 or 1e-3
        return f"{lo - (0 if floor0 else pad):.6g} -max {hi + pad:.6g}"

    out = WAVE_DO
    for tok, key, f0 in (("AX", "ax", 0), ("AY", "ay", 0), ("GZ", "gz", 0), ("FVX", "fvx", 0),
                         ("PSR", "psr", 1), ("VX", "vx", 0), ("VY", "vy", 0), ("SIG", "sigx", 1),
                         ("BX", "bx", 0)):
        out = out.replace(f"@{tok}@", rng(key, bool(f0)))
    return out


class Dual:
    """Drives the FPGA host driver (recording its commands) and the float Python filter with the
    same calls, and writes the Python filter's state after every FPGA command."""
    CMD = ("RESET", "PREDICT", "FLOW", "GNSS", "ZUPT")

    def __init__(self, fixed, ref, rec):
        self.fixed, self.ref, self.rec = fixed, ref, rec
        self.ref_lines, self.csv_lines, self.seen = [], [], 0
        self.inp = {"ax": 0.0, "ay": 0.0, "gz": 0.0, "fvx": 0.0, "psr": 0.0}

    @property
    def state(self):
        return self.ref.state

    def _log(self):
        cmds = [l for l in self.rec.lines if l[0] == "c"]
        s = self.ref.state
        for line in cmds[self.seen:]:
            vals = (s.vx, s.vy, s.bx, s.by, s.sigma_vx, s.sigma_vy)
            self.ref_lines.append(" ".join(f"{v:.12e}" for v in vals))
            i = self.inp
            ins = (i["ax"], i["ay"], i["gz"], i["fvx"], i["psr"])
            self.csv_lines.append(f"{len(self.csv_lines)},{self.CMD[int(line.split()[1], 16)]},"
                                  + ",".join(f"{v:.12e}" for v in ins + vals))
        self.seen = len(cmds)

    def seed(self, *a):
        self.fixed.seed(*a); self.ref.seed(*a); self._log()

    def predict(self, t, ax, ay, gz):
        self.inp.update(ax=ax, ay=ay, gz=gz)
        a = (t, ax, ay, gz)
        self.fixed.predict(*a); self.ref.predict(*a); self._log()

    def update_flow(self, t, vx, vy, q):
        self.inp.update(fvx=vx, psr=q)
        a = (t, vx, vy, q)
        self.fixed.update_flow(*a); self.ref.update_flow(*a); self._log()

    def update_gnss(self, *a):
        self.fixed.update_gnss(*a); self.ref.update_gnss(*a); self._log()

    def update_zero_velocity(self, *a):
        self.fixed.update_zero_velocity(*a); self.ref.update_zero_velocity(*a); self._log()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default=str(FPGA.parent / "data" / "phone_runs" / "2026-10-03_22-22-18"))
    ap.add_argument("--start", type=int, default=250, help="first IMU tick of the stretch")
    ap.add_argument("--ticks", type=int, default=80)
    ap.add_argument("--batch", action="store_true", help="no GUI: run to the end and quit")
    a = ap.parse_args(argv)

    r = kf_ref.load_run(a.run)
    cfg = kf_ref.FilterConfig.from_meta(r["meta"])
    i0, i1 = a.start, min(a.start + a.ticks, len(r["imu"]) - 1)
    t0, t1 = r["imu"][i0]["t"], r["imu"][i1]["t"]

    def sl(rows):
        return [s for s in rows if t0 <= s["t"] <= t1]

    est0 = r["est"][i0]
    seed = (t0, [est0["v_x"], est0["v_y"], 0.0, 0.0],
            [est0["sigma_vx"] ** 2, est0["sigma_vy"] ** 2, cfg.p0_b, cfg.p0_b])

    rec = kf_rtl._Recorder()
    f = Dual(kf_isa.FixedKF4(cfg, backend=rec), kf_ref.ReferenceKF4(cfg), rec)
    kf_ref.replay(f, sl(r["imu"]), sl(r["flow"]), sl(r["gnss"]), sl(r["events"]), seed=seed)

    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "wave_script.txt").write_text("\n".join(l for l in rec.lines if l[0] != "r") + "\n")
    (OUT / "ref_script.txt").write_text("\n".join(f.ref_lines) + "\n")
    (OUT / "ref_commands.csv").write_text(
        "n,command,in_ax,in_ay,in_gyro_z,in_flow_vx,in_flow_psr,v_x,v_y,bias_x,bias_y,sigma_vx,sigma_vy\n" + "\n".join(f.csv_lines) + "\n")
    tail = "wave zoom full\n" if a.batch else "run -all\nwave zoom full\n"
    (OUT / "wave.do").write_text(scaled_wave_do(f.csv_lines) + ("run -all\nquit -f\n" if a.batch else tail))
    print(f"{Path(a.run).name} ticks {i0}..{i1}: {len(rec.lines)} host accesses -> {OUT}")

    vsim, vlog, vlib = kf_rtl.which("vsim"), kf_rtl.which("vlog"), kf_rtl.which("vlib")
    if not vsim:
        sys.exit("ModelSim (vsim) not found")
    rtl = FPGA / "rtl"
    subprocess.run([vlib, "work"], cwd=OUT, check=True, stdout=subprocess.DEVNULL)
    subprocess.run([vlog, "-work", "work", f"+incdir+{rtl}", str(FPGA / "sim" / "tb_wave.v"),
                    str(rtl / "kf_cpu.v"), str(rtl / "kf_rom.v")], cwd=OUT, check=True)
    cmd = [vsim, "work.tb_wave", "+script=wave_script.txt", "+ref=ref_script.txt", "-do", "wave.do"]
    if a.batch:
        out = subprocess.run(cmd + ["-c"], cwd=OUT, text=True, capture_output=True).stdout
        print(out[-600:])
    else:
        subprocess.Popen(cmd, cwd=OUT)
        print("ModelSim is opening: the waveform window shows the inputs and outputs.")


if __name__ == "__main__":
    main()
