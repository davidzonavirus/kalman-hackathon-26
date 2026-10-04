"""
kf_rtl.py - the simulated FPGA as a backend for kf_isa.FixedKF4.

Runs the Verilog in ../rtl (kf_cpu + the generated microcode ROM) in a logic simulator and
talks to it over a pipe through sim/tb_stream.v. The host (Python) writes inputs and
constants into the processor's registers, starts a command and reads the state back, exactly
what a UART/SPI bridge would do on a board.

Simulators, in order of preference (override with backend=... or KF_SIM=...):
    modelsim  vsim from the Quartus/Intel FPGA install, simulating rtl/*.v
    iverilog  Icarus Verilog (open source), simulating rtl/*.v
    gate      vsim simulating the netlist Quartus synthesised and fitted for the Cyclone V
              (quartus/simulation/questa/kf_cpu.vo, made by quartus/gatesim.ps1), so what is
              simulated is the compiled design, not the source

Pure standard library.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
FPGA = HERE.parent
RTL = FPGA / "rtl"
BUILD = FPGA / "sim" / "build"
RTL_FILES = ["kf_cpu.v", "kf_rom.v"]

_IVERILOG_DIRS = [r"C:\iverilog\bin", "/usr/local/bin", "/opt/homebrew/bin"]
_MODELSIM_DIRS = [r"C:\intelFPGA\20.1\modelsim_ase\win32aloem", r"C:\intelFPGA_lite\modelsim_ase\win32aloem",
                  r"C:\altera_lite\25.1std\questa_fse\win64", r"C:\altera_lite\25.1std\questa_fe\win64"]
_EXTRA = {"iverilog": _IVERILOG_DIRS, "vvp": _IVERILOG_DIRS, "vsim": _MODELSIM_DIRS,
          "vlog": _MODELSIM_DIRS, "vlib": _MODELSIM_DIRS}


def which(tool: str) -> str | None:
    p = shutil.which(tool)
    if p:
        return p
    for d in _EXTRA.get(tool, []):
        for ext in ("", ".exe"):
            c = Path(d) / (tool + ext)
            if c.exists():
                return str(c)
    return None


def quartus_root() -> Path | None:
    """The Quartus install (the folder that has eda/sim_lib), from PATH or the usual places."""
    import glob
    q = shutil.which("quartus_sh")
    cands = [Path(q).resolve().parents[1]] if q else []
    for pat in (r"C:\altera_lite\*\quartus", r"C:\intelFPGA_lite\*\quartus", "/opt/intelFPGA*/*/quartus",
                str(Path.home() / "intelFPGA*" / "*" / "quartus")):
        cands += [Path(p) for p in sorted(glob.glob(pat))]
    for c in cands:
        if (c / "eda" / "sim_lib").exists():
            return c
    return None


GATE_NETLIST = FPGA / "quartus" / "simulation" / "questa" / "kf_cpu.vo"


def available() -> list[str]:
    out = []
    if which("vsim"):
        out.append("modelsim")
    if which("iverilog") and which("vvp"):
        out.append("iverilog")
    if which("vsim") and GATE_NETLIST.exists() and quartus_root():
        out.append("gate")
    return out


class RtlBackend:
    """(write, read, run) over a simulator process. `run` returns the clock count."""

    def __init__(self, simulator: str | None = None, rom_words=None, workdir=None):
        """rom_words: run a different microprogram (testing); built in `workdir`."""
        sim = simulator or os.environ.get("KF_SIM") or (available() or [None])[0]
        if sim is None:
            raise RuntimeError("no Verilog simulator found: install Icarus Verilog (iverilog) or "
                               "Quartus with ModelSim, or use the Python model (SimBackend)")
        self.simulator = sim
        self.name = f"rtl-{sim}"
        self.build = Path(workdir) if workdir else BUILD
        self.build.mkdir(parents=True, exist_ok=True)
        self.rtl_dir = RTL
        if rom_words is not None:
            import kf_isa
            self.rtl_dir = self.build / "rtl"
            self.rtl_dir.mkdir(exist_ok=True)
            for f in RTL_FILES + ["kf_isa.vh"]:
                shutil.copy(RTL / f, self.rtl_dir / f)
            (self.rtl_dir / "kf_rom.v").write_text(kf_isa.rom_verilog(rom_words))
        if sim == "iverilog":
            cmd = self._build_iverilog()
        elif sim in ("modelsim", "gate"):
            cmd = self._build_modelsim(gate=(sim == "gate"))
        else:
            raise ValueError(f"unknown simulator {sim!r}")
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, bufsize=1, cwd=str(self.build))
        self._out = []
        self.clocks = 0

    # -- building -------------------------------------------------------------------------
    def _stale(self, target: Path, sources) -> bool:
        return (not target.exists()) or any(s.stat().st_mtime > target.stat().st_mtime for s in sources)

    def _build_iverilog(self):
        iv, vvp = which("iverilog"), which("vvp")
        out = self.build / "tb_stream.vvp"
        srcs = [FPGA / "sim" / "tb_stream.v"] + [self.rtl_dir / f for f in RTL_FILES] + [self.rtl_dir / "kf_isa.vh"]
        if self._stale(out, srcs):
            subprocess.run([iv, "-g2005", "-I", str(self.rtl_dir), "-o", str(out)] + [str(s) for s in srcs[:-1]],
                           check=True)
        return [vvp, "-n", str(out)]

    def _build_modelsim(self, gate: bool = False):
        vsim = which("vsim")
        vlog = which("vlog")
        vlib = which("vlib")
        if gate:
            lib = quartus_root() / "eda" / "sim_lib"
            srcs = [FPGA / "sim" / "tb_stream.v", GATE_NETLIST, lib / "altera_primitives.v",
                    lib / "cyclonev_atoms.v",
                    lib / "mentor" / "cyclonev_atoms_ncrypt.v", lib / "altera_lnsim.sv"]
            work, defs = self.build / "work_gate", ["+define+GATE_LEVEL"]
            incs = []
        else:
            srcs = [FPGA / "sim" / "tb_stream.v"] + [self.rtl_dir / f for f in RTL_FILES] + [self.rtl_dir / "kf_isa.vh"]
            work, defs = self.build / "work", []
            incs = [f"+incdir+{self.rtl_dir}"]
            srcs = srcs[:-1]
        if self._stale(work / "_info", srcs):
            shutil.rmtree(work, ignore_errors=True)
            subprocess.run([vlib, work.name], check=True, cwd=str(self.build), stdout=subprocess.DEVNULL)
            subprocess.run([vlog, "-work", work.name] + defs + incs + [str(s) for s in srcs],
                           check=True, cwd=str(self.build), stdout=subprocess.DEVNULL)
        return [vsim, "-c", "-quiet", work.name + ".tb_stream", "-do", "run -all; quit -f"]

    # -- protocol -------------------------------------------------------------------------
    def _send(self, line: str):
        self.proc.stdin.write(line + "\n")

    def _recv(self, tag: str) -> str:
        while True:
            line = self.proc.stdout.readline()
            if not line:
                raise RuntimeError("simulator exited unexpectedly")
            line = line.strip()
            if line.startswith("# "):               # ModelSim prefixes $display output
                line = line[2:]
            if line.startswith(tag + " "):
                return line[2:]

    def write(self, addr: int, val: int):
        self._send(f"w {addr:x} {val & ((1 << 56) - 1):x}")

    def read(self, addr: int) -> int:
        return self.read_many([addr])[0]

    def read_many(self, addrs) -> list[int]:
        for a in addrs:
            self._send(f"r {a:x} 0")
        self.proc.stdin.flush()
        out = []
        for _ in addrs:
            v = int(self._recv("R"), 16)
            out.append(v - (1 << 56) if v >> 55 else v)
        return out

    def run(self, cmd: int) -> int:
        self._send(f"c {cmd:x} 0")
        self.proc.stdin.flush()
        n = int(self._recv("D"))
        self.clocks += n
        return n

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=5)
        except Exception:
            self.proc.kill()

    def __enter__(self): return self
    def __exit__(self, *a): self.close()


if __name__ == "__main__":
    print("simulators found:", available() or "none")


# ---- batch mode ---------------------------------------------------------------------------------
class _Recorder:
    """Records the host's writes / reads / runs; reads return 0 (control flow never depends on them)."""
    def __init__(self):
        self.lines = []

    def write(self, addr, val): self.lines.append(f"w {addr:x} {val & ((1 << 56) - 1):x}")
    def read(self, addr): self.lines.append(f"r {addr:x} 0"); return 0
    def read_many(self, addrs): return [self.read(a) for a in addrs]
    def run(self, cmd): self.lines.append(f"c {cmd:x} 0"); return 0
    def close(self): pass


class _Playback:
    """Feeds the simulator's answers back to the same host code, in order."""
    name = "rtl-batch"

    def __init__(self, results):
        self.results, self.i, self.clocks = results, 0, 0

    def _next(self, kind):
        k, v = self.results[self.i]
        assert k == kind, "host code is not deterministic"
        self.i += 1
        return v

    def write(self, addr, val): pass
    def read(self, addr): return self._next("R")
    def read_many(self, addrs): return [self._next("R") for _ in addrs]
    def run(self, cmd):
        n = self._next("D"); self.clocks += n; return n
    def close(self): pass


def run_batch(host_fn, simulator: str | None = None, timeout: float = 3600.0):
    """Run `host_fn(backend)` against the simulated FPGA in ONE simulator launch.

    ModelSim cannot be driven interactively through a pipe, so offline work (replays) uses
    this: pass 1 records every register access, the simulator runs the whole script from a
    file, pass 2 replays `host_fn` with the real answers. Needs `host_fn` to issue the same
    accesses whatever the answers are (true of kf_ref.replay: inputs come from the log).
    Returns whatever `host_fn` returns on the second pass, with the backend attached as
    `.backend` on the first return value if it is a FixedKF4.
    """
    sim = simulator or os.environ.get("KF_SIM") or (available() or [None])[0]
    if sim is None:
        raise RuntimeError("no Verilog simulator found")
    rec = _Recorder()
    host_fn(rec)
    be = RtlBackend.__new__(RtlBackend)               # reuse the build logic without starting a process
    be.simulator, be.build, be.rtl_dir = sim, BUILD, RTL
    BUILD.mkdir(parents=True, exist_ok=True)
    cmd = be._build_iverilog() if sim == "iverilog" else be._build_modelsim(gate=(sim == "gate"))
    script = BUILD / "batch_in.txt"
    script.write_text("\n".join(rec.lines) + "\n")
    with open(script) as fin:
        out = subprocess.run(cmd, stdin=fin, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, cwd=str(BUILD), timeout=timeout).stdout
    results = []
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("# "):
            line = line[2:]
        if line.startswith("R "):
            v = int(line[2:], 16)
            results.append(("R", v - (1 << 56) if v >> 55 else v))
        elif line.startswith("D "):
            results.append(("D", int(line[2:])))
    n_expected = sum(1 for l in rec.lines if l[0] in "rc")
    if len(results) != n_expected:
        raise RuntimeError(f"simulator returned {len(results)} answers, expected {n_expected}:\n{out[-800:]}")
    return host_fn(_Playback(results))
