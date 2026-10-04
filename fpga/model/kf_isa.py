"""
kf_isa.py - the FPGA Kalman filter: instruction set, microcode, and a bit-exact model.

The FPGA does not hard-wire the filter. `rtl/kf_cpu.v` is a small fixed-point processor
(one 56-bit multiplier/accumulator, a divider and square root, 1024 x 56-bit register file) and the filter is a microprogram for it. This file is the single source of
truth for that program:

    python kf_isa.py --emit ../rtl      # writes rtl/kf_rom.v and rtl/kf_isa.vh

and it contains `Cpu`, a Python model of the processor that does exactly what the Verilog
does (same rounding, same saturation), so the simulated FPGA can be checked bit for bit.

Number format: signed Q16.40 in 56 bits, value = integer / 2**40 (range +-32768, step 9e-13).
Everything saturates instead of wrapping.

The program is ReferenceKF4.swift (see kf_ref.py): predict, flow / GNSS / zero-velocity
scalar updates in Joseph form, innovation gate with arming and lockout recovery. Time
handling (dt, clamping, first-sample init) stays in the host (`FixedKF4`), the same split
the Swift app has between SensorFusionEngine and the filter.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

W = 56
F = 40
ONE_Q = 1 << F
MAXV = (1 << (W - 1)) - 1
MINV = -(1 << (W - 1))
NREG = 1024
IMEM_MAX = 1024

# ---- ISA ---------------------------------------------------------------------------------
OPS = ["NOP", "HALT", "ADD", "SUB", "MUL", "DIV", "SQRT", "MOV", "AVG", "MAX",
       "CLR", "MAC", "MACN", "STA", "JMP", "BLT", "BGE", "CALL", "RET"]
OP = {n: i for i, n in enumerate(OPS)}
# word = {op[35:30], d[29:20], a[19:10], b[9:0]}; branch target lives in d.

CMDS = ["RESET", "PREDICT", "FLOW", "GNSS", "ZUPT"]
CMD = {n: i for i, n in enumerate(CMDS)}

STATUS_ACCEPTED, STATUS_GATED, STATUS_SKIPPED = 0, 1, 2


def sat(v: int) -> int:
    return MAXV if v > MAXV else MINV if v < MINV else v


def to_q(x: float) -> int:
    """float -> Q12.36, saturating (host side conversion)."""
    return sat(int(round(x * ONE_Q)))


def from_q(v: int) -> float:
    return v / ONE_Q


def _rshift_round(v: int) -> int:
    return sat((v + (1 << (F - 1))) >> F)


def q_div(a: int, b: int) -> int:
    """(a << F) / b truncated toward zero, saturated; x/0 saturates by sign (0/0 = 0)."""
    if b == 0:
        return MAXV if a > 0 else MINV if a < 0 else 0
    n = (abs(a) << F) // abs(b)
    return sat(n if (a < 0) == (b < 0) else -n)


def q_sqrt(a: int) -> int:
    return math.isqrt(max(a, 0) << F)


# ---- assembler -------------------------------------------------------------------------------
class Asm:
    def __init__(self):
        self.regs: dict[str, int] = {}
        self.code: list[list] = []        # [op, d, a, b] (names / labels until link())
        self.labels: dict[str, int] = {}

    def r(self, name: str) -> str:
        if name not in self.regs:
            self.regs[name] = len(self.regs)
        return name

    def label(self, name: str):
        assert name not in self.labels, name
        self.labels[name] = len(self.code)

    def op(self, op, d=None, a=None, b=None):
        branch = op in ("JMP", "BLT", "BGE", "CALL")
        for x in ((a, b) if branch else (d, a, b)):
            if x is not None:
                self.r(x)                    # temporaries are allocated on first use
        self.code.append([op, d, a, b])

    # instruction helpers (operands are register names)
    def add(self, d, a, b): self.op("ADD", d, a, b)
    def sub(self, d, a, b): self.op("SUB", d, a, b)
    def mul(self, d, a, b): self.op("MUL", d, a, b)
    def div(self, d, a, b): self.op("DIV", d, a, b)
    def sqrt(self, d, a): self.op("SQRT", d, a, None)
    def mov(self, d, a): self.op("MOV", d, a, None)
    def avg(self, d, a, b): self.op("AVG", d, a, b)
    def max(self, d, a, b): self.op("MAX", d, a, b)
    def clr(self): self.op("CLR")
    def mac(self, a, b): self.op("MAC", None, a, b)
    def macn(self, a, b): self.op("MACN", None, a, b)
    def sta(self, d): self.op("STA", d)
    def jmp(self, lbl): self.op("JMP", lbl)
    def blt(self, a, b, lbl): self.op("BLT", lbl, a, b)      # if a <  b goto lbl
    def bge(self, a, b, lbl): self.op("BGE", lbl, a, b)      # if a >= b goto lbl
    def call(self, lbl): self.op("CALL", lbl)
    def ret(self): self.op("RET")
    def halt(self): self.op("HALT")

    def link(self) -> list[tuple[int, int, int, int]]:
        out = []
        for op, d, a, b in self.code:
            def reg(x):
                return 0 if x is None else self.regs[x]
            if op in ("JMP", "BLT", "BGE", "CALL"):
                dd = self.labels[d]
            else:
                dd = reg(d)
            out.append((OP[op], dd, reg(a), reg(b)))
        assert len(out) <= IMEM_MAX and len(self.regs) <= NREG
        return out


# ---- the program -------------------------------------------------------------------------------
# Constants the host writes after power-up (see FixedKF4.configure).
CONST_NAMES = ["ZERO", "ONE", "TWO", "C_QA", "C_QB", "C_RFB", "C_PSRREF", "C_PSRMIN", "C_RZ",
               "C_GATE", "C_P0V", "C_P0B", "C_GRC"]
INPUT_NAMES = ["DT", "AX", "AY", "RZ", "FVX", "FVY", "FQ", "GS", "GA"]
OUTPUT_NAMES = ["X0", "X1", "X2", "X3", "STATUS"] + [f"P{i}{i}" for i in range(4)]


def build_program() -> Asm:
    a = Asm()
    R = a.r
    for n in CONST_NAMES + INPUT_NAMES + ["STATUS", "GARMED", "GRUN"]:
        R(n)
    X = [R(f"X{i}") for i in range(4)]
    P = [[R(f"P{i}{j}") for j in range(4)] for i in range(4)]
    H = [R(f"H{i}") for i in range(4)]
    for n in ("RR", "NU", "S"):
        R(n)

    # -- command vector (the host starts a command by its index) --
    for c in CMDS:
        a.jmp("L_" + c)

    # ---------------------------------------------------------------- RESET
    a.label("L_RESET")
    for i in range(4):
        a.mov(X[i], "ZERO")
        for j in range(4):
            a.mov(P[i][j], "ZERO")
    a.mov(P[0][0], "C_P0V"); a.mov(P[1][1], "C_P0V")
    a.mov(P[2][2], "C_P0B"); a.mov(P[3][3], "C_P0B")
    a.mov("GARMED", "ZERO"); a.mov("GRUN", "ZERO")
    a.mov("STATUS", "ZERO")
    a.halt()

    # ---------------------------------------------------------------- PREDICT
    # x: v_x += dt (a_x - b_x + r v_y);  v_y += dt (a_y - b_y - r v_x), both from the OLD v.
    a.label("L_PREDICT")
    a.sub("t1", "AX", X[2]); a.mul("t2", "RZ", X[1]); a.add("t1", "t1", "t2"); a.mul("t1", "DT", "t1")
    a.sub("t3", "AY", X[3]); a.mul("t4", "RZ", X[0]); a.sub("t3", "t3", "t4"); a.mul("t3", "DT", "t3")
    a.add(X[0], X[0], "t1"); a.add(X[1], X[1], "t3")
    # P = F P F^T + Q, F = I + dt [[0, r, -1, 0], [-r, 0, 0, -1], 0, 0]; A = r dt, d = dt.
    A_ = R("PA")
    a.mul(A_, "RZ", "DT")
    D_ = "DT"
    M = [[None] * 4 for _ in range(4)]
    for j in range(4):                                   # M = F P (rows 0, 1 change)
        M[0][j] = R(f"M0{j}"); M[1][j] = R(f"M1{j}")
        M[2][j] = P[2][j]; M[3][j] = P[3][j]
        a.clr(); a.mac(P[0][j], "ONE"); a.mac(A_, P[1][j]); a.macn(D_, P[2][j]); a.sta(M[0][j])
        a.clr(); a.macn(A_, P[0][j]); a.mac(P[1][j], "ONE"); a.macn(D_, P[3][j]); a.sta(M[1][j])
    N = [[None] * 4 for _ in range(4)]
    for i in range(4):                                   # N = M F^T (columns 0, 1 change)
        N[i][0] = R(f"N{i}0"); N[i][1] = R(f"N{i}1")
        N[i][2] = M[i][2]; N[i][3] = M[i][3]
        a.clr(); a.mac(M[i][0], "ONE"); a.mac(A_, M[i][1]); a.macn(D_, M[i][2]); a.sta(N[i][0])
        a.clr(); a.macn(A_, M[i][0]); a.mac(M[i][1], "ONE"); a.macn(D_, M[i][3]); a.sta(N[i][1])
    a.mul("QV", "C_QA", "DT"); a.mul("QB", "C_QB", "DT")
    a.add(N[0][0], N[0][0], "QV"); a.add(N[1][1], N[1][1], "QV")
    a.add(N[2][2], N[2][2], "QB"); a.add(N[3][3], N[3][3], "QB")
    _symmetrize_into_P(a, N, P, "PS")
    a.halt()

    # ---------------------------------------------------------------- FLOW
    a.label("L_FLOW")
    a.blt("FQ", "C_PSRMIN", "L_SKIP")                    # low quality: skipped, not down-weighted
    a.max("fm", "FQ", "C_PSRMIN"); a.div("fk", "C_PSRREF", "fm"); a.mul("fk", "fk", "fk")
    a.mul("RR", "C_RFB", "fk")                           # R = r_flow_base (psr_ref / max(q, psr_min))^2
    a.bge("ZERO", "GARMED", "L_FLOW_APPLY")              # gate not armed yet: always apply
    # Gate on the PRIOR x, P, both axes: nu_i^2 / (P_ii + R) > gate  <=>  nu_i^2 > gate (P_ii + R)
    a.sub("fex", "FVX", X[0]); a.mul("fex", "fex", "fex"); a.add("fdx", P[0][0], "RR"); a.mul("fdx", "C_GATE", "fdx")
    a.sub("fey", "FVY", X[1]); a.mul("fey", "fey", "fey"); a.add("fdy", P[1][1], "RR"); a.mul("fdy", "C_GATE", "fdy")
    a.blt("fdx", "fex", "L_FLOW_GATED")
    a.blt("fdy", "fey", "L_FLOW_GATED")
    a.jmp("L_FLOW_APPLY")
    a.label("L_FLOW_GATED")
    a.add("GRUN", "GRUN", "ONE")
    a.bge("ZERO", "C_GRC", "L_GATED")                    # lockout recovery disabled
    a.blt("GRUN", "C_GRC", "L_GATED")                    # fewer than gate_reset_count in a row
    for i in range(2):                                   # lockout: re-open velocity covariance
        for j in range(4):
            if j != i:
                a.mov(P[i][j], "ZERO"); a.mov(P[j][i], "ZERO")
    a.max(P[0][0], P[0][0], "C_P0V"); a.max(P[1][1], P[1][1], "C_P0V")
    a.label("L_FLOW_APPLY")
    a.mov("GRUN", "ZERO")
    a.sub("NU", "FVX", X[0]); a.call("UPDX0")
    a.sub("NU", "FVY", X[1]); a.call("UPDX1")           # y innovation uses the x-updated state
    a.jmp("L_ACCEPT")

    # ---------------------------------------------------------------- GNSS
    a.label("L_GNSS")
    a.bge("ZERO", "GA", "L_SKIP")                        # needs speed_acc > 0
    a.mul("g1", X[0], X[0]); a.mul("g2", X[1], X[1]); a.add("g1", "g1", "g2"); a.sqrt("gs", "g1")
    a.bge("ONE", "GS", "L_SKIP")                         # needs speed > 1
    a.bge("ONE", "gs", "L_SKIP")                         # and |v_hat| > 1
    a.div(H[0], X[0], "gs"); a.div(H[1], X[1], "gs"); a.mov(H[2], "ZERO"); a.mov(H[3], "ZERO")
    a.mul("RR", "GA", "GA"); a.sub("NU", "GS", "gs")
    a.bge("ZERO", "GARMED", "L_GNSS_APPLY")
    _quad(a, P, H)                                       # S = H P H^T + R; gated if nu^2 > gate S
    a.mul("g1", "NU", "NU"); a.mul("g3", "C_GATE", "S")
    a.blt("g3", "g1", "L_GATED")
    a.label("L_GNSS_APPLY")
    a.call("UPD")
    a.jmp("L_ACCEPT")

    # ---------------------------------------------------------------- ZUPT (never gated)
    a.label("L_ZUPT")
    a.mov("RR", "C_RZ")
    a.sub("NU", "ZERO", X[0]); a.call("UPDX0")
    a.sub("NU", "ZERO", X[1]); a.call("UPDX1")
    a.jmp("L_ACCEPT")

    # ---------------------------------------------------------------- outcomes
    a.label("L_ACCEPT"); a.mov("GARMED", "ONE"); a.mov("STATUS", "ZERO"); a.halt()
    a.label("L_GATED"); a.mov("STATUS", "ONE"); a.halt()
    a.label("L_SKIP"); a.mov("STATUS", "TWO"); a.halt()

    # ---------------------------------------------------------------- subroutines
    # UPDX0 / UPDX1: scalar update with H = e_axis, Joseph form written out for that H:
    #   K = P[:,a] / (P_aa + R);  x += K nu;  AP = P - K P[a,:];  P = AP - AP[:,a] K^T + R K K^T
    # In: RR (variance), NU (innovation). Only the upper triangle is computed, then mirrored.
    for ax in (0, 1):
        _axis_update(a, P, X, ax)

    # UPD: general scalar update, Joseph form. In: H0..3, RR (variance), NU (innovation).
    a.label("UPD")
    _quad(a, P, H)
    a.bge("ZERO", "S", "UPD_END")                        # guard S > 0
    K = [R(f"K{i}") for i in range(4)]
    for i in range(4):
        a.div(K[i], f"PH{i}", "S")
    for i in range(4):
        a.mul("ut", K[i], "NU"); a.add(X[i], X[i], "ut")
    Am = [[R(f"A{i}{j}") for j in range(4)] for i in range(4)]
    for i in range(4):                                   # A = I - K H
        for j in range(4):
            a.mul("ut", K[i], H[j]); a.sub(Am[i][j], "ONE" if i == j else "ZERO", "ut")
    AP = [[R(f"AP{i}{j}") for j in range(4)] for i in range(4)]
    for i in range(4):                                   # AP = A P
        for j in range(4):
            a.clr()
            for m in range(4):
                a.mac(Am[i][m], P[m][j])
            a.sta(AP[i][j])
    PN = [[R(f"PN{i}{j}") for j in range(4)] for i in range(4)]
    for i in range(4):                                   # PN = AP A^T + R K K^T
        for j in range(4):
            a.clr()
            for m in range(4):
                a.mac(AP[i][m], Am[j][m])
            a.mul("ut", "RR", K[i]); a.mac("ut", K[j]); a.sta(PN[i][j])
    _symmetrize_into_P(a, PN, P, "US")
    a.label("UPD_END")
    a.ret()
    return a


def _axis_update(a: Asm, P, X, ax: int):
    R = a.r
    a.label(f"UPDX{ax}")
    a.add("xs", P[ax][ax], "RR")                         # S = P_aa + R   (> 0: P_aa >= 0, R > 0)
    a.bge("ZERO", "xs", f"UPDX{ax}_END")
    K = [R(f"XK{i}") for i in range(4)]
    for i in range(4):                                   # K_i = P_ia / S (1/S would overflow Q16.40)
        a.div(K[i], P[i][ax], "xs")
    for i in range(4):
        a.mul("xt", K[i], "NU"); a.add(X[i], X[i], "xt")
        a.mul(f"XR{i}", "RR", K[i])                      # R K_i
    # AP[i][j] for i <= j, and AP[i][a] for every i
    AP = {}
    for i in range(4):
        for j in sorted({j for j in range(4) if j >= i} | {ax}):
            AP[(i, j)] = R(f"XA{i}{j}")
            a.clr(); a.mac(P[i][j], "ONE"); a.macn(K[i], P[ax][j]); a.sta(AP[(i, j)])
    PN = {}
    for i in range(4):                                   # P[i][j] = AP[i][j] - AP[i][a] K_j + R K_i K_j
        for j in range(i, 4):
            PN[(i, j)] = R(f"XP{i}{j}")
            a.clr(); a.mac(AP[(i, j)], "ONE"); a.macn(AP[(i, ax)], K[j]); a.mac(f"XR{i}", K[j]); a.sta(PN[(i, j)])
    for i in range(4):
        for j in range(i, 4):
            a.mov(P[i][j], PN[(i, j)])
            if j != i:
                a.mov(P[j][i], PN[(i, j)])
    a.label(f"UPDX{ax}_END")
    a.ret()


def _quad(a: Asm, P, H):
    for i in range(4):
        a.clr()
        for j in range(4):
            a.mac(P[i][j], H[j])
        a.sta(f"PH{i}")
    a.clr()
    for i in range(4):
        a.mac(H[i], f"PH{i}")
    a.mac("RR", "ONE"); a.sta("S")


def _set_h(a: Asm, H, axis: int):
    for i in range(4):
        a.mov(H[i], "ONE" if i == axis else "ZERO")


def _symmetrize_into_P(a: Asm, Nm, P, tag: str):
    """P = (N + N^T) / 2, computed into temporaries first (N may alias P)."""
    tmp = {}
    for i in range(4):
        for j in range(i, 4):
            tmp[(i, j)] = a.r(f"{tag}{i}{j}")
            if i == j:
                a.mov(tmp[(i, j)], Nm[i][i])
            else:
                a.avg(tmp[(i, j)], Nm[i][j], Nm[j][i])
    for i in range(4):
        for j in range(4):
            a.mov(P[i][j], tmp[(min(i, j), max(i, j))])


_PROG = None


def program():
    """(asm, linked words) - built once."""
    global _PROG
    if _PROG is None:
        asm = build_program()
        _PROG = (asm, asm.link())
    return _PROG


def reg(name: str) -> int:
    return program()[0].regs[name]


# ---- bit-exact processor model ---------------------------------------------------------------
class Cpu:
    """What rtl/kf_cpu.v does, one instruction at a time. `rf` holds signed ints."""

    def __init__(self, words=None):
        self.rf = [0] * NREG
        self.acc = 0
        self.lr = 0
        self.instr_count = 0
        self.words = words if words is not None else program()[1]
        self.trace = None                    # set to a list to record executed pcs

    def write(self, addr: int, val: int):
        self.rf[addr] = val

    def read(self, addr: int) -> int:
        return self.rf[addr]

    def run(self, cmd: int) -> int:
        """Execute a command; returns the number of instructions executed."""
        pc, n, rf = cmd, 0, self.rf
        words = self.words
        while True:
            if self.trace is not None:
                self.trace.append(pc)
            op, d, a, b = words[pc]
            n += 1
            name = OPS[op]
            ra, rb = rf[a], rf[b]
            pc += 1
            if name == "HALT":
                break
            elif name == "ADD": rf[d] = sat(ra + rb)
            elif name == "SUB": rf[d] = sat(ra - rb)
            elif name == "MUL": rf[d] = _rshift_round(ra * rb)
            elif name == "DIV": rf[d] = q_div(ra, rb)
            elif name == "SQRT": rf[d] = q_sqrt(ra)
            elif name == "MOV": rf[d] = ra
            elif name == "AVG": rf[d] = (ra + rb) >> 1
            elif name == "MAX": rf[d] = ra if ra > rb else rb
            elif name == "CLR": self.acc = 0
            elif name == "MAC": self.acc += ra * rb
            elif name == "MACN": self.acc -= ra * rb
            elif name == "STA": rf[d] = _rshift_round(self.acc)
            elif name == "JMP": pc = d
            elif name == "BLT":
                if ra < rb: pc = d
            elif name == "BGE":
                if ra >= rb: pc = d
            elif name == "CALL": self.lr, pc = pc, d
            elif name == "RET": pc = self.lr
        self.instr_count += n
        return n


# ---- host side: the filter as the app sees it --------------------------------------------------
from kf_ref import (ACCEPTED, GATED, SKIPPED, MAX_DT, FilterConfig, FilterState)  # noqa: E402


class SimBackend:
    """Backend = (write, read, run) over the Python processor model."""
    name = "python-model"

    def __init__(self, words=None):
        self.cpu = Cpu(words)

    def write(self, addr, val): self.cpu.write(addr, val)
    def read(self, addr): return self.cpu.read(addr)
    def read_many(self, addrs): return [self.cpu.read(a) for a in addrs]
    def run(self, cmd): return self.cpu.run(cmd)
    def close(self): pass


class FixedKF4:
    """ReferenceKF4's interface on top of any backend (the Python model or the simulated
    FPGA). Same host/coprocessor split as the app: the host does time (dt, clamping, init),
    the coprocessor does the filter."""
    name = "FpgaKF4"

    def __init__(self, config: FilterConfig | None = None, backend=None):
        self.config = config or FilterConfig()
        self.backend = backend or SimBackend()
        self.state = FilterState()
        self.cycles = 0
        self._configure()

    def _configure(self):
        c, b = self.config, self.backend
        vals = {"ZERO": 0, "ONE": 1, "TWO": 2, "C_QA": c.q_accel, "C_QB": c.q_bias,
                "C_RFB": c.r_flow_base, "C_PSRREF": c.psr_ref, "C_PSRMIN": c.psr_min,
                "C_RZ": c.r_zupt, "C_GATE": c.gate, "C_P0V": c.p0_v, "C_P0B": c.p0_b,
                "C_GRC": c.gate_reset_count}
        for k, v in vals.items():
            b.write(reg(k), to_q(v))

    def _put(self, **inputs):
        for k, v in inputs.items():
            self.backend.write(reg(k), to_q(v))

    def _publish(self):
        b, s = self.backend, self.state
        addrs = [reg(f"X{i}") for i in range(4)] + [reg(f"P{i}{i}") for i in range(4)]
        v = b.read_many(addrs) if hasattr(b, "read_many") else [b.read(a) for a in addrs]
        s.vx, s.vy, s.bx, s.by = (from_q(q) for q in v[:4])
        s.p_diag = [from_q(q) for q in v[4:]]

    def _outcome(self):
        st = self.backend.read(reg("STATUS")) >> F
        return ((ACCEPTED, GATED, SKIPPED)[st], 0.0)     # the FPGA does not report NIS

    def _run(self, cmd):
        self.cycles += self.backend.run(CMD[cmd]) or 0

    def reset(self, t):
        self._run("RESET")
        self.state.t, self.state.initialized = t, True
        self._publish()

    def seed(self, t, x, p_diag):
        """Start from a logged state (diagonal P), like ReferenceKF4.seed."""
        self.reset(t)
        for i in range(4):
            self.backend.write(reg(f"X{i}"), to_q(x[i]))
            self.backend.write(reg(f"P{i}{i}"), to_q(p_diag[i]))
        self._publish()

    def predict(self, t, ax, ay, r):
        if not self.state.initialized:
            return self.reset(t)
        dt = t - self.state.t
        if not (dt > 0 and math.isfinite(dt)):
            return
        dt = min(dt, MAX_DT)
        self.state.t = t
        self._put(DT=dt, AX=ax, AY=ay, RZ=r)
        self._run("PREDICT")
        self._publish()

    def _update(self, cmd, **inputs):
        if not self.state.initialized:
            return (SKIPPED, 0.0)
        if not all(math.isfinite(v) for v in inputs.values()):
            return (SKIPPED, 0.0)
        self._put(**inputs)
        self._run(cmd)
        self._publish()
        return self._outcome()

    def update_flow(self, t, vx, vy, quality):
        return self._update("FLOW", FVX=vx, FVY=vy, FQ=quality)

    def update_gnss(self, t, speed, speed_accuracy):
        return self._update("GNSS", GS=speed, GA=speed_accuracy)

    def update_zero_velocity(self, t):
        return self._update("ZUPT")


# ---- Verilog generation --------------------------------------------------------------------------
def rom_verilog(words) -> str:
    lines = ["// kf_rom.v - GENERATED by model/kf_isa.py --emit. Do not edit by hand.",
             f"// {len(words)} instructions: {{op[35:30], d[29:20], a[19:10], b[9:0]}}.",
             "module kf_rom (input wire [9:0] addr, output reg [35:0] data);",
             "    always @* begin",
             "        case (addr)"]
    for pc, (op, d, a, b) in enumerate(words):
        lines.append(f"            10'd{pc}: data = 36'h{(op << 30 | d << 20 | a << 10 | b):09x};"
                     f"  // {OPS[op]}")
    lines += ["            default: data = 36'h0;", "        endcase", "    end", "endmodule", ""]
    return "\n".join(lines)


def emit_rtl(out_dir: Path):
    asm, words = program()
    (out_dir / "kf_rom.v").write_text(rom_verilog(words))
    hdr = ["// kf_isa.vh - GENERATED by model/kf_isa.py --emit. Do not edit by hand."]
    hdr += [f"localparam [5:0] OP_{n} = 6'd{i};" for n, i in OP.items()]
    hdr.append(f"localparam KF_W = {W};")
    hdr.append(f"localparam KF_F = {F};")
    (out_dir / "kf_isa.vh").write_text("\n".join(hdr) + "\n")
    regs = ["// kf_regs.vh - GENERATED by model/kf_isa.py --emit. Register addresses for testbenches."]
    regs += [f"localparam R_{n} = {i};" for n, i in asm.regs.items()]
    (out_dir / "kf_regs.vh").write_text("\n".join(regs) + "\n")
    print(f"wrote {out_dir / 'kf_rom.v'} ({len(words)} instructions, {len(asm.regs)} registers)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--emit", metavar="DIR", help="write kf_rom.v and kf_isa.vh into DIR")
    ap.add_argument("--map", action="store_true", help="print the register map")
    a = ap.parse_args()
    if a.emit:
        emit_rtl(Path(a.emit))
    if a.map:
        for n, i in program()[0].regs.items():
            print(f"{i:4d} {n}")
    if not (a.emit or a.map):
        asm, words = program()
        print(f"{len(words)} instructions, {len(asm.regs)} registers")


if __name__ == "__main__":
    main()
