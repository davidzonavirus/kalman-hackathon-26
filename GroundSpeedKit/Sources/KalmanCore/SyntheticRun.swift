import Foundation

/// Deterministic PRNG (SplitMix64) + Gaussian via Box–Muller, so synthetic runs are
/// bit-reproducible across machines.
public struct SeededRandom: Sendable {
    private var state: UInt64
    private var spare: Double?
    public init(seed: UInt64) { state = seed }

    public mutating func next() -> UInt64 {
        state &+= 0x9E37_79B9_7F4A_7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        return z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    public mutating func uniform() -> Double { Double(next() >> 11) * 0x1.0p-53 }

    public mutating func uniform(_ lo: Double, _ hi: Double) -> Double { lo + (hi - lo) * uniform() }

    /// Standard normal.
    public mutating func gaussian() -> Double {
        if let s = spare { spare = nil; return s }
        var u1 = uniform()
        while u1 <= 1e-300 { u1 = uniform() }
        let u2 = uniform()
        let r = (-2 * log(u1)).squareRoot()
        spare = r * sin(2 * .pi * u2)
        return r * cos(2 * .pi * u2)
    }

    public mutating func gaussian(_ sigma: Double) -> Double { sigma * gaussian() }
}

/// Synthetic rest-to-rest straight push, used by the checks and `kfreplay --synth`.
///
/// Profile (t relative to `t0`): rest `restBefore` s -> raised-cosine accel ramp (`ramp` s) ->
/// cruise at `cruiseSpeed` -> raised-cosine decel ramp -> rest `restAfter` s. Cruise length is
/// chosen so the total distance is exactly `distance` (each ramp covers cruiseSpeed·ramp/2).
///
/// Sensors (vehicle frame, as the phone logs them):
/// - IMU 100 Hz ± 0.3 ms jitter: a = true accel + constant bias + white noise (σ 0.02 m/s² at
///   rest, 0.15 m/s² moving = cart vibration), plus small vertical bumps on az; gyro noise σ
///   0.01 rad/s + 0.002 rad/s z-bias (true yaw rate 0).
/// - Flow 120 Hz (independent clock, ± 0.4 ms jitter): PSR 20–60 moving, 60–120 at rest;
///   v + white noise with σ = 0.9·√R_flow(PSR) under the default config moving (≈ 1.3–4 cm/s),
///   3 mm/s at rest; h = 0.30 m ± 2 mm. A `dropout` (lens covered) window produces samples
///   with PSR 1.5–6 (< psr_min) and near-zero bogus velocity — the filter must skip them.
///   Two isolated outliers (5 m/s, PSR 25–35) are injected during cruise to exercise the gate.
/// - No GNSS (indoors). Depth (LiDAR) 15 Hz ≈ h.
/// - Events: start, mark lens_covered / lens_uncovered, stop.
public struct SyntheticRun: Sendable {
    public struct Config: Sendable {
        public var t0 = 1000.0
        public var distance = 10.0
        public var cruiseSpeed = 1.0
        public var ramp = 2.0
        public var restBefore = 2.0
        public var restAfter = 2.0
        public var imuRate = 100.0
        public var flowRate = 120.0
        public var accelBias = (x: 0.05, y: -0.03)
        public var gyroBiasZ = 0.002
        public var height = 0.30
        /// Dropout window, seconds after motion start (nil = none). Default: 2 s mid-cruise.
        public var dropout: (start: Double, duration: Double)? = (start: 5.0, duration: 2.0)
        public var injectOutliers = true
        public var seed: UInt64 = 42
        public init() {}
    }

    public let config: Config
    public var imu: [ImuSample] = []
    public var flow: [FlowSample] = []
    public var gnss: [GnssSample] = []
    public var depth: [(t: Double, h: Double)] = []
    public var events: [RunEvent] = []
    /// Ground truth at each IMU timestamp (t, v_x, distance).
    public var truth: [(t: Double, v: Double, d: Double)] = []

    public var startT: Double { config.t0 }
    public var endT: Double { config.t0 + totalDuration }
    public var motionStart: Double { config.t0 + config.restBefore }
    public var dropoutWindow: ClosedRange<Double>? {
        guard let d = config.dropout else { return nil }
        let s = motionStart + d.start
        return s...(s + d.duration)
    }

    private var cruiseDuration: Double {
        max(0, (config.distance - config.cruiseSpeed * config.ramp) / config.cruiseSpeed)
    }
    public var totalDuration: Double {
        config.restBefore + 2 * config.ramp + cruiseDuration + config.restAfter
    }

    /// True (v, a, distance) at time t.
    public func truthAt(_ t: Double) -> (v: Double, a: Double, d: Double) {
        let c = config, V = c.cruiseSpeed, T = c.ramp
        var tau = t - motionStart
        if tau <= 0 { return (0, 0, 0) }
        // accel ramp: v = V/2 (1 − cos(π τ/T))
        if tau < T {
            let w = Double.pi / T
            return (V / 2 * (1 - cos(w * tau)), V / 2 * w * sin(w * tau), V / 2 * (tau - sin(w * tau) / w))
        }
        let dRamp = V * T / 2
        tau -= T
        if tau < cruiseDuration { return (V, 0, dRamp + V * tau) }
        tau -= cruiseDuration
        let dCruise = dRamp + V * cruiseDuration
        if tau < T {
            let w = Double.pi / T
            return (V / 2 * (1 + cos(w * tau)), -V / 2 * w * sin(w * tau), dCruise + V / 2 * (tau + sin(w * tau) / w))
        }
        return (0, 0, dCruise + dRamp)
    }

    public init(config: Config = Config()) {
        self.config = config
        var rng = SeededRandom(seed: config.seed)
        let t0 = config.t0, tEnd = t0 + totalDuration

        // IMU
        let nImu = Int((totalDuration * config.imuRate).rounded(.down))
        for k in 0...nImu {
            let t = t0 + Double(k) / config.imuRate + rng.uniform(-0.0003, 0.0003)
            let tr = truthAt(t)
            let moving = tr.v > 0.02
            let sa = moving ? 0.15 : 0.02
            imu.append(ImuSample(
                t: t,
                ax: tr.a + config.accelBias.x + rng.gaussian(sa),
                ay: config.accelBias.y + rng.gaussian(sa),
                az: rng.gaussian(moving ? 0.25 : 0.02),
                gx: rng.gaussian(0.01), gy: rng.gaussian(0.01),
                gz: config.gyroBiasZ + rng.gaussian(0.01)))
            truth.append((t, tr.v, tr.d))
        }

        // Flow (independent clock, starts 3.7 ms after IMU)
        let drop = dropoutWindow
        var outlierTimes: [Double] = []
        if config.injectOutliers {
            let cruiseMid = motionStart + config.ramp + cruiseDuration * 0.25
            outlierTimes = [cruiseMid, cruiseMid + 0.9]
        }
        var t = t0 + 0.0037
        while t < tEnd {
            let tr = truthAt(t)
            let h = config.height + rng.gaussian(0.002)
            if let drop, drop.contains(t) {
                // Lens covered: dark, featureless frame -> noise-level PSR, bogus ~0 shift.
                flow.append(FlowSample(t: t, vx: rng.gaussian(0.05), vy: rng.gaussian(0.05),
                                       quality: rng.uniform(1.5, 6.0), h: h))
            } else if let o = outlierTimes.firstIndex(where: { abs($0 - t) < 0.5 / config.flowRate }) {
                outlierTimes.remove(at: o)
                flow.append(FlowSample(t: t, vx: 5.0, vy: -1.0, quality: rng.uniform(25, 35), h: h))
            } else {
                // PSR drops when moving (motion blur); noise consistent with the default R(PSR).
                let moving = tr.v > 0.02
                let psr = moving ? rng.uniform(20, 60) : rng.uniform(60, 120)
                let sv = moving ? 0.9 * FilterConfig.default.flowVariance(quality: psr).squareRoot() : 0.003
                flow.append(FlowSample(t: t, vx: tr.v + rng.gaussian(sv), vy: rng.gaussian(sv),
                                       quality: psr, h: h))
            }
            t += 1 / config.flowRate + rng.uniform(-0.0004, 0.0004)
        }

        // Depth 15 Hz
        var td = t0 + 0.011
        while td < tEnd { depth.append((td, config.height + rng.gaussian(0.003))); td += 1.0 / 15 }

        events.append(RunEvent(t: t0, event: "start", value: "synthetic_10m"))
        if let drop {
            events.append(RunEvent(t: drop.lowerBound, event: "mark", value: "lens_covered"))
            events.append(RunEvent(t: drop.upperBound, event: "mark", value: "lens_uncovered"))
        }
        events.append(RunEvent(t: tEnd, event: "stop", value: ""))
    }
}
