import Foundation

/// Deterministic synthetic sensor stream for a straight 10.00 m push, used by `phone-sim`
/// (real-time playback) and by gsk-checks (fast-forward).
///
/// Profile (vehicle frame, x forward): rest -> accelerate at 0.5 m/s² to 1 m/s -> cruise ->
/// decelerate to rest -> rest. Exactly `distance` metres are travelled.
/// Sensors: IMU 100 Hz (noise + bias, extra vibration while moving), flow 120 Hz (noise,
/// good PSR, except a lens-covered dropout where PSR ≈ 2), GNSS 1 Hz, LiDAR depth 15 Hz.
public struct SimPush: Sendable {
    public enum Kind: Sendable { case imu, flow, gnss, depth }

    public struct Sample: Sendable {
        public var kind: Kind
        /// Seconds from the start of the synthetic run.
        public var t: Double
        public var v: (Double, Double, Double, Double, Double, Double)
    }

    public struct Config: Sendable {
        public var restBefore = 3.0
        public var restAfter = 3.0
        public var accel = 0.5
        public var cruiseSpeed = 1.0
        public var distance = 10.0
        public var imuHz = 100.0
        public var flowHz = 120.0
        public var gnssHz = 1.0
        public var depthHz = 15.0
        public var accelNoise = 0.03        // m/s² at rest
        public var vibrationNoise = 0.12    // m/s² extra while moving
        public var gyroNoise = 0.002        // rad/s
        public var accelBias = (x: 0.03, y: -0.02)
        public var gyroBias = 0.001
        public var flowNoise = 0.01         // m/s
        public var flowPSR = 25.0
        /// Lens-covered window (s from start), nil = none.
        public var dropout: ClosedRange<Double>? = 8.0...10.0
        public var height = 0.30
        public var seed: UInt64 = 42
        public init() {}
    }

    public let config: Config
    public let samples: [Sample]

    /// Total duration (s).
    public var duration: Double {
        let c = config
        let tRamp = c.cruiseSpeed / c.accel
        let dRamp = 0.5 * c.accel * tRamp * tRamp
        let tCruise = (c.distance - 2 * dRamp) / c.cruiseSpeed
        return c.restBefore + 2 * tRamp + tCruise + c.restAfter
    }

    public init(config: Config = Config()) {
        self.config = config
        self.samples = SimPush.generate(config)
    }

    /// True kinematics at time τ: (position, velocity, acceleration).
    public static func truth(_ c: Config, _ tau: Double) -> (x: Double, v: Double, a: Double) {
        let tRamp = c.cruiseSpeed / c.accel
        let dRamp = 0.5 * c.accel * tRamp * tRamp
        let tCruise = (c.distance - 2 * dRamp) / c.cruiseSpeed
        let t1 = c.restBefore, t2 = t1 + tRamp, t3 = t2 + tCruise, t4 = t3 + tRamp
        if tau < t1 { return (0, 0, 0) }
        if tau < t2 { let s = tau - t1; return (0.5 * c.accel * s * s, c.accel * s, c.accel) }
        if tau < t3 { let s = tau - t2; return (dRamp + c.cruiseSpeed * s, c.cruiseSpeed, 0) }
        if tau < t4 {
            let s = tau - t3
            return (dRamp + c.cruiseSpeed * tCruise + c.cruiseSpeed * s - 0.5 * c.accel * s * s,
                    c.cruiseSpeed - c.accel * s, -c.accel)
        }
        return (c.distance, 0, 0)
    }

    private static func generate(_ c: Config) -> [Sample] {
        var rng = SimRNG(seed: c.seed)
        var out: [Sample] = []
        let tRamp = c.cruiseSpeed / c.accel
        let dRamp = 0.5 * c.accel * tRamp * tRamp
        let total = c.restBefore + 2 * tRamp + (c.distance - 2 * dRamp) / c.cruiseSpeed + c.restAfter

        func add(_ kind: Kind, hz: Double, _ make: (Double) -> (Double, Double, Double, Double, Double, Double)) {
            let n = Int((total * hz).rounded(.down))
            for i in 0...n {
                let t = Double(i) / hz
                out.append(Sample(kind: kind, t: t, v: make(t)))
            }
        }

        add(.imu, hz: c.imuHz) { t in
            let tr = truth(c, t)
            let moving = tr.v > 0.02
            let sa = c.accelNoise + (moving ? c.vibrationNoise : 0)
            return (tr.a + c.accelBias.x + sa * rng.gauss(),
                    c.accelBias.y + sa * rng.gauss(),
                    sa * rng.gauss(),
                    c.gyroNoise * rng.gauss(),
                    c.gyroNoise * rng.gauss(),
                    c.gyroBias + c.gyroNoise * rng.gauss())
        }
        add(.flow, hz: c.flowHz) { t in
            let tr = truth(c, t)
            if let d = c.dropout, d.contains(t) {
                // Lens covered: correlator returns junk with a low PSR.
                return (0.05 * rng.gauss(), 0.05 * rng.gauss(), 2.0 + 0.3 * abs(rng.gauss()), c.height, 0, 0)
            }
            return (tr.v + c.flowNoise * rng.gauss(), c.flowNoise * rng.gauss(),
                    max(10, c.flowPSR + 4 * rng.gauss()), c.height, 0, 0)
        }
        add(.gnss, hz: c.gnssHz) { t in
            let tr = truth(c, t)
            return (max(0, tr.v + 0.1 * rng.gauss()), 0.3, 90, 0, 0, 0)
        }
        add(.depth, hz: c.depthHz) { _ in
            (c.height + 0.002 * rng.gauss(), 0, 0, 0, 0, 0)
        }
        out.sort { $0.t < $1.t }
        return out
    }

    /// Feeds sample `s` (time offset by `t0`) into the engine.
    public static func feed(_ s: Sample, t0: Double, into engine: SensorFusionEngine) {
        let t = t0 + s.t
        let v = s.v
        switch s.kind {
        case .imu: engine.ingestIMU(t: t, ax: v.0, ay: v.1, az: v.2, gx: v.3, gy: v.4, gz: v.5)
        case .flow: engine.ingestFlow(t: t, vx: v.0, vy: v.1, quality: v.2, h: v.3)
        case .gnss: engine.ingestGNSS(t: t, speed: v.0, speedAcc: v.1, course: v.2)
        case .depth: engine.ingestDepth(t: t, h: v.0)
        }
    }
}

/// Small deterministic RNG (SimRNG) with a Box–Muller Gaussian.
public struct SimRNG: RandomNumberGenerator, Sendable {
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

    /// Uniform in (0, 1).
    public mutating func uniform() -> Double {
        (Double(next() >> 11) + 0.5) / Double(1 << 53)
    }

    public mutating func gauss() -> Double {
        if let s = spare { spare = nil; return s }
        let u1 = uniform(), u2 = uniform()
        let r = (-2 * log(u1)).squareRoot()
        spare = r * sin(2 * .pi * u2)
        return r * cos(2 * .pi * u2)
    }
}
