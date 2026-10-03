/// Stationarity detector for zero-velocity updates.
///
/// Fires when, over the last `window` seconds of IMU samples, the variance of the
/// (gravity-removed) accel magnitude is below `accelVarThreshold` AND the latest flow
/// speed is below `flowSpeedThreshold`. If no fresh flow is available, `requireFlow`
/// decides (default false → accel alone may declare rest).
///
/// Uses a fixed-capacity ring buffer with running sums, no allocation per sample.
public struct ZuptDetector: Sendable {
    public struct Config: Codable, Sendable, Equatable {
        public var window: Double            // s
        public var accelVarThreshold: Double // (m/s²)²
        public var flowSpeedThreshold: Double // m/s
        public var flowMaxAge: Double        // s; older flow counts as "no flow"
        public var requireFlow: Bool

        public init(window: Double = 0.3, accelVarThreshold: Double = 0.0025,
                    flowSpeedThreshold: Double = 0.03, flowMaxAge: Double = 0.2,
                    requireFlow: Bool = false) {
            self.window = window; self.accelVarThreshold = accelVarThreshold
            self.flowSpeedThreshold = flowSpeedThreshold; self.flowMaxAge = flowMaxAge
            self.requireFlow = requireFlow
        }

        enum CodingKeys: String, CodingKey {
            case window, accelVarThreshold = "accel_var_threshold"
            case flowSpeedThreshold = "flow_speed_threshold", flowMaxAge = "flow_max_age"
            case requireFlow = "require_flow"
        }
    }

    public var config: Config
    private static let capacity = 512   // > 0.3 s at up to ~1.5 kHz
    private var ts = [Double](repeating: 0, count: capacity)
    private var ms = [Double](repeating: 0, count: capacity)
    private var head = 0, count = 0
    private var sum = 0.0, sumSq = 0.0
    private var flowT = -Double.infinity
    private var flowSpeed = Double.infinity
    public private(set) var isStationary = false

    public init(config: Config = Config()) { self.config = config }

    public mutating func reset() {
        head = 0; count = 0; sum = 0; sumSq = 0
        flowT = -.infinity; flowSpeed = .infinity; isStationary = false
    }

    /// Feed the latest flow speed hypot(vx, vy) (any quality ≥ the app's threshold).
    public mutating func addFlow(t: Double, speed: Double) {
        flowT = t; flowSpeed = speed
    }

    /// Feed one IMU sample (gravity removed); returns whether the cart is stationary now.
    @discardableResult
    public mutating func addIMU(t: Double, ax: Double, ay: Double, az: Double) -> Bool {
        let m = (ax * ax + ay * ay + az * az).squareRoot()
        // Drop samples older than the window (and make room if full).
        while count > 0 {
            let tail = (head - count + Self.capacity) % Self.capacity
            if t - ts[tail] > config.window || count == Self.capacity {
                sum -= ms[tail]; sumSq -= ms[tail] * ms[tail]; count -= 1
            } else { break }
        }
        ts[head] = t; ms[head] = m
        head = (head + 1) % Self.capacity
        count += 1
        sum += m; sumSq += m * m

        let oldest = ts[(head - count + Self.capacity) % Self.capacity]
        let spanOK = (t - oldest) >= 0.8 * config.window && count >= 5
        let n = Double(count)
        let mean = sum / n
        let variance = max(0, sumSq / n - mean * mean)
        let accelStill = spanOK && variance < config.accelVarThreshold

        let flowFresh = (t - flowT) <= config.flowMaxAge
        let flowStill = flowFresh ? flowSpeed < config.flowSpeedThreshold : !config.requireFlow
        isStationary = accelStill && flowStill
        return isStationary
    }

    /// One-call form: IMU sample plus optional fresh flow speed.
    public mutating func update(t: Double, ax: Double, ay: Double, az: Double, flowSpeed: Double?) -> Bool {
        if let fs = flowSpeed { addFlow(t: t, speed: fs) }
        return addIMU(t: t, ax: ax, ay: ay, az: az)
    }
}
