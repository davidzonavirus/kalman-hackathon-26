import Foundation

/// Variant: two independent 2-state filters [v_x, b_x] and [v_y, b_y].
///
/// The yaw-rate coupling r·v is treated as a known input using the *previous* estimate of
/// the other axis (u_x = a_x + r·v̂_y, u_y = a_y − r·v̂_x), so each axis is a scalar-velocity
/// filter with F = [[1, −Δt], [0, 1]]. Cheaper and simpler than ReferenceKF4; cross
/// covariance between axes is ignored. Same FilterConfig knobs, same gating/skip rules.
///
/// GNSS (|v| is nonlinear and couples axes) is approximated by projecting the speed onto
/// the current direction estimate: z_x = speed·v̂_x/|v̂|, z_y = speed·v̂_y/|v̂|, each with
/// R = speed_acc². Gated if either axis NIS > gate.
public final class DecoupledKF2x2: GroundSpeedFilter {
    public static let name = "DecoupledKF2x2"

    public var config: FilterConfig
    public private(set) var state = FilterState()
    /// False until the first accepted velocity measurement after init (gate disabled until then).
    private var gateArmed = false
    /// Consecutive gated flow samples (lockout recovery, same rule as ReferenceKF4).
    private var flowGatedRun = 0

    /// Per-axis state and covariance [[p00, p01], [p01, p11]].
    private struct Axis {
        var v = 0.0, b = 0.0
        var p00 = 0.0, p01 = 0.0, p11 = 0.0

        mutating func predict(u: Double, dt: Double, qa: Double, qb: Double) {
            v += dt * (u - b)
            // F P Fᵀ + Q with F = [[1, −dt], [0, 1]]
            let n00 = p00 - 2 * dt * p01 + dt * dt * p11 + qa * dt
            let n01 = p01 - dt * p11
            p00 = n00; p01 = n01; p11 += qb * dt
        }

        func nis(_ z: Double, _ R: Double) -> Double { sq(z - v) / (p00 + R) }

        /// H = [1, 0] scalar update, Joseph form.
        mutating func update(z: Double, R: Double) {
            let S = p00 + R
            guard S > 0, S.isFinite else { return }
            let k0 = p00 / S, k1 = p01 / S
            let nu = z - v
            v += k0 * nu; b += k1 * nu
            // A = [[1−k0, 0], [−k1, 1]];  P' = A P Aᵀ + R k kᵀ
            let a = 1 - k0
            let n00 = a * a * p00 + R * k0 * k0
            let n01 = a * (p01 - k1 * p00) + R * k0 * k1
            let n11 = k1 * k1 * p00 - 2 * k1 * p01 + p11 + R * k1 * k1
            p00 = n00; p01 = n01; p11 = n11
        }
    }

    private var ax = Axis(), ay = Axis()

    public init(config: FilterConfig = .default) { self.config = config }

    public func reset(t: Double) {
        ax = Axis(p00: config.p0V, p11: config.p0B)
        ay = Axis(p00: config.p0V, p11: config.p0B)
        state.t = t
        state.initialized = true
        gateArmed = false
        flowGatedRun = 0
        publish()
    }

    public func predict(t: Double, ax a_x: Double, ay a_y: Double, r: Double) {
        guard state.initialized else { reset(t: t); return }
        var dt = t - state.t
        guard dt > 0, dt.isFinite else { return }
        dt = min(dt, FilterConfig.maxDt)
        state.t = t
        let vx = ax.v, vy = ay.v
        ax.predict(u: a_x + r * vy, dt: dt, qa: config.qAccel, qb: config.qBias)
        ay.predict(u: a_y - r * vx, dt: dt, qa: config.qAccel, qb: config.qBias)
        publish()
    }

    @discardableResult
    public func updateFlow(t: Double, vx: Double, vy: Double, quality: Double) -> UpdateOutcome {
        guard state.initialized else { return .skipped(reason: "not initialized") }
        guard vx.isFinite, vy.isFinite, quality.isFinite else { return .skipped(reason: "non-finite") }
        guard quality >= config.psrMin else { return .skipped(reason: "low quality") }
        let R = config.flowVariance(quality: quality)
        let nis = max(ax.nis(vx, R), ay.nis(vy, R))
        if gateArmed && nis > config.gate {
            flowGatedRun += 1
            guard config.gateResetCount > 0, Double(flowGatedRun) >= config.gateResetCount else {
                return .gated(nis: nis)
            }
            ax.p01 = 0; ax.p00 = max(ax.p00, config.p0V)
            ay.p01 = 0; ay.p00 = max(ay.p00, config.p0V)
        }
        flowGatedRun = 0
        ax.update(z: vx, R: R)
        ay.update(z: vy, R: R)
        gateArmed = true
        publish()
        return .accepted(nis: nis)
    }

    @discardableResult
    public func updateGNSS(t: Double, speed: Double, speedAccuracy: Double) -> UpdateOutcome {
        guard state.initialized else { return .skipped(reason: "not initialized") }
        guard speed.isFinite, speedAccuracy.isFinite, speedAccuracy > 0 else {
            return .skipped(reason: "invalid accuracy")
        }
        let s = (ax.v * ax.v + ay.v * ay.v).squareRoot()
        guard speed > 1, s > 1 else { return .skipped(reason: "speed < 1 m/s") }
        let zx = speed * ax.v / s, zy = speed * ay.v / s
        let R = speedAccuracy * speedAccuracy
        let nis = max(ax.nis(zx, R), ay.nis(zy, R))
        if gateArmed && nis > config.gate { return .gated(nis: nis) }
        ax.update(z: zx, R: R)
        ay.update(z: zy, R: R)
        gateArmed = true
        publish()
        return .accepted(nis: nis)
    }

    @discardableResult
    public func updateZeroVelocity(t: Double) -> UpdateOutcome {
        guard state.initialized else { return .skipped(reason: "not initialized") }
        let nis = max(ax.nis(0, config.rZupt), ay.nis(0, config.rZupt))
        ax.update(z: 0, R: config.rZupt)
        ay.update(z: 0, R: config.rZupt)
        gateArmed = true
        publish()
        return .accepted(nis: nis)
    }

    private func publish() {
        state.vx = ax.v; state.vy = ay.v; state.bx = ax.b; state.by = ay.b
        state.pDiag = [ax.p00, ay.p00, ax.p11, ay.p11]
    }
}
