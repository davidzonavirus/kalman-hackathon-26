import Foundation

/// Reference 4-state filter, x = [v_x, v_y, b_x, b_y] (see "Kalman filter" in README.md).
///
/// Exact algorithm (a port in another language following these steps matches to 1e-6):
/// 1. First `predict` (or `reset(t:)`) initializes: x = 0, P = diag(p0_v, p0_v, p0_b, p0_b),
///    state.t = t, no propagation. Updates before that return `.skipped("not initialized")`.
/// 2. `predict`: Δt = t − state.t. Δt ≤ 0 -> ignored (state unchanged). Δt > 0.1 -> clamped to 0.1
///    (state.t still advances to t).
///      v_x += Δt·(a_x − b_x + r·v_y);  v_y += Δt·(a_y − b_y − r·v_x)   (both use the OLD v)
///      P = F P Fᵀ + diag(q_accel·Δt, q_accel·Δt, q_bias·Δt, q_bias·Δt), then P = (P+Pᵀ)/2.
/// 3. Scalar update with row H, value z, variance R:
///      ν = z − H x, S = H P Hᵀ + R, K = P Hᵀ / S, x += K ν,
///      P = (I − K H) P (I − K H)ᵀ + R K Kᵀ   (Joseph), then symmetrize.
/// 4. Flow: skipped if quality < psr_min or not finite. R = r_flow_base·(psr_ref/max(q, psr_min))².
///    Gate test uses the PRIOR x and P for both axes: nis_i = (z_i − v_i)²/(P_ii + R).
///    If either nis_i > gate -> whole sample rejected (`.gated(max nis)`), else x-axis then
///    y-axis sequential updates (`.accepted(max nis)`).
///    Lockout recovery: a gated flow sample increments a counter (any accepted flow resets it
///    to 0). When the counter reaches gate_reset_count (> 0), P_vv rows/cols are reset:
///    P[i][j] = 0 for i∈{0,1}, j≠i; P[i][i] = max(P[i][i], p0_v), i∈{0,1}; the counter is
///    zeroed and the sample is applied as an accepted update (nis from the prior).
///    Gating (flow and GNSS) is disarmed after init/reset until the first accepted flow, GNSS
///    or ZUPT update.
/// 5. GNSS: skipped unless speed > 1, |v̂| > 1 and speed_acc > 0. H = [v_x/|v|, v_y/|v|, 0, 0],
///    ν = speed − |v̂|, R = speed_acc². Gated if ν²/S > gate.
/// 6. ZUPT: two scalar updates z = 0 (x then y) with R = r_zupt, never gated.
public final class ReferenceKF4: GroundSpeedFilter {
    public static let name = "ReferenceKF4"

    public var config: FilterConfig
    public private(set) var state = FilterState()
    /// False until the first accepted velocity measurement after init (gate disabled until then).
    private var gateArmed = false
    /// Consecutive gated flow samples (lockout recovery).
    private var flowGatedRun = 0

    // State and covariance kept as plain stored values (no heap traffic per step).
    private var x: (Double, Double, Double, Double) = (0, 0, 0, 0)
    private var P = Mat4()

    public init(config: FilterConfig = .default) {
        self.config = config
    }

    public func reset(t: Double) {
        x = (0, 0, 0, 0)
        P = Mat4()
        P[0, 0] = config.p0V; P[1, 1] = config.p0V
        P[2, 2] = config.p0B; P[3, 3] = config.p0B
        state.t = t
        state.initialized = true
        gateArmed = false
        flowGatedRun = 0
        publish()
    }

    /// Full covariance (row-major 4×4), for diagnostics/checks.
    public var covariance: [[Double]] { (0..<4).map { i in (0..<4).map { j in P[i, j] } } }

    public func predict(t: Double, ax: Double, ay: Double, r: Double) {
        guard state.initialized else { reset(t: t); return }
        var dt = t - state.t
        guard dt > 0, dt.isFinite else { return }
        dt = min(dt, FilterConfig.maxDt)
        state.t = t

        let (vx, vy, bx, by) = x
        x.0 = vx + dt * (ax - bx + r * vy)
        x.1 = vy + dt * (ay - by - r * vx)

        // F = [[1, a, −d, 0], [−a, 1, 0, −d], [0,0,1,0], [0,0,0,1]], a = r·Δt, d = Δt.
        // M = F·P only changes rows 0 and 1; then P' = M·Fᵀ only changes columns 0 and 1.
        let a = r * dt, d = dt
        var M = P
        for j in 0..<4 {
            M[0, j] = P[0, j] + a * P[1, j] - d * P[2, j]
            M[1, j] = -a * P[0, j] + P[1, j] - d * P[3, j]
        }
        var N = M
        for i in 0..<4 {
            N[i, 0] = M[i, 0] + a * M[i, 1] - d * M[i, 2]
            N[i, 1] = -a * M[i, 0] + M[i, 1] - d * M[i, 3]
        }
        N[0, 0] += config.qAccel * dt
        N[1, 1] += config.qAccel * dt
        N[2, 2] += config.qBias * dt
        N[3, 3] += config.qBias * dt
        N.symmetrize()
        P = N
        publish()
    }

    @discardableResult
    public func updateFlow(t: Double, vx: Double, vy: Double, quality: Double) -> UpdateOutcome {
        guard state.initialized else { return .skipped(reason: "not initialized") }
        guard vx.isFinite, vy.isFinite, quality.isFinite else { return .skipped(reason: "non-finite") }
        guard quality >= config.psrMin else { return .skipped(reason: "low quality") }
        let R = config.flowVariance(quality: quality)
        let nx = sq(vx - x.0) / (P[0, 0] + R)
        let ny = sq(vy - x.1) / (P[1, 1] + R)
        let nis = max(nx, ny)
        if gateArmed && nis > config.gate {
            flowGatedRun += 1
            guard config.gateResetCount > 0, Double(flowGatedRun) >= config.gateResetCount else {
                return .gated(nis: nis)
            }
            // Lockout: our own estimate has diverged, not the camera. Re-open velocity.
            for i in 0..<2 {
                for j in 0..<4 where j != i { P[i, j] = 0; P[j, i] = 0 }
                P[i, i] = max(P[i, i], config.p0V)
            }
        }
        flowGatedRun = 0
        scalarUpdate(h: (1, 0, 0, 0), innovation: vx - x.0, R: R)
        scalarUpdate(h: (0, 1, 0, 0), innovation: vy - x.1, R: R)
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
        let s = (x.0 * x.0 + x.1 * x.1).squareRoot()
        guard speed > 1, s > 1 else { return .skipped(reason: "speed < 1 m/s") }
        let h = (x.0 / s, x.1 / s, 0.0, 0.0)
        let R = speedAccuracy * speedAccuracy
        let nu = speed - s
        let S = quad(h) + R
        let nis = nu * nu / S
        if gateArmed && nis > config.gate { return .gated(nis: nis) }
        scalarUpdate(h: h, innovation: nu, R: R)
        gateArmed = true
        publish()
        return .accepted(nis: nis)
    }

    @discardableResult
    public func updateZeroVelocity(t: Double) -> UpdateOutcome {
        guard state.initialized else { return .skipped(reason: "not initialized") }
        let R = config.rZupt
        let nx = sq(x.0) / (P[0, 0] + R)
        let ny = sq(x.1) / (P[1, 1] + R)
        scalarUpdate(h: (1, 0, 0, 0), innovation: -x.0, R: R)
        scalarUpdate(h: (0, 1, 0, 0), innovation: -x.1, R: R)
        gateArmed = true
        publish()
        return .accepted(nis: max(nx, ny))
    }

    // MARK: - Internals

    private typealias Vec4 = (Double, Double, Double, Double)

    /// H P Hᵀ for a row vector h.
    private func quad(_ h: Vec4) -> Double {
        let ph = P.mul(h)
        return h.0 * ph.0 + h.1 * ph.1 + h.2 * ph.2 + h.3 * ph.3
    }

    /// One scalar Kalman update in Joseph form.
    private func scalarUpdate(h: Vec4, innovation nu: Double, R: Double) {
        let ph = P.mul(h)                        // P Hᵀ
        let S = h.0 * ph.0 + h.1 * ph.1 + h.2 * ph.2 + h.3 * ph.3 + R
        guard S > 0, S.isFinite else { return }
        let k = [ph.0 / S, ph.1 / S, ph.2 / S, ph.3 / S]
        let hv = [h.0, h.1, h.2, h.3]
        x.0 += k[0] * nu; x.1 += k[1] * nu; x.2 += k[2] * nu; x.3 += k[3] * nu

        // A = I − K H
        var A = Mat4()
        for i in 0..<4 { for j in 0..<4 { A[i, j] = (i == j ? 1 : 0) - k[i] * hv[j] } }
        // P' = A P Aᵀ + R K Kᵀ
        let AP = A.times(P)
        var Pn = AP.timesTransposed(A)
        for i in 0..<4 { for j in 0..<4 { Pn[i, j] += R * k[i] * k[j] } }
        Pn.symmetrize()
        P = Pn
    }

    private func publish() {
        state.vx = x.0; state.vy = x.1; state.bx = x.2; state.by = x.3
        state.pDiag = [P[0, 0], P[1, 1], P[2, 2], P[3, 3]]
    }
}

@inline(__always) func sq(_ v: Double) -> Double { v * v }

/// Tiny fixed-size 4×4 matrix stored inline (16 doubles in a tuple -> no heap allocation).
struct Mat4: Sendable, Equatable {
    private var m: (Double, Double, Double, Double, Double, Double, Double, Double,
                    Double, Double, Double, Double, Double, Double, Double, Double)
        = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    static func == (a: Mat4, b: Mat4) -> Bool {
        for i in 0..<4 { for j in 0..<4 where a[i, j] != b[i, j] { return false } }
        return true
    }

    subscript(i: Int, j: Int) -> Double {
        get {
            withUnsafeBytes(of: m) { $0.load(fromByteOffset: (i * 4 + j) * 8, as: Double.self) }
        }
        set {
            withUnsafeMutableBytes(of: &m) { $0.storeBytes(of: newValue, toByteOffset: (i * 4 + j) * 8, as: Double.self) }
        }
    }

    func mul(_ v: (Double, Double, Double, Double)) -> (Double, Double, Double, Double) {
        func row(_ i: Int) -> Double { self[i, 0] * v.0 + self[i, 1] * v.1 + self[i, 2] * v.2 + self[i, 3] * v.3 }
        return (row(0), row(1), row(2), row(3))
    }

    func times(_ b: Mat4) -> Mat4 {
        var r = Mat4()
        for i in 0..<4 { for j in 0..<4 {
            var s = 0.0
            for k in 0..<4 { s += self[i, k] * b[k, j] }
            r[i, j] = s
        } }
        return r
    }

    /// self · bᵀ
    func timesTransposed(_ b: Mat4) -> Mat4 {
        var r = Mat4()
        for i in 0..<4 { for j in 0..<4 {
            var s = 0.0
            for k in 0..<4 { s += self[i, k] * b[j, k] }
            r[i, j] = s
        } }
        return r
    }

    mutating func symmetrize() {
        for i in 0..<4 { for j in (i + 1)..<4 {
            let a = 0.5 * (self[i, j] + self[j, i])
            self[i, j] = a; self[j, i] = a
        } }
    }
}
