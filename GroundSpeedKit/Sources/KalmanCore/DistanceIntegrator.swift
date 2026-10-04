/// Integrates distance over time (trapezoid rule).
///
/// Default mode `.pathLength` integrates hypot(v_x, v_y): total distance travelled (always
/// grows). `.forward` integrates the SIGNED forward speed v_x: net displacement along the
/// vehicle's own axis (back-and-forth jostling cancels).
///
/// Call `add(t:vx:vy:)` after every filter predict. Rules (mirror these in Python):
/// - first sample after init/`reset` only seeds the previous point (adds 0);
/// - Δt ≤ 0 adds nothing; Δt is clamped to `FilterConfig.maxDt` (0.1 s) like the filter;
/// - if `smoothing` τ > 0, (v_x, v_y) first pass a 1st-order low-pass
///   v̄ += α (v − v̄), α = Δt/(τ + Δt), starting from 0 (seeding would add τ·v). A wobbling phone swings
///   the lens back and forth over the floor; ∫|v| counts every swing, ∫|v̄| doesn't, and the
///   low-pass preserves the area under steady motion (it only delays it by τ);
/// - speeds below `deadband` (default 0 = off) count as 0.
public struct DistanceIntegrator: Sendable, Equatable {
    public enum Mode: String, Sendable, Codable { case forward, pathLength = "path_length" }

    public private(set) var distance: Double = 0
    public var deadband: Double
    public var mode: Mode
    public var smoothing: Double
    private var lastT: Double?
    private var lastSpeed = 0.0
    private var lpX = 0.0, lpY = 0.0

    public init(deadband: Double = 0, mode: Mode = .pathLength, smoothing: Double = 0) {
        self.deadband = deadband; self.mode = mode; self.smoothing = smoothing
    }

    /// Returns the updated distance.
    @discardableResult
    public mutating func add(t: Double, vx: Double, vy: Double) -> Double {
        var (ux, uy) = (vx, vy)
        if smoothing > 0, ux.isFinite, uy.isFinite {
            if let lt = lastT {
                let dt = min(max(t - lt, 0), FilterConfig.maxDt)
                let a = dt / (smoothing + dt)
                lpX += a * (ux - lpX); lpY += a * (uy - lpY)
            }
            (ux, uy) = (lpX, lpY)
        }
        var s = mode == .forward ? ux : (ux * ux + uy * uy).squareRoot()
        if abs(s) < deadband || !s.isFinite { s = 0 }
        if let lt = lastT {
            let dt = t - lt
            if dt > 0 {
                distance += 0.5 * (s + lastSpeed) * min(dt, FilterConfig.maxDt)
            } else { return distance }
        }
        lastT = t
        lastSpeed = s
        return distance
    }

    /// `distance` plus the part the low-pass hasn't released yet (τ·|v̄|). The low-pass
    /// delays steady motion by τ, so at 1 m/s and τ = 0.5 s the plain total trails by 0.5 m
    /// while moving; this lead term removes that lag. It is not integrated, so wobble cannot
    /// accumulate through it, and it returns to 0 when the sensor stops (total unchanged).
    /// It fades with |v̄| rather than cutting off at the deadband: a cutoff dropped the
    /// reported distance by τ·deadband in one step, which clients read as a reset.
    public var leadCompensated: Double {
        guard smoothing > 0, mode == .pathLength else { return distance }
        let v = (lpX * lpX + lpY * lpY).squareRoot()
        return v.isFinite ? distance + smoothing * v : distance
    }

    /// Convenience: integrate from a filter state.
    @discardableResult
    public mutating func add(_ s: FilterState) -> Double { add(t: s.t, vx: s.vx, vy: s.vy) }

    /// Zeroes the distance but keeps the last sample and the low-pass state, so integration
    /// continues seamlessly.
    public mutating func reset() { distance = 0 }

    /// Full reset (distance and history).
    public mutating func clear() { distance = 0; lastT = nil; lastSpeed = 0; lpX = 0; lpY = 0 }
}
