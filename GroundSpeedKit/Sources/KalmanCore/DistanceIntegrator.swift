/// Integrates distance over time (trapezoid rule).
///
/// Default mode `.forward` integrates the SIGNED forward speed v_x: an odometer along the
/// vehicle's own axis. Jostling the sensor back and forth cancels out instead of
/// accumulating, and because the vehicle frame turns with the cart it still measures the
/// length of a curved path. `.pathLength` integrates hypot(v_x, v_y) (always grows).
///
/// Call `add(t:vx:vy:)` after every filter predict. Rules (mirror these in Python):
/// - first sample after init/`reset` only seeds the previous point (adds 0);
/// - Δt ≤ 0 adds nothing; Δt is clamped to `FilterConfig.maxDt` (0.1 s) like the filter;
/// - speeds below `deadband` (default 0 = off) count as 0.
public struct DistanceIntegrator: Sendable, Equatable {
    public enum Mode: String, Sendable, Codable { case forward, pathLength = "path_length" }

    public private(set) var distance: Double = 0
    public var deadband: Double
    public var mode: Mode
    private var lastT: Double?
    private var lastSpeed = 0.0

    public init(deadband: Double = 0, mode: Mode = .forward) {
        self.deadband = deadband; self.mode = mode
    }

    /// Returns the updated distance.
    @discardableResult
    public mutating func add(t: Double, vx: Double, vy: Double) -> Double {
        var s = mode == .forward ? vx : (vx * vx + vy * vy).squareRoot()
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

    /// Convenience: integrate from a filter state.
    @discardableResult
    public mutating func add(_ s: FilterState) -> Double { add(t: s.t, vx: s.vx, vy: s.vy) }

    /// Zeroes the distance but keeps the last sample, so integration continues seamlessly.
    public mutating func reset() { distance = 0 }

    /// Full reset (distance and history).
    public mutating func clear() { distance = 0; lastT = nil; lastSpeed = 0 }
}
