import Foundation

/// Snapshot of a filter's estimate.
public struct FilterState: Sendable, Equatable {
    /// Time of the last predict (s). Updates are applied at this time and do not move it,
    /// so the next predict always integrates the full IMU interval.
    public var t: Double
    public var vx, vy: Double            // m/s, vehicle frame
    public var bx, by: Double            // accel biases, m/s²
    public var pDiag: [Double]           // [P_vx, P_vy, P_bx, P_by]
    public var sigmaVx: Double { pDiag[0].squareRoot() }
    public var sigmaVy: Double { pDiag[1].squareRoot() }
    public var initialized: Bool

    public init(t: Double = 0, vx: Double = 0, vy: Double = 0, bx: Double = 0, by: Double = 0,
                pDiag: [Double] = [0, 0, 0, 0], initialized: Bool = false) {
        self.t = t; self.vx = vx; self.vy = vy; self.bx = bx; self.by = by
        self.pDiag = pDiag; self.initialized = initialized
    }

    public var speed: Double { (vx * vx + vy * vy).squareRoot() }
}

public enum UpdateOutcome: Sendable, Equatable {
    case accepted(nis: Double)           // nis = ν²/S
    case gated(nis: Double)              // rejected by innovation gate
    case skipped(reason: String)         // e.g. GNSS speed < 1 m/s, not initialized

    public var isAccepted: Bool { if case .accepted = self { return true }; return false }
    public var isGated: Bool { if case .gated = self { return true }; return false }
}

public protocol GroundSpeedFilter: AnyObject {
    static var name: String { get }      // goes into meta.json "filter_name"
    var state: FilterState { get }
    var config: FilterConfig { get set } // Q, R, gate, etc. (Codable -> meta.json)

    func reset(t: Double)

    /// 100 Hz, on every IMU sample. Vehicle frame, gravity removed.
    /// ax, ay in m/s²; r = yaw rate in rad/s. dt derived from t − state.t.
    func predict(t: Double, ax: Double, ay: Double, r: Double)

    /// Optical flow velocity (vehicle frame, m/s). quality = PSR.
    /// Applied as two scalar updates (z = v_x, z = v_y) with R scaled by quality.
    @discardableResult
    func updateFlow(t: Double, vx: Double, vy: Double, quality: Double) -> UpdateOutcome

    /// GNSS speed |v| with reported accuracy (1σ m/s). Skip below 1 m/s.
    @discardableResult
    func updateGNSS(t: Double, speed: Double, speedAccuracy: Double) -> UpdateOutcome

    /// Zero-velocity pseudo-measurement z = 0 on both velocities.
    @discardableResult
    func updateZeroVelocity(t: Double) -> UpdateOutcome
}

public extension GroundSpeedFilter {
    /// Instance-side access to the static name (handy with `any GroundSpeedFilter`).
    var filterName: String { Self.name }
}

/// Tunables shared by all filter variants. Saved in meta.json and editable in the app.
///
/// Units / meaning (continuous-time densities, discretised as Q = q·Δt):
/// - `q_accel`: accel white-noise PSD driving velocity, (m/s²)²·s. Q_v = q_accel·Δt.
/// - `q_bias`: bias random-walk PSD, (m/s²)²/s. Q_b = q_bias·Δt.
/// - `r_flow_base`: flow velocity variance (m/s)² at PSR = `psr_ref`.
///   R_flow = r_flow_base · (psr_ref / max(quality, psr_min))².
/// - `psr_min`: flow samples with quality < psr_min are skipped (see ReferenceKF4).
///   PSR scale of `PhaseCorrelator`: noise/unrelated frames < ~7, textured floor 20–600, so
///   defaults are psr_min = 8, psr_ref = 20 (σ_flow = 4 cm/s at PSR 20, 1.6 cm/s at PSR 50).
/// - `r_zupt`: ZUPT pseudo-measurement variance (m/s)².
/// - `gate`: NIS threshold (ν²/S); 9 ≈ 3σ. The gate is only armed after the first accepted
///   velocity measurement (flow, GNSS or ZUPT) since init/reset, so a filter started while
///   already moving locks on instead of rejecting every sample.
/// - `p0_v`, `p0_b`: initial variances of velocity (m/s)² and bias (m/s²)².
/// - `gate_reset_count`: lockout recovery. After this many CONSECUTIVE gated flow samples
///   (good PSR, ≈ 0.1 s at 120 Hz) the filter concludes its own estimate has diverged
///   (e.g. after a jolt), raises the velocity variances to at least `p0_v` (velocity
///   cross-covariances zeroed) and accepts the sample. 0 disables recovery.
public struct FilterConfig: Codable, Sendable, Equatable {
    public var qAccel: Double
    public var qBias: Double
    public var rFlowBase: Double
    public var psrRef: Double
    public var psrMin: Double
    public var rZupt: Double
    public var gate: Double
    public var p0V: Double
    public var p0B: Double
    public var gateResetCount: Double

    public init(qAccel: Double = 0.1, qBias: Double = 1e-5, rFlowBase: Double = 1.6e-3,
                psrRef: Double = 20, psrMin: Double = 8, rZupt: Double = 1e-4,
                gate: Double = 9, p0V: Double = 0.25, p0B: Double = 0.01,
                gateResetCount: Double = 12) {
        self.qAccel = qAccel; self.qBias = qBias; self.rFlowBase = rFlowBase
        self.psrRef = psrRef; self.psrMin = psrMin; self.rZupt = rZupt
        self.gate = gate; self.p0V = p0V; self.p0B = p0B
        self.gateResetCount = gateResetCount
    }

    public static let `default` = FilterConfig()

    enum CodingKeys: String, CodingKey {
        case qAccel = "q_accel", qBias = "q_bias", rFlowBase = "r_flow_base"
        case psrRef = "psr_ref", psrMin = "psr_min", rZupt = "r_zupt"
        case gate, p0V = "p0_v", p0B = "p0_b", gateResetCount = "gate_reset_count"
    }

    /// Missing keys fall back to defaults, so old meta.json / partial cfg files still load.
    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        let d = FilterConfig()
        qAccel = try c.decodeIfPresent(Double.self, forKey: .qAccel) ?? d.qAccel
        qBias = try c.decodeIfPresent(Double.self, forKey: .qBias) ?? d.qBias
        rFlowBase = try c.decodeIfPresent(Double.self, forKey: .rFlowBase) ?? d.rFlowBase
        psrRef = try c.decodeIfPresent(Double.self, forKey: .psrRef) ?? d.psrRef
        psrMin = try c.decodeIfPresent(Double.self, forKey: .psrMin) ?? d.psrMin
        rZupt = try c.decodeIfPresent(Double.self, forKey: .rZupt) ?? d.rZupt
        gate = try c.decodeIfPresent(Double.self, forKey: .gate) ?? d.gate
        p0V = try c.decodeIfPresent(Double.self, forKey: .p0V) ?? d.p0V
        p0B = try c.decodeIfPresent(Double.self, forKey: .p0B) ?? d.p0B
        gateResetCount = try c.decodeIfPresent(Double.self, forKey: .gateResetCount) ?? d.gateResetCount
    }

    /// `meta.json` "q" object.
    public var qDict: [String: Double] { ["q_accel": qAccel, "q_bias": qBias] }
    /// `meta.json` "r" object (measurement-side knobs incl. gate and P0).
    public var rDict: [String: Double] {
        ["r_flow_base": rFlowBase, "psr_ref": psrRef, "psr_min": psrMin, "r_zupt": rZupt,
         "gate": gate, "p0_v": p0V, "p0_b": p0B, "gate_reset_count": gateResetCount]
    }

    /// Flow measurement variance for a given PSR.
    @inlinable public func flowVariance(quality: Double) -> Double {
        let k = psrRef / max(quality, psrMin)
        return rFlowBase * k * k
    }

    /// Largest Δt a single predict will integrate; longer gaps are clamped.
    public static let maxDt = 0.1
}
