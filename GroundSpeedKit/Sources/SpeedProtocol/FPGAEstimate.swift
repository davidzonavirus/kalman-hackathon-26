import Foundation

/// The Kalman filter's estimate computed on the (simulated) FPGA by a dashboard running with
/// `--fpga`, sent back to the phone (PROTOCOL.md §4).
///
/// One JSON object per UDP datagram, from the dashboard's telemetry socket (port 9000) to the
/// source address of the phone's telemetry, so it arrives on the connection the phone already
/// sends telemetry from. It answers the phone frame with the same `seq`.
///
/// `{"type":"fpga_est","seq":1234,"t":5012.31,"v_x":1.02,"v_y":-0.01,"sigma_vx":0.02,
///   "sigma_vy":0.02,"distance":4.81,"net_forward":4.79,"status":547,"backend":"rtl-iverilog"}`
public struct FPGAEstimate: Sendable, Equatable, Codable {
    public static let typeName = "fpga_est"

    /// Sequence number of the phone frame this estimate answers.
    public var seq: UInt32
    /// That frame's timestamp (phone host clock, s).
    public var t: Double
    public var vx: Double
    public var vy: Double
    public var sigmaVx: Double
    public var sigmaVy: Double
    /// Total distance travelled (m), integrated by the dashboard from the FPGA's velocity.
    public var distance: Double
    /// Signed ∫v_x dt (m).
    public var netForward: Double
    public var status: StatusFlags
    /// How the FPGA was run, e.g. "rtl-iverilog" (clock-level Verilog) or "python-model".
    public var backend: String

    public init(seq: UInt32, t: Double, vx: Double, vy: Double, sigmaVx: Double, sigmaVy: Double,
                distance: Double, netForward: Double, status: StatusFlags, backend: String) {
        self.seq = seq; self.t = t; self.vx = vx; self.vy = vy
        self.sigmaVx = sigmaVx; self.sigmaVy = sigmaVy
        self.distance = distance; self.netForward = netForward
        self.status = status; self.backend = backend
    }

    public var speed: Double { (vx * vx + vy * vy).squareRoot() }

    enum CodingKeys: String, CodingKey {
        case type, seq, t, status, backend, distance
        case vx = "v_x", vy = "v_y", sigmaVx = "sigma_vx", sigmaVy = "sigma_vy", netForward = "net_forward"
    }

    public enum DecodeError: Error, Sendable, Equatable {
        case notAnEstimate
    }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        guard try c.decodeIfPresent(String.self, forKey: .type) == Self.typeName else {
            throw DecodeError.notAnEstimate
        }
        seq = try c.decodeIfPresent(UInt32.self, forKey: .seq) ?? 0
        t = try c.decode(Double.self, forKey: .t)
        vx = try c.decode(Double.self, forKey: .vx)
        vy = try c.decode(Double.self, forKey: .vy)
        sigmaVx = try c.decode(Double.self, forKey: .sigmaVx)
        sigmaVy = try c.decode(Double.self, forKey: .sigmaVy)
        distance = try c.decode(Double.self, forKey: .distance)
        netForward = try c.decode(Double.self, forKey: .netForward)
        status = StatusFlags(rawValue: try c.decodeIfPresent(UInt16.self, forKey: .status) ?? 0)
        backend = try c.decodeIfPresent(String.self, forKey: .backend) ?? "?"
    }

    public func encode(to encoder: Encoder) throws {
        var c = encoder.container(keyedBy: CodingKeys.self)
        try c.encode(Self.typeName, forKey: .type)
        try c.encode(seq, forKey: .seq)
        try c.encode(t, forKey: .t)
        try c.encode(vx, forKey: .vx)
        try c.encode(vy, forKey: .vy)
        try c.encode(sigmaVx, forKey: .sigmaVx)
        try c.encode(sigmaVy, forKey: .sigmaVy)
        try c.encode(distance, forKey: .distance)
        try c.encode(netForward, forKey: .netForward)
        try c.encode(status.rawValue, forKey: .status)
        try c.encode(backend, forKey: .backend)
    }

    /// Parses one datagram; nil for anything that is not an estimate (receivers ignore those).
    public static func decode(_ data: Data) -> FPGAEstimate? {
        guard data.first == UInt8(ascii: "{") else { return nil }
        return try? JSONDecoder().decode(FPGAEstimate.self, from: data)
    }

    public func encodeJSON() -> Data {
        let enc = JSONEncoder()
        enc.outputFormatting = [.sortedKeys]
        return (try? enc.encode(self)) ?? Data("{}".utf8)
    }
}
