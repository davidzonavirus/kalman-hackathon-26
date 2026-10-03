import Foundation

public enum TelemetryDecodeError: Error, Sendable, Equatable {
    case empty
    case badLength(Int)
    case badMagic(UInt8)
    case badVersion(UInt8)
    case badCRC(expected: UInt16, got: UInt16)
    case badJSON(String)
}

/// One telemetry sample, phone → dashboard (PROTOCOL.md §1).
///
/// Binary layout: 48 bytes little-endian, see `encodeBinary()`. Floating fields are
/// stored as `Double`/`Float` in Swift exactly matching their wire width so an
/// encode→decode roundtrip is bit-exact.
public struct TelemetryFrame: Sendable, Equatable, Codable {
    public static let magic: UInt8 = 0xA5
    public static let version: UInt8 = 1
    public static let binarySize = 48

    public var seq: UInt32
    public var t: Double
    public var vx: Float
    public var vy: Float
    public var sigmaVx: Float
    public var sigmaVy: Float
    public var distance: Float
    public var flowQuality: Float
    public var h: Float
    public var status: StatusFlags
    /// Percent 0–100, `0xFF` = unknown.
    public var battery: UInt8
    /// Torch level ×100 (0–100).
    public var torch: UInt8

    public init(seq: UInt32 = 0, t: Double = 0, vx: Float = 0, vy: Float = 0,
                sigmaVx: Float = 0, sigmaVy: Float = 0, distance: Float = 0,
                flowQuality: Float = 0, h: Float = 0, status: StatusFlags = [],
                battery: UInt8 = 0xFF, torch: UInt8 = 0) {
        self.seq = seq; self.t = t; self.vx = vx; self.vy = vy
        self.sigmaVx = sigmaVx; self.sigmaVy = sigmaVy; self.distance = distance
        self.flowQuality = flowQuality; self.h = h; self.status = status
        self.battery = battery; self.torch = torch
    }

    enum CodingKeys: String, CodingKey {
        case seq, t
        case vx = "v_x", vy = "v_y"
        case sigmaVx = "sigma_vx", sigmaVy = "sigma_vy"
        case distance
        case flowQuality = "flow_quality"
        case h, status, battery, torch
    }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        seq = try c.decode(UInt32.self, forKey: .seq)
        t = try c.decode(Double.self, forKey: .t)
        vx = try c.decode(Float.self, forKey: .vx)
        vy = try c.decode(Float.self, forKey: .vy)
        sigmaVx = try c.decode(Float.self, forKey: .sigmaVx)
        sigmaVy = try c.decode(Float.self, forKey: .sigmaVy)
        distance = try c.decode(Float.self, forKey: .distance)
        flowQuality = try c.decode(Float.self, forKey: .flowQuality)
        h = try c.decode(Float.self, forKey: .h)
        status = StatusFlags(rawValue: try c.decode(UInt16.self, forKey: .status))
        battery = try c.decode(UInt8.self, forKey: .battery)
        torch = try c.decode(UInt8.self, forKey: .torch)
    }

    public func encode(to encoder: Encoder) throws {
        var c = encoder.container(keyedBy: CodingKeys.self)
        try c.encode(seq, forKey: .seq)
        try c.encode(t, forKey: .t)
        try c.encode(vx, forKey: .vx)
        try c.encode(vy, forKey: .vy)
        try c.encode(sigmaVx, forKey: .sigmaVx)
        try c.encode(sigmaVy, forKey: .sigmaVy)
        try c.encode(distance, forKey: .distance)
        try c.encode(flowQuality, forKey: .flowQuality)
        try c.encode(h, forKey: .h)
        try c.encode(status.rawValue, forKey: .status)
        try c.encode(battery, forKey: .battery)
        try c.encode(torch, forKey: .torch)
    }

    // MARK: Binary

    /// Exactly 48 bytes, little-endian, CRC-16/CCITT-FALSE over bytes 0..45 at offset 46.
    public func encodeBinary() -> Data {
        var b = [UInt8]()
        b.reserveCapacity(Self.binarySize)
        b.append(Self.magic)
        b.append(Self.version)
        Self.put(seq, &b)
        Self.put(t.bitPattern, &b)
        for f in [vx, vy, sigmaVx, sigmaVy, distance, flowQuality, h] { Self.put(f.bitPattern, &b) }
        Self.put(status.rawValue, &b)
        b.append(battery)
        b.append(torch)
        Self.put(CRC16.ccittFalse(b), &b)
        assert(b.count == Self.binarySize)
        return Data(b)
    }

    /// JSON debug frame with the exact snake_case keys from PROTOCOL.md (sorted keys).
    public func encodeJSON() -> Data {
        let enc = JSONEncoder()
        enc.outputFormatting = [.sortedKeys]
        // Encoding a struct of plain numbers cannot fail.
        return (try? enc.encode(self)) ?? Data()
    }

    /// Decodes either format: a first byte of `{` means JSON, otherwise binary.
    public static func decode(_ data: Data) throws -> TelemetryFrame {
        guard let first = data.first else { throw TelemetryDecodeError.empty }
        if first == UInt8(ascii: "{") {
            do { return try JSONDecoder().decode(TelemetryFrame.self, from: data) }
            catch { throw TelemetryDecodeError.badJSON(String(describing: error)) }
        }
        let b = [UInt8](data)
        guard b.count == binarySize else { throw TelemetryDecodeError.badLength(b.count) }
        guard b[0] == magic else { throw TelemetryDecodeError.badMagic(b[0]) }
        guard b[1] == version else { throw TelemetryDecodeError.badVersion(b[1]) }
        let expected = CRC16.ccittFalse(b[0..<46])
        let got: UInt16 = get(b, 46)
        guard expected == got else { throw TelemetryDecodeError.badCRC(expected: expected, got: got) }
        func f(_ o: Int) -> Float { Float(bitPattern: get(b, o)) }
        return TelemetryFrame(
            seq: get(b, 2), t: Double(bitPattern: get(b, 6)),
            vx: f(14), vy: f(18), sigmaVx: f(22), sigmaVy: f(26),
            distance: f(30), flowQuality: f(34), h: f(38),
            status: StatusFlags(rawValue: get(b, 42)), battery: b[44], torch: b[45])
    }

    // MARK: LE helpers

    private static func put<T: FixedWidthInteger>(_ v: T, _ b: inout [UInt8]) {
        var le = v.littleEndian
        withUnsafeBytes(of: &le) { b.append(contentsOf: $0) }
    }

    private static func get<T: FixedWidthInteger>(_ b: [UInt8], _ off: Int) -> T {
        var v: T = 0
        for i in 0..<MemoryLayout<T>.size { v |= T(b[off + i]) << (8 * i) }
        return v
    }
}
