import Foundation

public enum TelemetryDecodeError: Error, Sendable, Equatable {
    case empty
    case badLength(Int)
    case badMagic(UInt8)
    case badVersion(UInt8)
    case badCRC(expected: UInt16, got: UInt16)
    case badJSON(String)
}

/// One telemetry sample, phone -> dashboard (PROTOCOL.md §1).
///
/// Binary layout: v1 = 48 bytes, v2 = 72 bytes (adds accel, yaw rate, raw flow, net forward)
/// little-endian, see `encodeBinary()`. `version` selects what is encoded; decode accepts both. Floating fields are
/// stored as `Double`/`Float` in Swift exactly matching their wire width so an
/// encode->decode roundtrip is bit-exact.
public struct TelemetryFrame: Sendable, Equatable, Codable {
    public static let magic: UInt8 = 0xA5
    /// Default (latest) wire version.
    public static let currentVersion: UInt8 = 2
    public static let binarySizeV1 = 48
    public static let binarySizeV2 = 72

    public var version: UInt8

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
    // v2 fields (0 in decoded v1 frames)
    public var ax: Float
    public var ay: Float
    public var gz: Float
    public var flowVx: Float
    public var flowVy: Float
    public var netForward: Float

    public var binarySize: Int { version == 1 ? Self.binarySizeV1 : Self.binarySizeV2 }

    public init(seq: UInt32 = 0, t: Double = 0, vx: Float = 0, vy: Float = 0,
                sigmaVx: Float = 0, sigmaVy: Float = 0, distance: Float = 0,
                flowQuality: Float = 0, h: Float = 0, status: StatusFlags = [],
                battery: UInt8 = 0xFF, torch: UInt8 = 0,
                ax: Float = 0, ay: Float = 0, gz: Float = 0,
                flowVx: Float = 0, flowVy: Float = 0, netForward: Float = 0,
                version: UInt8 = TelemetryFrame.currentVersion) {
        self.seq = seq; self.t = t; self.vx = vx; self.vy = vy
        self.sigmaVx = sigmaVx; self.sigmaVy = sigmaVy; self.distance = distance
        self.flowQuality = flowQuality; self.h = h; self.status = status
        self.battery = battery; self.torch = torch
        self.ax = ax; self.ay = ay; self.gz = gz
        self.flowVx = flowVx; self.flowVy = flowVy; self.netForward = netForward
        self.version = version
    }

    enum CodingKeys: String, CodingKey {
        case seq, t
        case vx = "v_x", vy = "v_y"
        case sigmaVx = "sigma_vx", sigmaVy = "sigma_vy"
        case distance
        case flowQuality = "flow_quality"
        case h, status, battery, torch
        case version, ax, ay, gz, flowVx = "flow_vx", flowVy = "flow_vy", netForward = "net_forward"
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
        version = try c.decodeIfPresent(UInt8.self, forKey: .version) ?? 1
        ax = try c.decodeIfPresent(Float.self, forKey: .ax) ?? 0
        ay = try c.decodeIfPresent(Float.self, forKey: .ay) ?? 0
        gz = try c.decodeIfPresent(Float.self, forKey: .gz) ?? 0
        flowVx = try c.decodeIfPresent(Float.self, forKey: .flowVx) ?? 0
        flowVy = try c.decodeIfPresent(Float.self, forKey: .flowVy) ?? 0
        netForward = try c.decodeIfPresent(Float.self, forKey: .netForward) ?? 0
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
        guard version >= 2 else { return }   // v1 JSON keeps exactly the v1 key set
        try c.encode(version, forKey: .version)
        try c.encode(ax, forKey: .ax)
        try c.encode(ay, forKey: .ay)
        try c.encode(gz, forKey: .gz)
        try c.encode(flowVx, forKey: .flowVx)
        try c.encode(flowVy, forKey: .flowVy)
        try c.encode(netForward, forKey: .netForward)
    }

    // MARK: Binary

    /// v1: 48 bytes, CRC over 0..45 at 46. v2: 72 bytes, bytes 0..45 as v1, then
    /// ax, ay, gz, flow_vx, flow_vy, net_forward (f32) and CRC over 0..69 at 70.
    public func encodeBinary() -> Data {
        var b = [UInt8]()
        b.reserveCapacity(binarySize)
        b.append(Self.magic)
        b.append(version == 1 ? 1 : 2)
        Self.put(seq, &b)
        Self.put(t.bitPattern, &b)
        for f in [vx, vy, sigmaVx, sigmaVy, distance, flowQuality, h] { Self.put(f.bitPattern, &b) }
        Self.put(status.rawValue, &b)
        b.append(battery)
        b.append(torch)
        if version >= 2 {
            for f in [ax, ay, gz, flowVx, flowVy, netForward] { Self.put(f.bitPattern, &b) }
        }
        Self.put(CRC16.ccittFalse(b), &b)
        assert(b.count == binarySize)
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
        guard b.count >= 2 else { throw TelemetryDecodeError.badLength(b.count) }
        guard b[0] == magic else { throw TelemetryDecodeError.badMagic(b[0]) }
        let ver = b[1]
        let size: Int
        switch ver {
        case 1: size = binarySizeV1
        case 2: size = binarySizeV2
        default: throw TelemetryDecodeError.badVersion(ver)
        }
        guard b.count == size else { throw TelemetryDecodeError.badLength(b.count) }
        let expected = CRC16.ccittFalse(b[0..<(size - 2)])
        let got: UInt16 = get(b, size - 2)
        guard expected == got else { throw TelemetryDecodeError.badCRC(expected: expected, got: got) }
        func f(_ o: Int) -> Float { Float(bitPattern: get(b, o)) }
        var fr = TelemetryFrame(
            seq: get(b, 2), t: Double(bitPattern: get(b, 6)),
            vx: f(14), vy: f(18), sigmaVx: f(22), sigmaVy: f(26),
            distance: f(30), flowQuality: f(34), h: f(38),
            status: StatusFlags(rawValue: get(b, 42)), battery: b[44], torch: b[45], version: ver)
        if ver >= 2 {
            fr.ax = f(46); fr.ay = f(50); fr.gz = f(54)
            fr.flowVx = f(58); fr.flowVy = f(62); fr.netForward = f(66)
        }
        return fr
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
