/// Telemetry status bit flags (PROTOCOL.md §1). Bits 11–15 reserved (0).
public struct StatusFlags: OptionSet, Sendable, Hashable, Codable {
    public let rawValue: UInt16
    public init(rawValue: UInt16) { self.rawValue = rawValue }

    public static let imuOK       = StatusFlags(rawValue: 1 << 0)
    public static let flowOK      = StatusFlags(rawValue: 1 << 1)
    public static let gnssOK      = StatusFlags(rawValue: 1 << 2)
    public static let lidarOK     = StatusFlags(rawValue: 1 << 3)
    public static let recording   = StatusFlags(rawValue: 1 << 4)
    public static let zupt        = StatusFlags(rawValue: 1 << 5)
    public static let flowGated   = StatusFlags(rawValue: 1 << 6)
    public static let gnssGated   = StatusFlags(rawValue: 1 << 7)
    public static let torchOn     = StatusFlags(rawValue: 1 << 8)
    public static let filterInit  = StatusFlags(rawValue: 1 << 9)
    public static let calibrating = StatusFlags(rawValue: 1 << 10)

    /// Human-readable names of set bits (for logs / debug UIs).
    public var names: [String] {
        let all: [(StatusFlags, String)] = [
            (.imuOK, "IMU_OK"), (.flowOK, "FLOW_OK"), (.gnssOK, "GNSS_OK"), (.lidarOK, "LIDAR_OK"),
            (.recording, "RECORDING"), (.zupt, "ZUPT"), (.flowGated, "FLOW_GATED"),
            (.gnssGated, "GNSS_GATED"), (.torchOn, "TORCH_ON"), (.filterInit, "FILTER_INIT"),
            (.calibrating, "CALIBRATING"),
        ]
        return all.filter { contains($0.0) }.map { $0.1 }
    }
}
