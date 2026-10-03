import Foundation

/// Per-run CSV files and their frozen headers (PROTOCOL.md §3).
public enum CSVSchema {
    public static let imuFile = "imu.csv"
    public static let flowFile = "flow.csv"
    public static let gnssFile = "gnss.csv"
    public static let depthFile = "depth.csv"
    public static let estFile = "est.csv"
    public static let eventsFile = "events.csv"
    public static let metaFile = "meta.json"

    public static let imuHeader = "t,ax,ay,az,gx,gy,gz"
    public static let flowHeader = "t,vx_cam,vy_cam,quality,h"
    public static let gnssHeader = "t,speed,speed_acc,course"
    public static let depthHeader = "t,h"
    public static let estHeader = "t,v_x,v_y,sigma_vx,sigma_vy,distance,status,flow_quality,h"
    public static let eventsHeader = "t,event,value"

    /// filename → header, for writers that open all files at once.
    public static let headers: [String: String] = [
        imuFile: imuHeader, flowFile: flowHeader, gnssFile: gnssHeader,
        depthFile: depthHeader, estFile: estHeader, eventsFile: eventsHeader,
    ]

    /// Event names used in `events.csv` (`value` column is free text / number).
    public enum Event {
        public static let start = "start"
        public static let stop = "stop"
        public static let mark = "mark"
        public static let torch = "torch"
        public static let calibrate = "calibrate"
        /// Replay applies a zero-velocity update at this timestamp.
        public static let zupt = "zupt"
        public static let resetDistance = "reset_distance"
    }
}

/// Number formatting for CSV rows.
///
/// Convention (all writers and the Python side should match):
/// - `t` (first column) is printed `%.6f` → microsecond resolution. Mach-clock seconds are
///   ~1e5–1e6, so 6 decimals keep ≤ 16 significant digits, within Double precision.
/// - every other real value is printed `%.9g` (≥ float32 round-trip, ~1e-9 relative for
///   doubles; plenty for the 1e-6 m/s replay cross-check).
/// - integers (status, battery) are printed as plain integers.
/// - text fields are quoted only if they contain `,` `"` or a newline.
public enum CSVRow {
    public static func time(_ t: Double) -> String { String(format: "%.6f", t) }
    public static func real(_ v: Double) -> String { String(format: "%.9g", v) }

    /// `t` followed by real values: `"12.345678,1.5,0.25"`.
    public static func format(t: Double, _ values: [Double]) -> String {
        var s = time(t)
        for v in values { s += ","; s += real(v) }
        return s
    }

    /// Mixed row with already-formatted cells.
    public static func join(_ cells: [String]) -> String { cells.joined(separator: ",") }

    /// Quotes a text cell when needed (RFC 4180 style).
    public static func text(_ s: String) -> String {
        if s.contains(where: { $0 == "," || $0 == "\"" || $0 == "\n" || $0 == "\r" }) {
            return "\"" + s.replacingOccurrences(of: "\"", with: "\"\"") + "\""
        }
        return s
    }

    /// Minimal CSV line splitter that understands the quoting produced by `text(_:)`.
    public static func split(_ line: Substring) -> [String] {
        var out: [String] = []
        var cur = ""
        var inQuotes = false
        var it = line.makeIterator()
        while let ch = it.next() {
            if inQuotes {
                if ch == "\"" {
                    // Peek: doubled quote is an escaped quote.
                    if let nx = it.next() {
                        if nx == "\"" { cur.append("\"") }
                        else { inQuotes = false; if nx == "," { out.append(cur); cur = "" } else { cur.append(nx) } }
                    } else { inQuotes = false }
                } else { cur.append(ch) }
            } else if ch == "\"" { inQuotes = true }
            else if ch == "," { out.append(cur); cur = "" }
            else if ch != "\r" { cur.append(ch) }
        }
        out.append(cur)
        return out
    }
}

/// A loosely typed JSON value, for free-form sub-objects in `meta.json` (`mount_config`).
public enum JSONValue: Sendable, Equatable, Codable {
    case null
    case bool(Bool)
    case number(Double)
    case string(String)
    case array([JSONValue])
    case object([String: JSONValue])

    public init(from decoder: Decoder) throws {
        let c = try decoder.singleValueContainer()
        if c.decodeNil() { self = .null }
        else if let b = try? c.decode(Bool.self) { self = .bool(b) }
        else if let n = try? c.decode(Double.self) { self = .number(n) }
        else if let s = try? c.decode(String.self) { self = .string(s) }
        else if let a = try? c.decode([JSONValue].self) { self = .array(a) }
        else { self = .object(try c.decode([String: JSONValue].self)) }
    }

    public func encode(to encoder: Encoder) throws {
        var c = encoder.singleValueContainer()
        switch self {
        case .null: try c.encodeNil()
        case .bool(let b): try c.encode(b)
        case .number(let n): try c.encode(n)
        case .string(let s): try c.encode(s)
        case .array(let a): try c.encode(a)
        case .object(let o): try c.encode(o)
        }
    }

    /// Converts any Encodable (e.g. `MountMapping`, `FilterConfig`) to a JSONValue.
    public static func from<T: Encodable>(_ v: T) throws -> JSONValue {
        try JSONDecoder().decode(JSONValue.self, from: JSONEncoder().encode(v))
    }

    /// Decodes this value as `T`.
    public func decode<T: Decodable>(_ type: T.Type) throws -> T {
        try JSONDecoder().decode(T.self, from: JSONEncoder().encode(self))
    }
}

/// `meta.json`, written once per run (PROTOCOL.md §3). Keys are snake_case on disk.
///
/// `mount_config`, `q`, `r` are free-form objects so SpeedProtocol need not depend on
/// OpticalFlow/KalmanCore. Typical contents: `mount_config` = encoded `MountMapping`;
/// `q` = `{"q_accel":…, "q_bias":…}`; `r` = `{"r_flow_base":…, "psr_ref":…, …}`
/// (`FilterConfig.qDict` / `.rDict` in KalmanCore produce these).
public struct RunMeta: Sendable, Equatable, Codable {
    public var runId: String
    public var label: String?
    public var appVersion: String
    public var device: String
    public var filterName: String
    public var mountHeightM: Double
    public var torchLevel: Double
    public var scaleFactor: Double
    public var focalPx: Double
    public var mountConfig: JSONValue
    public var startT: Double
    public var endT: Double?
    public var batteryStart: Int?
    public var batteryEnd: Int?
    public var q: [String: Double]
    public var r: [String: Double]

    enum CodingKeys: String, CodingKey {
        case runId = "run_id", label
        case appVersion = "app_version", device
        case filterName = "filter_name"
        case mountHeightM = "mount_height_m"
        case torchLevel = "torch_level"
        case scaleFactor = "scale_factor"
        case focalPx = "focal_px"
        case mountConfig = "mount_config"
        case startT = "start_t", endT = "end_t"
        case batteryStart = "battery_start", batteryEnd = "battery_end"
        case q, r
    }

    public init(runId: String, label: String? = nil, appVersion: String = "0.1", device: String = "unknown",
                filterName: String, mountHeightM: Double, torchLevel: Double = 0, scaleFactor: Double = 1,
                focalPx: Double, mountConfig: JSONValue = .object([:]), startT: Double, endT: Double? = nil,
                batteryStart: Int? = nil, batteryEnd: Int? = nil,
                q: [String: Double] = [:], r: [String: Double] = [:]) {
        self.runId = runId; self.label = label; self.appVersion = appVersion; self.device = device
        self.filterName = filterName; self.mountHeightM = mountHeightM; self.torchLevel = torchLevel
        self.scaleFactor = scaleFactor; self.focalPx = focalPx; self.mountConfig = mountConfig
        self.startT = startT; self.endT = endT; self.batteryStart = batteryStart
        self.batteryEnd = batteryEnd; self.q = q; self.r = r
    }

    /// Always writes every key (nil → `null`) so the file shape is stable for Python.
    public func encode(to encoder: Encoder) throws {
        var c = encoder.container(keyedBy: CodingKeys.self)
        try c.encode(runId, forKey: .runId)
        try c.encode(label, forKey: .label)
        try c.encode(appVersion, forKey: .appVersion)
        try c.encode(device, forKey: .device)
        try c.encode(filterName, forKey: .filterName)
        try c.encode(mountHeightM, forKey: .mountHeightM)
        try c.encode(torchLevel, forKey: .torchLevel)
        try c.encode(scaleFactor, forKey: .scaleFactor)
        try c.encode(focalPx, forKey: .focalPx)
        try c.encode(mountConfig, forKey: .mountConfig)
        try c.encode(startT, forKey: .startT)
        try c.encode(endT, forKey: .endT)
        try c.encode(batteryStart, forKey: .batteryStart)
        try c.encode(batteryEnd, forKey: .batteryEnd)
        try c.encode(q, forKey: .q)
        try c.encode(r, forKey: .r)
    }

    public func encodedJSON() -> Data {
        let enc = JSONEncoder()
        enc.outputFormatting = [.prettyPrinted, .sortedKeys, .withoutEscapingSlashes]
        return (try? enc.encode(self)) ?? Data()
    }
}
