import Foundation

/// Dashboard → phone command (PROTOCOL.md §2). One JSON object per line, `"cmd"` key.
public enum Command: Sendable, Equatable, Codable {
    case ping
    case startRun(label: String?)
    case stopRun
    case mark(label: String?)
    case calibrate
    /// 0.0 = off … 1.0 = full.
    case setTorch(level: Double)
    case resetDistance

    /// Wire name used in the `"cmd"` field.
    public var name: String {
        switch self {
        case .ping: return "ping"
        case .startRun: return "start_run"
        case .stopRun: return "stop_run"
        case .mark: return "mark"
        case .calibrate: return "calibrate"
        case .setTorch: return "set_torch"
        case .resetDistance: return "reset_distance"
        }
    }

    enum CodingKeys: String, CodingKey { case cmd, label, level }

    public enum ParseError: Error, Sendable, Equatable {
        case unknownCommand(String)
        case missingField(String)
        case invalidJSON
    }

    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        let cmd = try c.decode(String.self, forKey: .cmd)
        switch cmd {
        case "ping": self = .ping
        case "start_run": self = .startRun(label: try c.decodeIfPresent(String.self, forKey: .label))
        case "stop_run": self = .stopRun
        case "mark": self = .mark(label: try c.decodeIfPresent(String.self, forKey: .label))
        case "calibrate": self = .calibrate
        case "set_torch":
            guard let lvl = try c.decodeIfPresent(Double.self, forKey: .level) else {
                throw ParseError.missingField("level")
            }
            self = .setTorch(level: lvl)
        case "reset_distance": self = .resetDistance
        default: throw ParseError.unknownCommand(cmd)
        }
    }

    public func encode(to encoder: Encoder) throws {
        var c = encoder.container(keyedBy: CodingKeys.self)
        try c.encode(name, forKey: .cmd)
        switch self {
        case .startRun(let label), .mark(let label):
            try c.encodeIfPresent(label, forKey: .label)
        case .setTorch(let level):
            try c.encode(level, forKey: .level)
        default: break
        }
    }

    /// Parse one line (with or without trailing newline).
    public static func parse(_ line: Data) throws -> Command {
        do { return try JSONDecoder().decode(Command.self, from: line) }
        catch let e as ParseError { throw e }
        catch DecodingError.keyNotFound(let k, _) { throw ParseError.missingField(k.stringValue) }
        catch { throw ParseError.invalidJSON }
    }

    public static func parse(_ line: String) throws -> Command {
        try parse(Data(line.utf8))
    }

    /// JSON object + `\n`.
    public func encodeLine() -> Data { ndjsonLine(self) }
}

/// Reply to a command: `{"ok":true,"cmd":"start_run","run":"…"}` or `{"ok":false,"cmd":"…","error":"…"}`.
public struct CommandReply: Sendable, Equatable, Codable {
    public var ok: Bool
    public var cmd: String
    public var run: String?
    public var error: String?
    /// Run distance at stop (m); carried by `stop_run` replies, omitted otherwise.
    public var distance: Double?

    public init(ok: Bool, cmd: String, run: String? = nil, error: String? = nil, distance: Double? = nil) {
        self.ok = ok; self.cmd = cmd; self.run = run; self.error = error; self.distance = distance
    }

    public static func success(_ cmd: String, run: String? = nil, distance: Double? = nil) -> CommandReply {
        CommandReply(ok: true, cmd: cmd, run: run, distance: distance)
    }
    public static func failure(_ cmd: String, _ error: String) -> CommandReply {
        CommandReply(ok: false, cmd: cmd, error: error)
    }

    public func encodeLine() -> Data { ndjsonLine(self) }

    public static func parse(_ line: Data) throws -> CommandReply {
        try JSONDecoder().decode(CommandReply.self, from: line)
    }
}

/// Splits a TCP byte stream into newline-delimited lines (tolerates `\r\n`).
public struct LineBuffer: Sendable {
    private var buf = Data()
    public init() {}

    /// Appends bytes and returns every complete line (without the terminator, empty lines dropped).
    public mutating func append(_ data: Data) -> [Data] {
        buf.append(data)
        var lines: [Data] = []
        while let nl = buf.firstIndex(of: 0x0A) {
            var line = buf[buf.startIndex..<nl]
            if line.last == 0x0D { line = line.dropLast() }
            if !line.isEmpty { lines.append(Data(line)) }
            buf = Data(buf[(nl + 1)...])
        }
        return lines
    }
}

private func ndjsonLine<T: Encodable>(_ v: T) -> Data {
    let enc = JSONEncoder()
    enc.outputFormatting = [.sortedKeys, .withoutEscapingSlashes]
    var d = (try? enc.encode(v)) ?? Data("{}".utf8)
    d.append(0x0A)
    return d
}
