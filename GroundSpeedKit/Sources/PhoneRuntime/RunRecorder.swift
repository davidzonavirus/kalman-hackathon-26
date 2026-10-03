import Foundation
import SpeedProtocol

/// Per-run CSV logger. Layout and headers follow docs/PROTOCOL.md §3 exactly:
/// `<runsDirectory>/<run_id>/{imu,flow,gnss,depth,est,events}.csv` + `meta.json`.
///
/// Thread-safe: every public method may be called from any thread. Rows are formatted on
/// the caller's thread (cheap) and appended to in-memory buffers on a private serial queue;
/// buffers are flushed to disk about once per second and on stop.
public final class RunRecorder: @unchecked Sendable {
    public enum Stream: String, CaseIterable, Sendable {
        case imu, flow, gnss, depth, est, events

        public var fileName: String {
            switch self {
            case .imu: return CSVSchema.imuFile
            case .flow: return CSVSchema.flowFile
            case .gnss: return CSVSchema.gnssFile
            case .depth: return CSVSchema.depthFile
            case .est: return CSVSchema.estFile
            case .events: return CSVSchema.eventsFile
            }
        }

        public var header: String {
            switch self {
            case .imu: return CSVSchema.imuHeader
            case .flow: return CSVSchema.flowHeader
            case .gnss: return CSVSchema.gnssHeader
            case .depth: return CSVSchema.depthHeader
            case .est: return CSVSchema.estHeader
            case .events: return CSVSchema.eventsHeader
            }
        }
    }

    public let runsDirectory: URL
    public var flushInterval: TimeInterval = 1.0

    private let queue = DispatchQueue(label: "gsk.recorder", qos: .utility)
    private var handles: [Stream: FileHandle] = [:]
    private var buffers: [Stream: Data] = [:]
    private var timer: DispatchSourceTimer?
    private var _runId: String?
    private var _runURL: URL?
    private let stateLock = NSLock()
    private var _isRecording = false

    public init(runsDirectory: URL) {
        self.runsDirectory = runsDirectory
    }

    /// `Documents/runs` on iOS (and the user's Documents/runs elsewhere).
    public static func defaultRunsDirectory() -> URL {
        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first
            ?? URL(fileURLWithPath: NSTemporaryDirectory())
        return docs.appendingPathComponent("runs", isDirectory: true)
    }

    public var isRecording: Bool {
        stateLock.lock(); defer { stateLock.unlock() }
        return _isRecording
    }

    public var currentRunId: String? {
        stateLock.lock(); defer { stateLock.unlock() }
        return _runId
    }

    public var currentRunURL: URL? {
        stateLock.lock(); defer { stateLock.unlock() }
        return _runURL
    }

    /// Restrict labels to filesystem-safe characters.
    public static func sanitizeLabel(_ label: String?) -> String? {
        guard let label, !label.isEmpty else { return nil }
        let allowed = Set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-.")
        let cleaned = String(label.map { allowed.contains($0) ? $0 : "_" })
        return cleaned.isEmpty ? nil : String(cleaned.prefix(64))
    }

    public static func makeRunId(label: String?, date: Date = Date()) -> String {
        var id = Clock.runIdTimestamp(date)
        if let l = sanitizeLabel(label) { id += "_" + l }
        return id
    }

    /// Creates the run folder and all CSV files (with headers). Returns the run id.
    /// If a run is already active it is stopped first (without meta).
    @discardableResult
    public func startRun(label: String?, date: Date = Date()) throws -> String {
        if isRecording { _ = stopRun(meta: Optional<RunMeta>.none) }
        var runId = Self.makeRunId(label: label, date: date)
        let fm = FileManager.default
        try fm.createDirectory(at: runsDirectory, withIntermediateDirectories: true)
        var url = runsDirectory.appendingPathComponent(runId, isDirectory: true)
        var n = 2
        while fm.fileExists(atPath: url.path) {
            runId = Self.makeRunId(label: label, date: date) + "_\(n)"
            url = runsDirectory.appendingPathComponent(runId, isDirectory: true)
            n += 1
        }
        try fm.createDirectory(at: url, withIntermediateDirectories: true)

        var newHandles: [Stream: FileHandle] = [:]
        for s in Stream.allCases {
            let f = url.appendingPathComponent(s.fileName)
            guard fm.createFile(atPath: f.path, contents: Data((s.header + "\n").utf8)) else {
                throw CocoaError(.fileWriteUnknown, userInfo: [NSFilePathErrorKey: f.path])
            }
            let h = try FileHandle(forWritingTo: f)
            try h.seekToEnd()
            newHandles[s] = h
        }

        queue.sync {
            self.handles = newHandles
            self.buffers = [:]
            let t = DispatchSource.makeTimerSource(queue: self.queue)
            t.schedule(deadline: .now() + flushInterval, repeating: flushInterval)
            t.setEventHandler { [weak self] in self?.flushLocked() }
            t.resume()
            self.timer = t
        }
        stateLock.lock()
        _runId = runId
        _runURL = url
        _isRecording = true
        stateLock.unlock()
        return runId
    }

    /// Flushes, closes all files, writes `meta.json` (if given). Returns the run folder.
    @discardableResult
    public func stopRun<M: Encodable>(meta: M?) -> URL? {
        stateLock.lock()
        let url = _runURL
        let wasRecording = _isRecording
        _isRecording = false
        _runId = nil
        _runURL = nil
        stateLock.unlock()
        guard wasRecording, let url else { return nil }

        queue.sync {
            self.timer?.cancel()
            self.timer = nil
            self.flushLocked()
            for (_, h) in self.handles { try? h.close() }
            self.handles = [:]
            self.buffers = [:]
        }
        if let meta {
            let enc = JSONEncoder()
            enc.outputFormatting = [.prettyPrinted, .sortedKeys]
            if let data = try? enc.encode(meta) {
                try? data.write(to: url.appendingPathComponent(CSVSchema.metaFile), options: .atomic)
            }
        }
        return url
    }

    /// Writes (or overwrites) meta.json for the active run, e.g. right at start so a crash
    /// still leaves metadata behind.
    public func writeMeta<M: Encodable>(_ meta: M) {
        guard let url = currentRunURL else { return }
        let enc = JSONEncoder()
        enc.outputFormatting = [.prettyPrinted, .sortedKeys]
        if let data = try? enc.encode(meta) {
            try? data.write(to: url.appendingPathComponent(CSVSchema.metaFile), options: .atomic)
        }
    }

    /// Force a flush now (blocks until written).
    public func flush() {
        queue.sync { self.flushLocked() }
    }

    // MARK: - Row appenders (no-ops when not recording)

    public func recordIMU(t: Double, ax: Double, ay: Double, az: Double, gx: Double, gy: Double, gz: Double) {
        append(.imu, row(t, ax, ay, az, gx, gy, gz))
    }

    public func recordFlow(t: Double, vx: Double, vy: Double, quality: Double, h: Double) {
        append(.flow, row(t, vx, vy, quality, h))
    }

    public func recordGNSS(t: Double, speed: Double, speedAcc: Double, courseDeg: Double) {
        append(.gnss, row(t, speed, speedAcc, courseDeg))
    }

    public func recordDepth(t: Double, h: Double) {
        append(.depth, row(t, h))
    }

    public func recordEstimate(t: Double, vx: Double, vy: Double, sigmaVx: Double, sigmaVy: Double,
                               distance: Double, status: UInt16, flowQuality: Double, h: Double) {
        append(.est, CSVRow.join([CSVRow.time(t), CSVRow.real(vx), CSVRow.real(vy), CSVRow.real(sigmaVx),
                                  CSVRow.real(sigmaVy), CSVRow.real(distance), String(status),
                                  CSVRow.real(flowQuality), CSVRow.real(h)]) + "\n")
    }

    public func recordEvent(t: Double, event: String, value: String = "") {
        append(.events, CSVRow.join([CSVRow.time(t), CSVRow.text(event), CSVRow.text(value)]) + "\n")
    }

    // MARK: - Internals

    /// Formatting follows `CSVRow` (t: %.6f, reals: %.9g) so every writer matches Python.
    private func row(_ t: Double, _ xs: Double...) -> String {
        CSVRow.format(t: t, xs) + "\n"
    }

    private func append(_ stream: Stream, _ line: String) {
        guard isRecording else { return }
        queue.async {
            guard self.handles[stream] != nil else { return }
            self.buffers[stream, default: Data()].append(contentsOf: line.utf8)
        }
    }

    private func flushLocked() {
        for (s, data) in buffers where !data.isEmpty {
            if let h = handles[s] {
                try? h.write(contentsOf: data)
            }
            buffers[s] = Data()
        }
    }
}

// MARK: - Run folder management / export

public struct RunSummary: Identifiable, Hashable, Sendable {
    public var id: String { runId }
    public let runId: String
    public let url: URL
    public let created: Date
    public let sizeBytes: Int64
}

public extension RunRecorder {
    /// Lists run folders, newest first.
    func listRuns() -> [RunSummary] {
        let fm = FileManager.default
        guard let items = try? fm.contentsOfDirectory(at: runsDirectory,
                                                      includingPropertiesForKeys: [.creationDateKey, .isDirectoryKey],
                                                      options: [.skipsHiddenFiles]) else { return [] }
        var out: [RunSummary] = []
        for u in items {
            let rv = try? u.resourceValues(forKeys: [.creationDateKey, .isDirectoryKey])
            guard rv?.isDirectory == true else { continue }
            var size: Int64 = 0
            if let files = try? fm.contentsOfDirectory(at: u, includingPropertiesForKeys: [.fileSizeKey]) {
                for f in files {
                    size += Int64((try? f.resourceValues(forKeys: [.fileSizeKey]).fileSize) ?? 0)
                }
            }
            out.append(RunSummary(runId: u.lastPathComponent, url: u,
                                  created: rv?.creationDate ?? .distantPast, sizeBytes: size))
        }
        return out.sorted { $0.runId > $1.runId }
    }

    func deleteRun(_ runId: String) throws {
        try FileManager.default.removeItem(at: runsDirectory.appendingPathComponent(runId, isDirectory: true))
    }

    /// Zips a run folder using `NSFileCoordinator(.forUploading)` (Foundation, works on iOS and
    /// macOS) and copies the archive to `destinationDirectory/<run_id>.zip`.
    static func zipRunFolder(_ folder: URL, destinationDirectory: URL = FileManager.default.temporaryDirectory) throws -> URL {
        let coordinator = NSFileCoordinator()
        var coordError: NSError?
        var resultURL: URL?
        var innerError: Error?
        coordinator.coordinate(readingItemAt: folder, options: [.forUploading], error: &coordError) { zipURL in
            do {
                let dest = destinationDirectory.appendingPathComponent(folder.lastPathComponent + ".zip")
                let fm = FileManager.default
                if fm.fileExists(atPath: dest.path) { try fm.removeItem(at: dest) }
                try fm.copyItem(at: zipURL, to: dest)
                resultURL = dest
            } catch {
                innerError = error
            }
        }
        if let coordError { throw coordError }
        if let innerError { throw innerError }
        guard let resultURL else { throw CocoaError(.fileWriteUnknown) }
        return resultURL
    }
}
