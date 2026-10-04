import Foundation
import KalmanCore
import SpeedProtocol

/// Wires engine + recorder + UDP telemetry + TCP command server together and implements the
/// command set (PROTOCOL.md §2). Platform-agnostic: the iOS app and `phone-sim` both use it;
/// only the sensor sources differ.
public final class GroundSpeedRuntime: CommandHandling, @unchecked Sendable {
    /// Host-provided hooks (all optional, called from arbitrary threads).
    public struct Hooks {
        /// Battery percent 0–100, nil = unknown.
        public var battery: @Sendable () -> Int? = { nil }
        /// Apply torch level 0…1 to the hardware. Return the level actually applied.
        public var setTorch: @Sendable (Double) -> Double = { $0 }
        /// Focal length (px, full-res) of the active camera format, for meta.json.
        public var focalPx: @Sendable () -> Double = { 0 }
        /// Measure camera height (LiDAR); call `done(height or nil, message)` when finished.
        /// nil = not supported on this host.
        public var measureHeight: (@Sendable (_ done: @escaping @Sendable (Double?, String) -> Void) -> Void)?
        public var deviceName: String = "unknown"
        public var appVersion: String = "0.1"
        public init() {}
    }

    public let engine: SensorFusionEngine
    public let recorder: RunRecorder
    public let sender: TelemetrySender
    public let server: CommandServer
    public var hooks = Hooks()

    /// Called (any thread) whenever run state / connection state changes, so UIs can refresh.
    public var onStateChange: (@Sendable () -> Void)?

    private let lock = NSLock()
    private var _settings: RuntimeSettings
    private var _torchLevel: Double = 0
    private var _meta: RunMeta?
    private var _dashboardPeer: String?
    private var _lastRunId: String?
    private var _lastRunDistance: Double?
    /// Newest Kalman estimate a dashboard computed on the FPGA (PROTOCOL.md §4) + when it arrived.
    private var _fpga: (estimate: FPGAEstimate, receivedAt: Double)?
    private var _fpgaCount: UInt64 = 0

    public init(settings: RuntimeSettings, runsDirectory: URL = RunRecorder.defaultRunsDirectory(),
                bonjourName: String? = nil) {
        _settings = settings
        engine = SensorFusionEngine(filterName: settings.filterName, config: settings.filterConfig,
                                    settings: settings.engineSettings)
        recorder = RunRecorder(runsDirectory: runsDirectory)
        sender = TelemetrySender(host: settings.dashboardHost, port: settings.telemetryPort,
                                 useJSON: settings.useJSON)
        server = CommandServer(port: settings.commandPort, bonjourName: bonjourName)
        engine.recorder = recorder
        server.handler = self
        sender.frameSource = { [weak self] in self?.makeFrame() }
        server.onPeerConnected = { [weak self] ip in
            guard let self else { return }
            self.lock.withLock { self._dashboardPeer = ip }
            self.sender.setDestination(host: ip)
            self.engine.recordEvent("dashboard_connected", value: ip)
            self.onStateChange?()
        }
        server.onClientCountChanged = { [weak self] _ in self?.onStateChange?() }
        sender.onDatagram = { [weak self] data in
            guard let self, let est = FPGAEstimate.decode(data) else { return }
            let first = self.lock.withLock { () -> Bool in
                self._fpga = (est, Clock.now())
                self._fpgaCount &+= 1
                return self._fpgaCount == 1
            }
            if first { self.engine.recordEvent("fpga_estimates", value: est.backend) }
        }
    }

    // MARK: Lifecycle

    /// Starts telemetry and the command server. Server failure is thrown but telemetry keeps running.
    public func start() throws {
        sender.start()
        try server.start()
    }

    public func stop() {
        if isRecording { _ = stopRun() }
        sender.stop()
        server.stop()
    }

    // MARK: State

    public var settings: RuntimeSettings { lock.withLock { _settings } }
    public var torchLevel: Double { lock.withLock { _torchLevel } }
    public var dashboardPeer: String? { lock.withLock { _dashboardPeer } }
    public var isRecording: Bool { recorder.isRecording }
    public var currentRunId: String? { recorder.currentRunId }
    public var lastRunId: String? { lock.withLock { _lastRunId } }
    /// Distance (m) of the most recently stopped run, at the moment it stopped.
    public var lastRunDistance: Double? { lock.withLock { _lastRunDistance } }

    /// The newest estimate the dashboard computed on the FPGA, if it arrived within `maxAge`
    /// seconds (nil: no dashboard is running the filter on the FPGA, or the link dropped, so
    /// the phone's own filter is what counts).
    public func fpgaEstimate(maxAge: Double = 0.5) -> FPGAEstimate? {
        lock.withLock { () -> FPGAEstimate? in
            guard let f = _fpga, Clock.now() - f.receivedAt <= maxAge else { return nil }
            return f.estimate
        }
    }

    /// Number of FPGA estimates received since launch.
    public var fpgaEstimatesReceived: UInt64 { lock.withLock { _fpgaCount } }

    /// Apply new settings (pushes to engine/sender). Filter changes reset the filter.
    public func apply(_ new: RuntimeSettings) {
        let old = lock.withLock { () -> RuntimeSettings in
            let o = _settings
            _settings = new
            return o
        }
        engine.updateSettings(new.engineSettings)
        if old.filterName != new.filterName || old.filterConfig != new.filterConfig {
            engine.setFilter(name: new.filterName, config: new.filterConfig)
        }
        if old.useJSON != new.useJSON { sender.useJSON = new.useJSON }
        if old.dashboardHost != new.dashboardHost || old.telemetryPort != new.telemetryPort {
            sender.setDestination(host: new.dashboardHost, port: new.telemetryPort)
        }
        onStateChange?()
    }

    // MARK: Actions

    @discardableResult
    public func startRun(label: String? = nil) throws -> String {
        let s = settings
        let runId = try recorder.startRun(label: label)
        engine.setRecording(true)
        engine.beginRun()
        let t0 = Clock.now()
        let filterName = FilterRegistry.make(name: s.filterName) != nil ? s.filterName : engine.currentFilterName
        let meta = RunMeta(
            runId: runId, label: RunRecorder.sanitizeLabel(label), appVersion: hooks.appVersion,
            device: hooks.deviceName, filterName: filterName, mountHeightM: s.manualHeight,
            torchLevel: torchLevel, scaleFactor: s.scaleFactor, focalPx: hooks.focalPx(),
            mountConfig: mountConfig(s), startT: t0, endT: nil,
            batteryStart: hooks.battery(), batteryEnd: nil,
            q: s.filterConfig.qDict, r: s.filterConfig.rDict)
        lock.withLock { _meta = meta; _lastRunId = runId }
        recorder.writeMeta(meta)   // provisional, survives a crash
        engine.recordEvent(CSVSchema.Event.start, value: label ?? "")
        onStateChange?()
        return runId
    }

    /// Stops the active run, writes meta.json. Returns the run id.
    @discardableResult
    public func stopRun() -> String? {
        guard let runId = recorder.currentRunId else { return nil }
        engine.drain()
        engine.recordEvent(CSVSchema.Event.stop)
        engine.drain()
        let dist = engine.snapshot().distance
        engine.setRecording(false)
        var meta = lock.withLock { _meta }
        meta?.endT = Clock.now()
        meta?.batteryEnd = hooks.battery()
        meta?.torchLevel = torchLevel
        recorder.stopRun(meta: meta)
        lock.withLock { _meta = nil; _lastRunDistance = dist }
        onStateChange?()
        return runId
    }

    public func mark(label: String? = nil) {
        engine.recordEvent(CSVSchema.Event.mark, value: label ?? "")
    }

    public func calibrate() {
        engine.calibrate()
    }

    public func resetDistance() {
        engine.resetDistance()
    }

    /// Zero: distance + net forward to 0 and re-estimate IMU bias (hold still ~1 s).
    public func zero() {
        engine.resetDistance()
        engine.calibrate()
        engine.recordEvent(CSVSchema.Event.mark, value: "zero")
    }

    /// Sets the torch (0 = off … 1). Returns the applied level.
    @discardableResult
    public func setTorch(level: Double) -> Double {
        let req = max(0, min(1, level.isFinite ? level : 0))
        let applied = hooks.setTorch(req)
        lock.withLock {
            _torchLevel = applied
            _settings.torchLevel = req
        }
        engine.setTorchOn(applied > 0)
        engine.recordEvent(CSVSchema.Event.torch, value: CSVRow.real(applied))
        onStateChange?()
        return applied
    }

    // MARK: CommandHandling (called on the command server queue)

    public func handle(_ command: Command) -> CommandReply {
        switch command {
        case .ping:
            return .success(command.name)
        case .startRun(let label):
            do {
                let id = try startRun(label: label)
                return .success(command.name, run: id)
            } catch {
                return .failure(command.name, "\(error)")
            }
        case .stopRun:
            guard let id = stopRun() else { return .failure(command.name, "not recording") }
            return .success(command.name, run: id, distance: lastRunDistance)
        case .mark(let label):
            mark(label: label)
            return .success(command.name)
        case .calibrate:
            calibrate()
            return .success(command.name)
        case .setTorch(let level):
            setTorch(level: level)
            return .success(command.name)
        case .resetDistance:
            resetDistance()
            return .success(command.name)
        case .zero:
            zero()
            return .success(command.name)
        case .measureHeight:
            guard let measure = hooks.measureHeight else { return .failure(command.name, "no LiDAR on this host") }
            let sem = DispatchSemaphore(value: 0)
            let result = LockedResult()
            measure { h, msg in result.set(h, msg); sem.signal() }
            guard sem.wait(timeout: .now() + 8) == .success else {
                return .failure(command.name, "LiDAR measurement timed out")
            }
            let (h, msg) = result.get()
            guard let h else { return CommandReply(ok: false, cmd: command.name, error: msg, message: msg) }
            return CommandReply(ok: true, cmd: command.name, h: h, message: msg)
        }
    }

    // MARK: Telemetry

    /// Builds the outgoing frame from the latest engine snapshot (seq set by the sender).
    public func makeFrame() -> TelemetryFrame {
        let s = engine.snapshot()
        let batt = hooks.battery().map { UInt8(clamping: max(0, min(100, $0))) } ?? 0xFF
        let torch = UInt8(clamping: Int((torchLevel * 100).rounded()))
        return TelemetryFrame(
            seq: 0, t: s.t, vx: Float(s.vx), vy: Float(s.vy),
            sigmaVx: Float(s.sigmaVx), sigmaVy: Float(s.sigmaVy),
            distance: Float(s.distance), flowQuality: Float(s.flowQuality), h: Float(s.h),
            status: s.status, battery: batt, torch: torch,
            ax: Float(s.imuAx), ay: Float(s.imuAy), gz: Float(s.imuGz),
            flowVx: Float(s.flowVx), flowVy: Float(s.flowVy), netForward: Float(s.netForward))
    }

    private final class LockedResult: @unchecked Sendable {
        private let lock = NSLock()
        private var h: Double?
        private var msg = ""
        func set(_ h: Double?, _ m: String) { lock.withLock { self.h = h; msg = m } }
        func get() -> (Double?, String) { lock.withLock { (h, msg) } }
    }

    private func mountConfig(_ s: RuntimeSettings) -> JSONValue {
        .object([
            "flow": (try? JSONValue.from(s.flowMapping)) ?? .null,
            "imu": (try? JSONValue.from(s.imuMapping)) ?? .null,
            "use_lidar": .bool(s.useLiDAR),
            "flow_quality_threshold": .number(max(s.flowQualityThreshold, s.filterConfig.psrMin)),
        ])
    }
}
