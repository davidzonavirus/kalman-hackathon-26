import Foundation
import KalmanCore
import OpticalFlow
import PhoneRuntime
import SpeedProtocol
import SwiftUI
import UIKit

/// Tiny lock-protected box for values read from non-main threads (runtime hooks).
final class LockedBox<T>: @unchecked Sendable {
    private let lock = NSLock()
    private var value: T
    init(_ v: T) { value = v }
    func get() -> T { lock.withLock { value } }
    func set(_ v: T) { lock.withLock { value = v } }
}

/// App state + wiring of the iOS sensor sources into the platform-agnostic `GroundSpeedRuntime`.
@MainActor
final class AppModel: ObservableObject {
    static let settingsKey = "gsk.runtimeSettings.v1"

    // MARK: Published UI state
    @Published private(set) var snap = FusionSnapshot()
    @Published private(set) var settings: RuntimeSettings
    @Published private(set) var isRecording = false
    @Published private(set) var runId: String?
    @Published private(set) var runStart: Date?
    @Published private(set) var sensorsOn = false
    @Published private(set) var dashboardClients = 0
    @Published private(set) var dashboardPeer: String?
    @Published private(set) var telemetryTarget = ""
    @Published private(set) var framesSent: UInt64 = 0
    @Published private(set) var serverState = ""
    @Published private(set) var camera = CameraFlowSource.Status()
    @Published private(set) var gnssHorizontalAcc: Double = -1
    @Published private(set) var battery: Int?
    @Published private(set) var torchLevel: Double = 0
    @Published private(set) var lidarMessage = DepthSource.isAvailable ? "LiDAR ready — tap MEASURE H" : "No LiDAR — manual h"
    @Published private(set) var measuringHeight = false
    @Published private(set) var marks = 0
    @Published private(set) var learnMessage: String?
    @Published var runs: [RunSummary] = []
    @Published var errorMessage: String?

    // MARK: Runtime + sources
    let runtime: GroundSpeedRuntime
    private let motion = MotionSource()
    private let cameraSource = CameraFlowSource()
    private let location = LocationSource()
    private let depth = DepthSource()
    private let batteryBox = LockedBox<Int?>(nil)
    private var refreshTask: Task<Void, Never>?
    private var started = false

    // Mount-learning accumulator (camera axes, during a forward push).
    private var learnUntil: Date?
    private var learnSum = (vx: 0.0, vy: 0.0, n: 0)

    init() {
        let s = RuntimeSettings.decode(UserDefaults.standard.data(forKey: Self.settingsKey))
        settings = s
        runtime = GroundSpeedRuntime(settings: s, runsDirectory: RunRecorder.defaultRunsDirectory(),
                                     bonjourName: UIDevice.current.name)
        var hooks = GroundSpeedRuntime.Hooks()
        let box = batteryBox
        let cam = cameraSource
        hooks.battery = { box.get() }
        hooks.setTorch = { level in cam.setTorch(level: level) }
        hooks.focalPx = { cam.status.focalPx }
        hooks.deviceName = "\(UIDevice.current.model) iOS \(UIDevice.current.systemVersion)"
        let version = Bundle.main.object(forInfoDictionaryKey: "CFBundleShortVersionString") as? String ?? "0.1"
        let build = Bundle.main.object(forInfoDictionaryKey: "CFBundleVersion") as? String ?? "1"
        hooks.appVersion = "\(version) (\(build))"
        runtime.hooks = hooks

        let engine = runtime.engine
        motion.mapping = s.imuMapping
        motion.onSample = { t, ax, ay, az, gx, gy, gz in
            engine.ingestIMU(t: t, ax: ax, ay: ay, az: az, gx: gx, gy: gy, gz: gz)
        }
        cameraSource.mapping = s.flowMapping
        cameraSource.scaleFactor = s.scaleFactor
        cameraSource.heightProvider = { engine.currentHeight }
        let mot = motion
        cameraSource.gravityProvider = { mot.gravityDevice }
        cameraSource.onFlow = { t, vx, vy, q, h in
            engine.ingestFlow(t: t, vx: vx, vy: vy, quality: q, h: h)
        }
        location.onSample = { t, speed, acc, course in
            engine.ingestGNSS(t: t, speed: speed, speedAcc: acc, course: course)
        }
        depth.onSample = { t, h in engine.ingestDepth(t: t, h: h) }
        runtime.onStateChange = { [weak self] in
            guard let model = self else { return }
            Task { @MainActor in model.refreshState() }
        }
    }

    // MARK: Lifecycle

    func startIfNeeded() {
        guard !started else { return }
        started = true
        UIDevice.current.isBatteryMonitoringEnabled = true
        do {
            try runtime.start()
        } catch {
            errorMessage = "Command server: \(error.localizedDescription)"
        }
        startSensors()
        refreshRuns()
        refreshTask = Task { @MainActor [weak self] in
            while !Task.isCancelled {
                self?.tick()
                try? await Task.sleep(nanoseconds: 66_000_000)   // ~15 Hz UI
            }
        }
    }

    func handleScenePhase(_ phase: ScenePhase) {
        switch phase {
        case .active:
            UIApplication.shared.isIdleTimerDisabled = true
            if started && !sensorsOn { startSensors() }
        case .background:
            // The camera stops in the background; close the run cleanly.
            if isRecording { toggleRecording() }
            stopSensors()
        default:
            break
        }
    }

    func startSensors() {
        if !motion.start() { errorMessage = "Device motion unavailable" }
        location.start()
        let level = settings.torchLevel
        let cam = cameraSource
        let rt = runtime
        weak let me = self
        CameraFlowSource.requestAccess { granted in
            Task { @MainActor in
                guard let self = me else { return }
                guard granted else {
                    self.errorMessage = "Camera access denied — enable it in Settings › GroundSpeed"
                    return
                }
                cam.start(torch: level)
                DispatchQueue.global(qos: .userInitiated).async { rt.setTorch(level: level) }
            }
        }
        sensorsOn = true
    }

    func stopSensors() {
        motion.stop()
        location.stop()
        cameraSource.stop()
        sensorsOn = false
    }

    func toggleSensors() {
        sensorsOn ? stopSensors() : startSensors()
    }

    // MARK: Actions

    func toggleRecording() {
        if isRecording {
            runtime.stopRun()
            refreshRuns()
        } else {
            do {
                try runtime.startRun(label: nil)
                marks = 0
            } catch {
                errorMessage = "Could not start run: \(error.localizedDescription)"
            }
        }
        refreshState()
    }

    func mark() {
        marks += 1
        runtime.mark(label: "m\(marks)")
    }

    func calibrate() { runtime.calibrate() }

    func resetDistance() { runtime.resetDistance() }

    func setTorch(_ level: Double) {
        var s = settings
        s.torchLevel = level
        settings = s
        save(s)
        let rt = runtime
        DispatchQueue.global(qos: .userInitiated).async { rt.setTorch(level: level) }
    }

    func apply(_ new: RuntimeSettings) {
        let old = settings
        settings = new
        save(new)
        runtime.apply(new)
        motion.mapping = new.imuMapping
        cameraSource.mapping = new.flowMapping
        cameraSource.scaleFactor = new.scaleFactor
        if abs(old.manualHeight - new.manualHeight) > 0.005 { cameraSource.relock() }
        if old.torchLevel != new.torchLevel { setTorch(new.torchLevel) }
    }

    private func save(_ s: RuntimeSettings) {
        UserDefaults.standard.set(s.encoded(), forKey: Self.settingsKey)
    }

    /// Pauses the flow camera, measures height with LiDAR, stores it as the mount height.
    func measureHeight() {
        guard DepthSource.isAvailable else {
            lidarMessage = "No LiDAR on this device — set h manually"
            return
        }
        guard !measuringHeight else { return }
        measuringHeight = true
        lidarMessage = "Measuring… keep the cart still"
        let cam = cameraSource
        let dep = depth
        weak let me = self
        DispatchQueue.global(qos: .userInitiated).async {
            cam.stopAndWait()
            dep.measure { h, message in
                Task { @MainActor in me?.finishHeight(h, message) }
            }
        }
    }

    private func finishHeight(_ h: Double?, _ message: String) {
        measuringHeight = false
        lidarMessage = message + (h == nil ? " — using manual h" : "")
        if let h {
            var s = settings
            s.manualHeight = (h * 1000).rounded() / 1000
            apply(s)
            runtime.engine.recordEvent("lidar_height", value: CSVRow.real(h))
        }
        if sensorsOn {
            cameraSource.start(torch: settings.torchLevel)
            let rt = runtime
            let level = settings.torchLevel
            DispatchQueue.global(qos: .userInitiated).async { rt.setTorch(level: level) }
        }
    }

    /// Learn the flow mount mapping from a 3 s straight forward push.
    func learnMountFromPush() {
        learnSum = (0, 0, 0)
        learnUntil = Date().addingTimeInterval(3)
        learnMessage = "Push the cart straight FORWARD now…"
    }

    // MARK: Runs

    func refreshRuns() {
        runs = runtime.recorder.listRuns()
    }

    func deleteRun(_ run: RunSummary) {
        do { try runtime.recorder.deleteRun(run.runId) } catch { errorMessage = "\(error.localizedDescription)" }
        refreshRuns()
    }

    /// Zips a run folder off the main thread.
    func exportRun(_ run: RunSummary) async -> URL? {
        let folder = run.url
        return await Task.detached(priority: .userInitiated) { () -> URL? in
            try? RunRecorder.zipRunFolder(folder)
        }.value
    }

    // MARK: Refresh

    private func tick() {
        snap = runtime.engine.snapshot()
        camera = cameraSource.status
        gnssHorizontalAcc = location.horizontalAccuracy
        let lvl = UIDevice.current.batteryLevel
        let b: Int? = lvl < 0 ? nil : Int((lvl * 100).rounded())
        if b != battery { battery = b; batteryBox.set(b) }
        framesSent = runtime.sender.framesSent
        torchLevel = runtime.torchLevel
        refreshState()

        if let until = learnUntil {
            let c = camera
            if (c.camVx * c.camVx + c.camVy * c.camVy).squareRoot() > 0.15 {
                learnSum.vx += c.camVx; learnSum.vy += c.camVy; learnSum.n += 1
            }
            if Date() >= until {
                learnUntil = nil
                if learnSum.n >= 10 {
                    let m = MountMapping.fromForwardPush(vxCam: learnSum.vx / Double(learnSum.n),
                                                         vyCam: learnSum.vy / Double(learnSum.n))
                    var s = settings
                    s.flowMapping = m
                    apply(s)
                    learnMessage = "Flow mapping set: swap \(m.swapXY ? "on" : "off"), flip X \(m.flipX ? "on" : "off"), flip Y \(m.flipY ? "on" : "off")"
                } else {
                    learnMessage = "Not enough motion — push faster (> 0.15 m/s) and retry"
                }
            }
        }
    }

    private func refreshState() {
        let rec = runtime.isRecording
        if rec != isRecording {
            isRecording = rec
            runStart = rec ? Date() : nil
            if !rec { refreshRuns() }
        }
        let id = runtime.currentRunId ?? runtime.lastRunId
        if id != runId { runId = id }
        dashboardClients = runtime.server.clientCount
        dashboardPeer = runtime.dashboardPeer
        telemetryTarget = "\(runtime.sender.host):\(runtime.sender.port)"
        serverState = runtime.server.state
    }
}
