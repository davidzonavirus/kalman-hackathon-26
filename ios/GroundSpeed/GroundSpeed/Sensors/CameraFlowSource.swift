import AVFoundation
import CoreMedia
import CoreVideo
import Foundation
import OpticalFlow
import PhoneRuntime

/// Back wide camera looking at the floor → phase-correlation optical flow → vehicle-frame
/// ground velocity.
///
/// Format: ~1280×720 at 120 fps if available (else the highest fps ≤ 240 near 720p, else
/// 60/30), 420f. Video stabilisation off. Torch via `setTorchModeOn(level:)`. After a short
/// settle with continuous AF/AE (torch on), focus is locked at the current lens position and
/// exposure is switched to custom ≤ 1/1000 s with ISO raised to keep brightness.
final class CameraFlowSource: NSObject, AVCaptureVideoDataOutputSampleBufferDelegate, @unchecked Sendable {
    typealias Sink = (_ t: Double, _ vx: Double, _ vy: Double, _ quality: Double, _ h: Double) -> Void

    struct Status: Sendable {
        var running = false
        var formatDescription = "—"
        var fps: Double = 0
        var focalPx: Double = 0
        var focusLocked = false
        var exposureLocked = false
        var torchAvailable = false
        var error: String?
        /// Last camera-axes velocity (before mount mapping), for the orientation check.
        var camVx: Double = 0
        var camVy: Double = 0
        var lastPSR: Double = 0
        var processingMs: Double = 0
        var focalFromIntrinsics = false
        var exposureS: Double = 0
        var iso: Double = 0
        /// Frames delivered vs dropped (late) since start.
        var framesIn: Int = 0
        var framesDropped: Int = 0
        /// Session restarts by the stall watchdog / runtime-error handler.
        var restarts: Int = 0
    }

    /// Requested capture rate; falls back to the highest rate the camera offers below it.
    static let targetFps: Double = 240

    let session = AVCaptureSession()
    private let sessionQueue = DispatchQueue(label: "gsk.camera.session")
    private let videoQueue = DispatchQueue(label: "gsk.camera.frames", qos: .userInteractive)
    private let output = AVCaptureVideoDataOutput()
    private var device: AVCaptureDevice?
    private var configured = false
    // Session-queue state.
    private var wantRunning = false
    private var wantTorch = 0.0
    private var watchdog: DispatchSourceTimer?
    private var chosenFormat: (AVCaptureDevice.Format, Double)?
    private var pressureObservation: NSKeyValueObservation?
    /// Frame rate used while the camera reports serious/critical system pressure (heat).
    static let hotFps: Double = 120
    private var throttled = false
    static let stallTimeout = 1.5

    // Frame-queue state.
    private let correlator = PhaseCorrelator(n: 128, downsample: 2)
    private var lastPTS: Double?
    /// Left, right, top, bottom patches for `HeightTracker` (ride-height changes between
    /// LiDAR fixes). Re-anchored to `heightProvider()` on session start and when it changes.
    private let sidePatches = (0..<4).map { _ in PhaseCorrelator(n: 128, downsample: 2) }
    private var heightTracker: HeightTracker?
    /// Off: on the car (2026-10-04, two runs) the tracker drifted to its lower clamp at
    /// 1.5–2.5 m/s and every speed after that read 0.67× GPS. Height = LiDAR/manual.
    static let tracksHeight = false
    /// Motion prediction (full-res px per frame) from the last confident shift. Without it the
    /// correlator wraps at ±64 downsampled px per frame (≈ 6 m/s at h 0.17 m, 240 fps).
    private var lastGood: (dx: Double, dy: Double) = (0, 0)
    private var lostFrames = 0
    static let trackPSR = 10.0
    /// A shift this confident is accepted even if it jumps away from `lastGood`.
    static let relockPSR = 30.0
    /// Lost frames spent still predicting `lastGood` (single-frame dips from vibration).
    static let coastFrames = 24
    /// Then tried in turn, as multiples of `lastGood`.
    static let reacquire: [Double] = [0.5, 1.5, 0.75, 1.25, 0, 1]
    /// Allowed change from `lastGood` (full-res px): base + per lost frame. A weak peak at a
    /// very different motion is the sensor's fixed pattern or a wrap, not the car (seen:
    /// 4.7 m/s → 0 in one frame at PSR 13, after which tracking never recovered).
    static let jumpBase = 8.0, jumpPerFrame = 2.0
    /// Exactly-zero motion (< `zeroShift` full-res px) while `lastGood` is above `movingShift`
    /// needs `relockPSR`: the fixed pattern peaks at zero with PSR 10–25, and a car cannot
    /// stop within a lost-track interval (seen: locked at 0 while driving 5–6 m/s for 30 s).
    static let zeroShift = 1.5, movingShift = 8.0
    /// Below this |lastGood| (full-res px, ¼ of the ±128 px window) the plain un-predicted
    /// correlation is used: the path the cart runs were calibrated on (±0.4 %).
    static let predictFrom = 32.0

    // Cross-thread state.
    private let lock = NSLock()
    private var _status = Status()
    private var _mapping = MountMapping()
    private var _scaleFactor = 1.0
    private var _fovDegrees: Double = 0
    private var _torchLevel: Double = 0
    private var _lastFlowWall: Double = 0
    /// Every delivered frame / every frame that produced a flow sample (diagnostics).
    private var _framesSeen = 0
    private var _flowsOut = 0

    var onFlow: Sink?
    /// Session lifecycle notes (restart, interruption, runtime error), any thread.
    var onEvent: (@Sendable (_ event: String, _ value: String) -> Void)?
    /// Camera height (m) to use for each frame (LiDAR-measured or manual).
    var heightProvider: () -> Double = { 0.30 }
    /// Gravity in device coordinates (unit g) for tilt correction; nil = assume straight down.
    var gravityProvider: () -> SIMD3<Double>? = { nil }
    /// Exposure ceiling (s). Short exposures kill motion blur at 1 m/s.
    var maxExposure: Double = 1.0 / 1000.0

    var status: Status { lock.withLock { _status } }

    var mapping: MountMapping {
        get { lock.withLock { _mapping } }
        set { lock.withLock { _mapping = newValue } }
    }

    var scaleFactor: Double {
        get { lock.withLock { _scaleFactor } }
        set { lock.withLock { _scaleFactor = newValue } }
    }

    var torchLevel: Double { lock.withLock { _torchLevel } }

    private func updateStatus(_ body: (inout Status) -> Void) {
        lock.withLock { body(&_status) }
    }

    // MARK: - Lifecycle

    static func requestAccess(_ completion: @escaping @Sendable (Bool) -> Void) {
        switch AVCaptureDevice.authorizationStatus(for: .video) {
        case .authorized: completion(true)
        case .notDetermined: AVCaptureDevice.requestAccess(for: .video, completionHandler: completion)
        default: completion(false)
        }
    }

    /// Configures (once) and starts the session. `torch` is applied once running.
    func start(torch: Double) {
        sessionQueue.async {
            if !self.configured {
                do {
                    try self.configure()
                    self.configured = true
                    self.installObservers()
                } catch {
                    self.updateStatus { $0.error = "\(error)"; $0.running = false }
                    return
                }
            }
            self.wantRunning = true
            self.wantTorch = torch
            self.startWatchdog()
            guard !self.session.isRunning else { return }
            self.startSessionOnQueue()
        }
    }

    func stop() {
        sessionQueue.async { self.stopSessionOnQueue() }
    }

    /// Stops the session and blocks until it is stopped (used before the LiDAR session runs).
    func stopAndWait() {
        sessionQueue.sync { self.stopSessionOnQueue() }
    }

    private func startSessionOnQueue() {
        videoQueue.sync {
            correlator.reset()
            sidePatches.forEach { $0.reset() }
            heightTracker = nil
            lastPTS = nil
            lastGood = (0, 0)
            lostFrames = 0
        }
        // Start at a low rate, then raise it once running. Starting straight at 240 fps right
        // after the LiDAR session crashes cameracaptured (abort in -[BWFigVideoCaptureStream
        // setMinimumFrameRate:] while it reuses the depth session's 30 fps stream).
        let target = chosenFormat
        if let dev = device, let (format, fps) = target {
            let range = format.videoSupportedFrameRateRanges.first { $0.minFrameRate <= fps && fps <= $0.maxFrameRate }
            let startFps = min(fps, max(30, range?.minFrameRate ?? 30))
            do { try applyFormat(dev, format, fps: startFps) } catch { updateStatus { $0.error = "format: \(error)" } }
        }
        let (seen0, out0) = lock.withLock { () -> (Int, Int) in
            _lastFlowWall = Clock.now()
            return (_framesSeen, _flowsOut)
        }
        session.startRunning()
        raiseFrameRateIfRunning()
        updateStatus { $0.running = self.session.isRunning }
        applyTorchOnSessionQueue(wantTorch)
        unlockFocusAndExposure()
        // Let AF/AE settle with the torch on, then lock for flow.
        weak let me = self
        sessionQueue.asyncAfter(deadline: .now() + 1.5) {
            me?.lockFocusAndExposure()
        }
        sessionQueue.asyncAfter(deadline: .now() + 1.0) {
            guard let me, let dev = me.device else { return }
            let (seen, out) = me.lock.withLock { (me._framesSeen - seen0, me._flowsOut - out0) }
            let d = CMVideoFormatDescriptionGetDimensions(dev.activeFormat.formatDescription)
            let fps = 1 / max(1e-6, CMTimeGetSeconds(dev.activeVideoMinFrameDuration))
            me.onEvent?("camera_started", String(format: "%dx%d @ %.0f fps; 1 s: frames %d flows %d; running %@ thermal %d",
                                                 d.width, d.height, fps, seen, out,
                                                 me.session.isRunning ? "yes" : "no",
                                                 ProcessInfo.processInfo.thermalState.rawValue))
        }
    }

    private func stopSessionOnQueue() {
        wantRunning = false
        if session.isRunning { session.stopRunning() }
        updateStatus { $0.running = false }
        lock.withLock { _torchLevel = 0 }
    }

    /// Tear down and start again; used when frames stop arriving while we want them.
    private func restartSessionOnQueue(reason: String) {
        guard wantRunning else { return }
        if session.isRunning { session.stopRunning() }
        updateStatus { $0.restarts += 1 }
        onEvent?("camera_restart", reason)
        startSessionOnQueue()
    }

    /// Restarts the session if no flow sample has come out for `stallTimeout` s while it should
    /// run. Seen on device: after a LiDAR measurement the restarted session produced no flow
    /// until the app was relaunched.
    private func startWatchdog() {
        guard watchdog == nil else { return }
        let timer = DispatchSource.makeTimerSource(queue: sessionQueue)
        timer.schedule(deadline: .now() + 1, repeating: 0.5)
        timer.setEventHandler { [weak self] in
            guard let self, self.wantRunning else { return }
            let (age, seen) = self.lock.withLock { (Clock.now() - self._lastFlowWall, self._framesSeen) }
            if age > Self.stallTimeout {
                self.restartSessionOnQueue(reason: String(format: "no flow for %.1f s (frames seen %d running %@ interrupted %@)",
                                                          age, seen, self.session.isRunning ? "yes" : "no",
                                                          self.session.isInterrupted ? "yes" : "no"))
            }
        }
        timer.resume()
        watchdog = timer
    }

    private func runFps(_ fps: Double) -> Double { throttled ? min(fps, Self.hotFps) : fps }

    private func raiseFrameRateIfRunning() {
        guard wantRunning, session.isRunning, let dev = device, let fps = chosenFormat?.1 else { return }
        let target = runFps(fps)
        guard abs(1 / max(1e-6, CMTimeGetSeconds(dev.activeVideoMinFrameDuration)) - target) > 1 else { return }
        do { try applyFrameRate(dev, fps: target) } catch { updateStatus { $0.error = "fps: \(error)" } }
    }

    /// 240 fps + torch heats the phone (seen: thermal "serious", cameracaptured unresponsive).
    /// Drop to `hotFps` under serious/critical camera pressure, restore when it clears.
    private func handlePressure(_ level: AVCaptureDevice.SystemPressureState.Level) {
        let hot = level == .serious || level == .critical || level == .shutdown
        guard hot != throttled, let dev = device, let fps = chosenFormat?.1 else { return }
        throttled = hot
        let newFps = runFps(fps)
        // Not while stopped: the LiDAR session may own the camera; start applies runFps.
        if session.isRunning {
            do { try applyFrameRate(dev, fps: newFps) } catch { updateStatus { $0.error = "fps: \(error)" } }
        }
        updateStatus { $0.fps = newFps }
        onEvent?("camera_pressure", "\(level.rawValue) → \(Int(newFps)) fps")
    }

    private func installObservers() {
        if let dev = device {
            pressureObservation = dev.observe(\.systemPressureState, options: [.initial, .new]) { [weak self] d, _ in
                let level = d.systemPressureState.level
                self?.sessionQueue.async { self?.handlePressure(level) }
            }
        }
        let nc = NotificationCenter.default
        nc.addObserver(forName: AVCaptureSession.runtimeErrorNotification, object: session, queue: nil) { [weak self] n in
            let err = (n.userInfo?[AVCaptureSessionErrorKey] as? NSError).map { "\($0.domain) \($0.code)" } ?? "?"
            guard let self else { return }
            self.updateStatus { $0.error = "session error \(err)" }
            self.sessionQueue.async { self.restartSessionOnQueue(reason: "runtime error \(err)") }
        }
        nc.addObserver(forName: AVCaptureSession.wasInterruptedNotification, object: session, queue: nil) { [weak self] n in
            let reason = (n.userInfo?[AVCaptureSessionInterruptionReasonKey] as? Int).map(String.init) ?? "?"
            self?.onEvent?("camera_interrupted", "reason \(reason)")
        }
        nc.addObserver(forName: AVCaptureSession.interruptionEndedNotification, object: session, queue: nil) { [weak self] _ in
            self?.onEvent?("camera_interruption_ended", "")
            self?.sessionQueue.async { self?.raiseFrameRateIfRunning() }
        }
        // startRunning() can return before the session runs (app launched with the screen
        // locked, interruption); the 30 fps start rate must still be raised when it does run.
        nc.addObserver(forName: AVCaptureSession.didStartRunningNotification, object: session, queue: nil) { [weak self] _ in
            self?.sessionQueue.async { self?.raiseFrameRateIfRunning() }
        }
    }

    /// Re-run the focus/exposure lock (e.g. after changing mount height).
    func relock() {
        sessionQueue.async {
            self.unlockFocusAndExposure()
            weak let me = self
            self.sessionQueue.asyncAfter(deadline: .now() + 1.5) {
                me?.lockFocusAndExposure()
            }
        }
    }

    /// Thread-safe torch control (0 = off … 1). Returns the level actually applied.
    @discardableResult
    func setTorch(level: Double) -> Double {
        sessionQueue.sync { self.applyTorchOnSessionQueue(level) }
    }

    // MARK: - Configuration (session queue)

    private enum ConfigError: Error, CustomStringConvertible {
        case noCamera, cannotAddInput, cannotAddOutput, noFormat
        var description: String {
            switch self {
            case .noCamera: return "no back wide camera"
            case .cannotAddInput: return "cannot add camera input"
            case .cannotAddOutput: return "cannot add video output"
            case .noFormat: return "no usable camera format"
            }
        }
    }

    private func configure() throws {
        guard let dev = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back) else {
            throw ConfigError.noCamera
        }
        device = dev
        session.beginConfiguration()
        defer { session.commitConfiguration() }

        let input = try AVCaptureDeviceInput(device: dev)
        guard session.canAddInput(input) else { throw ConfigError.cannotAddInput }
        session.addInput(input)

        output.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarFullRange]
        output.alwaysDiscardsLateVideoFrames = true
        output.setSampleBufferDelegate(self, queue: videoQueue)
        guard session.canAddOutput(output) else { throw ConfigError.cannotAddOutput }
        session.addOutput(output)
        if let conn = output.connection(with: .video), conn.isVideoStabilizationSupported {
            conn.preferredVideoStabilizationMode = .off   // stabilisation would corrupt the flow
        }
        if let conn = output.connection(with: .video), conn.isCameraIntrinsicMatrixDeliverySupported {
            conn.isCameraIntrinsicMatrixDeliveryEnabled = true
        }

        guard let chosen = Self.chooseFormat(dev) else { throw ConfigError.noFormat }
        let format = chosen.0
        let fps = chosen.1
        chosenFormat = chosen
        try applyFormat(dev, format, fps: fps)

        let dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
        let fov = Double(format.videoFieldOfView)
        lock.withLock { _fovDegrees = fov }
        let fpx = Self.focalPx(width: Double(dims.width), fovDegrees: fov)
        updateStatus {
            $0.formatDescription = "\(dims.width)×\(dims.height) @ \(Int(fps.rounded())) fps, FOV \(String(format: "%.1f", fov))°"
            $0.fps = fps
            $0.focalPx = fpx
            $0.torchAvailable = dev.hasTorch
            $0.error = nil
        }
    }

    /// The LiDAR session drives the same physical wide camera and leaves its own format and
    /// frame rate on it, so this runs before every session start, not just once.
    private func applyFormat(_ dev: AVCaptureDevice, _ format: AVCaptureDevice.Format, fps: Double) throws {
        try dev.lockForConfiguration()
        dev.activeFormat = format
        let frameDuration = CMTime(value: 1, timescale: CMTimeScale(fps.rounded()))
        dev.activeVideoMinFrameDuration = frameDuration
        dev.activeVideoMaxFrameDuration = frameDuration
        if dev.isLowLightBoostSupported { dev.automaticallyEnablesLowLightBoostWhenAvailable = false }
        if dev.isAutoFocusRangeRestrictionSupported { dev.autoFocusRangeRestriction = .near }
        if dev.isSmoothAutoFocusSupported { dev.isSmoothAutoFocusEnabled = false }
        dev.unlockForConfiguration()
    }

    private func applyFrameRate(_ dev: AVCaptureDevice, fps: Double) throws {
        try dev.lockForConfiguration()
        let frameDuration = CMTime(value: 1, timescale: CMTimeScale(fps.rounded()))
        dev.activeVideoMinFrameDuration = frameDuration
        dev.activeVideoMaxFrameDuration = frameDuration
        dev.unlockForConfiguration()
    }

    /// Centre focal length ÷ the pinhole focal implied by `videoFieldOfView`. The wide lens's
    /// barrel distortion squeezes the edges, so the FOV underestimates the magnification at
    /// the centre crop the correlator uses. Fitted jointly with `DepthSource.heightOffset` on
    /// iPhone 18 Pro Max, 1280×720 @ 240 fps: six 16 ft (4.877 m) pushes at LiDAR h = 0.186 m
    /// and 0.235 m → 1.027, all runs within ±0.4 %. Only applies when the per-frame intrinsic
    /// matrix is unavailable (it is at 240 fps).
    static let fovFocalCorrection = 1.027

    /// f_px = (width/2) / tan(hfov/2) × `fovFocalCorrection`, in pixels of an image `width` wide.
    static func focalPx(width: Double, fovDegrees: Double) -> Double {
        guard fovDegrees > 0 else { return 0 }
        return (width / 2) / tan(fovDegrees * .pi / 360) * fovFocalCorrection
    }

    /// fx (px of this buffer) from the per-frame intrinsic matrix, if delivery is enabled.
    static func intrinsicFx(_ sb: CMSampleBuffer) -> Double? {
        guard let data = CMGetAttachment(sb, key: kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix,
                                         attachmentModeOut: nil) as? Data,
              data.count >= MemoryLayout<matrix_float3x3>.size else { return nil }
        let k = data.withUnsafeBytes { $0.loadUnaligned(as: matrix_float3x3.self) }
        let fx = Double(k.columns.0.x)
        return fx > 0 ? fx : nil
    }

    /// Prefers `targetFps` near 1280×720, else the highest fps below it near 720p.
    static func chooseFormat(_ dev: AVCaptureDevice) -> (AVCaptureDevice.Format, Double)? {
        var best: (format: AVCaptureDevice.Format, fps: Double, key: (Double, Double, Int))?
        for f in dev.formats {
            let dims = CMVideoFormatDescriptionGetDimensions(f.formatDescription)
            let w = Int(dims.width), h = Int(dims.height)
            // The correlator needs a 256×256 centre crop; avoid huge sensor modes.
            guard w >= 640, h >= 480, w <= 1920 else { continue }
            let sub = CMFormatDescriptionGetMediaSubType(f.formatDescription)
            let fullRange = sub == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
            guard fullRange || sub == kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange else { continue }
            let maxFps = f.videoSupportedFrameRateRanges.map(\.maxFrameRate).max() ?? 0
            guard maxFps >= 30 else { continue }
            let fps = min(maxFps, targetFps)
            guard f.videoSupportedFrameRateRanges.contains(where: { $0.minFrameRate <= fps && fps <= $0.maxFrameRate }) else { continue }
            let key = (fps, -abs(Double(w) - 1280), fullRange ? 1 : 0)
            if let b = best {
                if key > b.key { best = (f, fps, key) }
            } else {
                best = (f, fps, key)
            }
        }
        return best.map { ($0.format, $0.fps) }
    }

    @discardableResult
    private func applyTorchOnSessionQueue(_ level: Double) -> Double {
        guard let dev = device, dev.hasTorch else {
            lock.withLock { _torchLevel = 0 }
            return 0
        }
        let req = max(0, min(1, level))
        var applied = 0.0
        do {
            try dev.lockForConfiguration()
        } catch {
            updateStatus { $0.error = "torch: \(error)" }
            return torchLevel
        }
        if req <= 0.001 {
            if dev.isTorchModeSupported(.off) { dev.torchMode = .off }
        } else if dev.isTorchAvailable {
            let lvl = max(0.01, min(Float(req), AVCaptureDevice.maxAvailableTorchLevel))
            do {
                try dev.setTorchModeOn(level: lvl)
                applied = Double(lvl)
            } catch {
                updateStatus { $0.error = "torch: \(error)" }
            }
        }
        dev.unlockForConfiguration()
        lock.withLock { _torchLevel = applied }
        return applied
    }

    private func unlockFocusAndExposure() {
        guard let dev = device else { return }
        do {
            try dev.lockForConfiguration()
            if dev.isFocusModeSupported(.continuousAutoFocus) { dev.focusMode = .continuousAutoFocus }
            if dev.isExposureModeSupported(.continuousAutoExposure) { dev.exposureMode = .continuousAutoExposure }
            dev.unlockForConfiguration()
            updateStatus { $0.focusLocked = false; $0.exposureLocked = false }
        } catch {
            updateStatus { $0.error = "AF/AE: \(error)" }
        }
    }

    private func lockFocusAndExposure() {
        guard let dev = device, session.isRunning else { return }
        do {
            try dev.lockForConfiguration()
            var focusLocked = false
            if dev.isLockingFocusWithCustomLensPositionSupported {
                dev.setFocusModeLocked(lensPosition: dev.lensPosition, completionHandler: nil)
                focusLocked = true
            } else if dev.isFocusModeSupported(.locked) {
                dev.focusMode = .locked
                focusLocked = true
            }
            var exposureLocked = false
            if dev.isExposureModeSupported(.custom) {
                let fmt = dev.activeFormat
                let curDur = CMTimeGetSeconds(dev.exposureDuration)
                let minDur = CMTimeGetSeconds(fmt.minExposureDuration)
                let target = max(minDur, min(maxExposure, curDur > 0 ? curDur : maxExposure))
                // Keep brightness: ISO × duration ≈ const.
                let gain = curDur > 0 ? curDur / target : 1
                let iso = max(fmt.minISO, min(fmt.maxISO, Float(Double(dev.iso) * gain)))
                let dur = CMTime(seconds: target, preferredTimescale: 1_000_000)
                dev.setExposureModeCustom(duration: dur, iso: iso, completionHandler: nil)
                exposureLocked = true
            }
            dev.unlockForConfiguration()
            updateStatus { $0.focusLocked = focusLocked; $0.exposureLocked = exposureLocked }
        } catch {
            updateStatus { $0.error = "lock: \(error)" }
        }
    }

    // MARK: - Frames (video queue)

    func captureOutput(_ output: AVCaptureOutput, didDrop sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {
        updateStatus { $0.framesDropped += 1 }
    }

    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {
        guard let pb = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        lock.withLock { _framesSeen += 1 }
        // Capture-session PTS is on the host clock (mach time, seconds) = our Clock domain.
        let pts = Clock.fromHostClockSeconds(CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(sampleBuffer)))
        let t0 = Clock.now()

        CVPixelBufferLockBaseAddress(pb, .readOnly)
        let shift: FlowShift?
        var sides: [FlowShift?] = []
        let width = CVPixelBufferGetWidthOfPlane(pb, 0)
        let height = CVPixelBufferGetHeightOfPlane(pb, 0)
        let crop = correlator.cropSize
        let offX = min(448, (width - crop) / 2 - 8), offY = min(448, (height - crop) / 2 - 8)
        if let base = CVPixelBufferGetBaseAddressOfPlane(pb, 0) {
            let bpr = CVPixelBufferGetBytesPerRowOfPlane(pb, 0)
            let raw = UnsafeRawPointer(base)
            let fast = hypot(lastGood.dx, lastGood.dy) >= Self.predictFrom
            let m = !fast ? 0 : lostFrames <= Self.coastFrames ? 1 : Self.reacquire[lostFrames % Self.reacquire.count]
            let s = correlator.ingest(lumaBase: raw, width: width, height: height, bytesPerRow: bpr,
                                      predictX: Int((lastGood.dx * m).rounded()),
                                      predictY: Int((lastGood.dy * m).rounded()))
            let ds = Double(correlator.downsample)
            if let s, s.psr >= Self.trackPSR {
                let jump = hypot(s.dx * ds - lastGood.dx, s.dy * ds - lastGood.dy)
                let fakeStop = hypot(s.dx * ds, s.dy * ds) < Self.zeroShift
                    && hypot(lastGood.dx, lastGood.dy) > Self.movingShift
                let confident = s.psr >= Self.relockPSR
                if confident || (!fakeStop && jump <= Self.jumpBase + Self.jumpPerFrame * Double(lostFrames)) {
                    lastGood = (s.dx * ds, s.dy * ds)
                    lostFrames = 0
                    shift = s
                } else {
                    lostFrames += 1
                    shift = FlowShift(dx: s.dx, dy: s.dy, psr: 0)
                }
            } else {
                lostFrames += 1
                shift = s
            }
            if Self.tracksHeight, offX > 0, offY > 0 {
                // Every patch sees every frame so all four compare the same frame pair.
                sides = zip(sidePatches, [(-offX, 0), (offX, 0), (0, -offY), (0, offY)]).map { c, o in
                    c.ingest(lumaBase: raw, width: width, height: height, bytesPerRow: bpr,
                             offsetX: o.0, offsetY: o.1)
                }
            }
        } else {
            shift = nil
        }
        CVPixelBufferUnlockBaseAddress(pb, .readOnly)

        let anchorH = heightProvider()
        if heightTracker?.anchor != anchorH { heightTracker = HeightTracker(height: anchorH) }
        if sides.count == 4, let l = sides[0], let r = sides[1], let t = sides[2], let b = sides[3] {
            let ds = Double(correlator.downsample)
            heightTracker?.update(left: l, right: r, top: t, bottom: b,
                                  offsetX: Double(offX) / ds, offsetY: Double(offY) / ds)
        }

        let prev = lastPTS
        lastPTS = pts
        guard let shift, let prev else { return }
        let dt = pts - prev
        guard dt > 0, dt < 0.1 else { return }

        let (fov, mapping, scale) = lock.withLock { (_fovDegrees, _mapping, _scaleFactor) }
        let kfx = Self.intrinsicFx(sampleBuffer)
        let fpx = kfx ?? Self.focalPx(width: Double(width), fovDegrees: fov)
        guard fpx > 0 else { return }
        let h = heightTracker?.height ?? anchorH
        let converter = FlowConverter(focalPx: fpx, downsample: Double(correlator.downsample), scaleFactor: scale)
        let vCam = converter.velocity(shift, dt: dt, h: h, gravityCam: gravityProvider())
        let vVeh = mapping.apply(vCam)
        let ms = (Clock.now() - t0) * 1000
        updateStatus {
            $0.camVx = vCam.vx; $0.camVy = vCam.vy; $0.lastPSR = shift.psr
            $0.processingMs = $0.processingMs == 0 ? ms : 0.95 * $0.processingMs + 0.05 * ms
            $0.focalPx = fpx
            $0.focalFromIntrinsics = kfx != nil
            $0.framesIn += 1
            if let dev = self.device {
                $0.exposureS = CMTimeGetSeconds(dev.exposureDuration)
                $0.iso = Double(dev.iso)
            }
        }
        lock.withLock { _flowsOut += 1; _lastFlowWall = Clock.now() }
        // Velocity is the mean over [prev, pts]: timestamp it at the midpoint.
        onFlow?(pts - dt / 2, vVeh.vx, vVeh.vy, shift.psr, h)
    }
}
