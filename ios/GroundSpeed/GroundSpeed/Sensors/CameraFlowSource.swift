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
    }

    let session = AVCaptureSession()
    private let sessionQueue = DispatchQueue(label: "gsk.camera.session")
    private let videoQueue = DispatchQueue(label: "gsk.camera.frames", qos: .userInteractive)
    private let output = AVCaptureVideoDataOutput()
    private var device: AVCaptureDevice?
    private var configured = false

    // Frame-queue state.
    private let correlator = PhaseCorrelator(n: 128, downsample: 2)
    private var lastPTS: Double?

    // Cross-thread state.
    private let lock = NSLock()
    private var _status = Status()
    private var _mapping = MountMapping()
    private var _scaleFactor = 1.0
    private var _fovDegrees: Double = 0
    private var _torchLevel: Double = 0

    var onFlow: Sink?
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
                } catch {
                    self.updateStatus { $0.error = "\(error)"; $0.running = false }
                    return
                }
            }
            guard !self.session.isRunning else { return }
            self.correlator.reset()
            self.lastPTS = nil
            self.session.startRunning()
            self.updateStatus { $0.running = self.session.isRunning }
            self.applyTorchOnSessionQueue(torch)
            self.unlockFocusAndExposure()
            // Let AF/AE settle with the torch on, then lock for flow.
            weak let me = self
            self.sessionQueue.asyncAfter(deadline: .now() + 1.5) {
                me?.lockFocusAndExposure()
            }
        }
    }

    func stop() {
        sessionQueue.async {
            if self.session.isRunning { self.session.stopRunning() }
            self.updateStatus { $0.running = false }
            self.lock.withLock { self._torchLevel = 0 }
        }
    }

    /// Stops the session and blocks until it is stopped (used before the LiDAR session runs).
    func stopAndWait() {
        sessionQueue.sync {
            if self.session.isRunning { self.session.stopRunning() }
            self.updateStatus { $0.running = false }
            self.lock.withLock { self._torchLevel = 0 }
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

        guard let chosen = Self.chooseFormat(dev) else { throw ConfigError.noFormat }
        let format = chosen.0
        let fps = chosen.1
        try dev.lockForConfiguration()
        dev.activeFormat = format
        let frameDuration = CMTime(value: 1, timescale: CMTimeScale(fps.rounded()))
        dev.activeVideoMinFrameDuration = frameDuration
        dev.activeVideoMaxFrameDuration = frameDuration
        if dev.isLowLightBoostSupported { dev.automaticallyEnablesLowLightBoostWhenAvailable = false }
        if dev.isAutoFocusRangeRestrictionSupported { dev.autoFocusRangeRestriction = .near }
        if dev.isSmoothAutoFocusSupported { dev.isSmoothAutoFocusEnabled = false }
        dev.unlockForConfiguration()

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

    /// f_px = (width/2) / tan(hfov/2), in pixels of an image `width` wide.
    static func focalPx(width: Double, fovDegrees: Double) -> Double {
        guard fovDegrees > 0 else { return 0 }
        return (width / 2) / tan(fovDegrees * .pi / 360)
    }

    /// Prefers 120 fps near 1280×720, else the highest fps ≤ 240 near 720p, else 60/30.
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
            // Target 120; accept up to 240 when 120 isn't offered, otherwise the highest we get.
            let fps: Double
            if maxFps >= 120 { fps = 120 } else { fps = min(maxFps, 240) }
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

    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer,
                       from connection: AVCaptureConnection) {
        guard let pb = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        // Capture-session PTS is on the host clock (mach time, seconds) = our Clock domain.
        let pts = Clock.fromHostClockSeconds(CMTimeGetSeconds(CMSampleBufferGetPresentationTimeStamp(sampleBuffer)))
        let t0 = Clock.now()

        CVPixelBufferLockBaseAddress(pb, .readOnly)
        let shift: FlowShift?
        let width = CVPixelBufferGetWidthOfPlane(pb, 0)
        if let base = CVPixelBufferGetBaseAddressOfPlane(pb, 0) {
            shift = correlator.ingest(lumaBase: UnsafeRawPointer(base), width: width,
                                      height: CVPixelBufferGetHeightOfPlane(pb, 0),
                                      bytesPerRow: CVPixelBufferGetBytesPerRowOfPlane(pb, 0))
        } else {
            shift = nil
        }
        CVPixelBufferUnlockBaseAddress(pb, .readOnly)

        let prev = lastPTS
        lastPTS = pts
        guard let shift, let prev else { return }
        let dt = pts - prev
        guard dt > 0, dt < 0.1 else { return }

        let (fov, mapping, scale) = lock.withLock { (_fovDegrees, _mapping, _scaleFactor) }
        let fpx = Self.focalPx(width: Double(width), fovDegrees: fov)
        guard fpx > 0 else { return }
        let h = heightProvider()
        let converter = FlowConverter(focalPx: fpx, downsample: Double(correlator.downsample), scaleFactor: scale)
        let vCam = converter.velocity(shift, dt: dt, h: h, gravityCam: gravityProvider())
        let vVeh = mapping.apply(vCam)
        let ms = (Clock.now() - t0) * 1000
        updateStatus {
            $0.camVx = vCam.vx; $0.camVy = vCam.vy; $0.lastPSR = shift.psr
            $0.processingMs = $0.processingMs == 0 ? ms : 0.95 * $0.processingMs + 0.05 * ms
            $0.focalPx = fpx
        }
        // Velocity is the mean over [prev, pts]: timestamp it at the midpoint.
        onFlow?(pts - dt / 2, vVeh.vx, vVeh.vy, shift.psr, h)
    }
}
