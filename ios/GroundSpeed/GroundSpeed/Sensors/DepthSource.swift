import AVFoundation
import CoreVideo
import Foundation
import PhoneRuntime
import SpeedProtocol

/// LiDAR height measurement in a SEPARATE, short-lived capture session.
///
/// iOS can't run two capture sessions at once, and the LiDAR depth camera can't deliver
/// 120 fps video, so height is *measured* (cart still, flow session paused): ~1.5 s of depth
/// frames, median of the centre patch of each, median over frames.
///
/// The depth map holds Z-depth along the optical axis, so a tilted phone reads h/cos θ at the
/// centre; the result is multiplied by cos θ (from gravity) to give the VERTICAL height that
/// `FlowConverter` expects. A measurement is rejected (completion gets nil, manual h kept)
/// when the depth is relative rather than metric, the frames disagree, too few pixels are
/// valid, or the height is outside the LiDAR's usable range. Every attempt is logged
/// frame-by-frame to `Documents/lidar/<stamp>.csv` for post-mortem.
final class DepthSource: NSObject, AVCaptureDepthDataOutputDelegate, @unchecked Sendable {
    typealias Sink = (_ t: Double, _ h: Double) -> Void

    var onSample: Sink?
    /// Gravity in device coordinates (unit g); nil = assume the camera looks straight down.
    var gravityProvider: () -> SIMD3<Double>? = { nil }

    /// Usable vertical height range (m). Depth reads stable down to ~0.08 m on an iPhone 18 Pro Max.
    static let minHeight = 0.05
    /// Below this the wide camera can't focus on the floor and the flow window covers so little
    /// floor that fast motion outruns it.
    static let recommendedMinHeight = 0.20
    static let maxHeight = 2.0
    /// Max frame-to-frame spread (p90 − p10 of per-frame heights) relative to the median.
    static let maxRelativeSpread = 0.08
    /// Added to the LiDAR height to get the height of the flow camera's projection centre.
    /// Fitted with `CameraFlowSource.fovFocalCorrection` from 16 ft pushes at two heights:
    /// −7.9 mm (90 % interval −11…−5 mm). A pure focal fit leaves a height-dependent error
    /// (+0.5 % at 0.19 m, −0.5 % at 0.24 m, ≈ −2 % at 0.5 m). The per-frame log keeps raw LiDAR.
    static let heightOffset = -0.0079
    // Validated mount heights: ±1 % at 0.19–0.25 m. At 0.79 m the floor moves ~1 px/frame
    // at walking pace and distance read 15–17 % low.

    private let queue = DispatchQueue(label: "gsk.depth", qos: .userInitiated)
    private var session: AVCaptureSession?
    private var depthOutput = AVCaptureDepthDataOutput()
    private var samples: [Double] = []
    private var completion: ((Double?, String) -> Void)?
    private var deadline = 0.0
    private var log: FileHandle?
    private var formatNote = ""

    static var isAvailable: Bool {
        AVCaptureDevice.default(.builtInLiDARDepthCamera, for: .video, position: .back) != nil
    }

    static func logDirectory() -> URL {
        let docs = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        return docs.appendingPathComponent("lidar", isDirectory: true)
    }

    /// Measures camera height (m). Completion runs on the main queue with (height, message).
    func measure(duration: Double = 1.5, completion: @escaping (Double?, String) -> Void) {
        queue.async {
            self.samples = []
            self.completion = completion
            self.openLog()
            do {
                try self.startSession()
            } catch {
                self.finish(message: "LiDAR unavailable: \(error)")
                return
            }
            self.deadline = Clock.now() + duration
            weak let me = self
            self.queue.asyncAfter(deadline: .now() + duration + 3.0) {
                // Safety net if no (valid) frames arrive.
                guard let me, me.completion != nil else { return }
                me.finish(message: "LiDAR: \(me.samples.count) valid frames — floor too close, dark or shiny?")
            }
        }
    }

    private enum DepthError: Error, CustomStringConvertible {
        case noLiDAR, cannotAdd, noDepthFormat
        var description: String {
            switch self {
            case .noLiDAR: return "no LiDAR depth camera"
            case .cannotAdd: return "cannot add LiDAR input/output"
            case .noDepthFormat: return "no float depth format"
            }
        }
    }

    private func startSession() throws {
        guard let dev = AVCaptureDevice.default(.builtInLiDARDepthCamera, for: .video, position: .back) else {
            throw DepthError.noLiDAR
        }
        let s = AVCaptureSession()
        depthOutput = AVCaptureDepthDataOutput()
        s.beginConfiguration()
        let input = try AVCaptureDeviceInput(device: dev)
        guard s.canAddInput(input) else { s.commitConfiguration(); throw DepthError.cannotAdd }
        s.addInput(input)
        guard s.canAddOutput(depthOutput) else { s.commitConfiguration(); throw DepthError.cannotAdd }
        s.addOutput(depthOutput)
        // Temporal filtering smooths holes but also smears edges; a flat floor is fine either way,
        // and the per-frame log should show the raw sensor.
        depthOutput.isFilteringEnabled = false
        depthOutput.alwaysDiscardsLateDepthData = true
        depthOutput.setDelegate(self, callbackQueue: queue)

        // Prefer a ≤ 30 fps video format near 1920 wide that offers float depth, then its
        // largest float depth format.
        let isFloatDepth: (AVCaptureDevice.Format) -> Bool = { f in
            let st = CMFormatDescriptionGetMediaSubType(f.formatDescription)
            return st == kCVPixelFormatType_DepthFloat32 || st == kCVPixelFormatType_DepthFloat16
        }
        let candidates = dev.formats.filter { $0.supportedDepthDataFormats.contains(where: isFloatDepth) }
        let format = candidates.min(by: {
            abs(Int(CMVideoFormatDescriptionGetDimensions($0.formatDescription).width) - 1920)
                < abs(Int(CMVideoFormatDescriptionGetDimensions($1.formatDescription).width) - 1920)
        })
        guard let format,
              let depthFormat = format.supportedDepthDataFormats.filter(isFloatDepth).max(by: {
                  CMVideoFormatDescriptionGetDimensions($0.formatDescription).width
                      < CMVideoFormatDescriptionGetDimensions($1.formatDescription).width
              }) else {
            s.commitConfiguration()
            throw DepthError.noDepthFormat
        }
        try dev.lockForConfiguration()
        dev.activeFormat = format
        dev.activeDepthDataFormat = depthFormat
        dev.unlockForConfiguration()
        s.commitConfiguration()
        let vd = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
        let dd = CMVideoFormatDescriptionGetDimensions(depthFormat.formatDescription)
        formatNote = "video \(vd.width)x\(vd.height) depth \(dd.width)x\(dd.height)"
        writeLog("# \(formatNote)")
        session = s
        s.startRunning()
    }

    func depthDataOutput(_ output: AVCaptureDepthDataOutput, didOutput depthData: AVDepthData,
                         timestamp: CMTime, connection: AVCaptureConnection) {
        guard completion != nil else { return }
        let depth = depthData.depthDataType == kCVPixelFormatType_DepthFloat32
            ? depthData : depthData.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32)
        let t = Clock.fromHostClockSeconds(CMTimeGetSeconds(timestamp))
        let absolute = depth.depthDataAccuracy == .absolute
        let stats = Self.centreStats(depth.depthDataMap)
        let cosTilt = Self.cosTilt(gravityProvider())
        let h = stats.map { $0.median * cosTilt }
        writeLog([
            CSVRow.time(t),
            stats.map { CSVRow.real($0.median) } ?? "",
            stats.map { CSVRow.real($0.p10) } ?? "",
            stats.map { CSVRow.real($0.p90) } ?? "",
            stats.map { CSVRow.real($0.validFraction) } ?? "0",
            CSVRow.real(cosTilt),
            h.map { CSVRow.real($0) } ?? "",
            absolute ? "absolute" : "relative",
            depth.depthDataQuality == .high ? "high" : "low",
        ].joined(separator: ","))

        guard absolute else {
            finish(message: "LiDAR depth is relative (not metric) — set h manually")
            return
        }
        // A tilted floor legitimately spreads depth across the patch, so only holes reject a
        // frame; motion and clutter are caught by the frame-to-frame spread check below.
        guard let s = stats, let h, s.validFraction >= 0.5 else { return }
        onSample?(t, h + Self.heightOffset)
        samples.append(h)
        if Clock.now() >= deadline, samples.count >= 10 {
            let sorted = samples.sorted()
            let med = sorted[sorted.count / 2] + Self.heightOffset
            let spread = sorted[sorted.count * 9 / 10] - sorted[sorted.count / 10]
            if med < Self.minHeight || med > Self.maxHeight {
                finish(message: String(format: "LiDAR h = %.3f m is outside %.2f–%.1f m — set h manually",
                                       med, Self.minHeight, Self.maxHeight))
            } else if spread > max(Self.maxRelativeSpread * med, 0.01) {
                finish(message: String(format: "LiDAR unstable (%.3f ± %.3f m) — hold still and retry", med, spread / 2))
            } else {
                let low = med < Self.recommendedMinHeight
                    ? String(format: " — mount ≥ %.0f cm for focus and speed range", Self.recommendedMinHeight * 100) : ""
                finish(height: med, message: String(format: "LiDAR h = %.3f m ± %.3f (%d frames, tilt %.0f°)%@",
                                                    med, spread / 2, samples.count, acos(cosTilt) * 180 / .pi, low))
            }
        }
    }

    struct PatchStats { var median: Double; var p10: Double; var p90: Double; var validFraction: Double }

    /// Depth statistics over the central 20% × 20% of the map (metres, along the optical axis).
    static func centreStats(_ map: CVPixelBuffer) -> PatchStats? {
        CVPixelBufferLockBaseAddress(map, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(map, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(map) else { return nil }
        let w = CVPixelBufferGetWidth(map), h = CVPixelBufferGetHeight(map)
        let bpr = CVPixelBufferGetBytesPerRow(map)
        let x0 = w * 2 / 5, x1 = w * 3 / 5, y0 = h * 2 / 5, y1 = h * 3 / 5
        let total = (x1 - x0) * (y1 - y0)
        var vals: [Float] = []
        vals.reserveCapacity(total)
        for y in y0..<y1 {
            let row = (base + y * bpr).assumingMemoryBound(to: Float32.self)
            for x in x0..<x1 {
                let v = row[x]
                if v.isFinite, v > 0.02, v < 5 { vals.append(v) }
            }
        }
        guard vals.count >= 10, total > 0 else { return nil }
        vals.sort()
        return PatchStats(median: Double(vals[vals.count / 2]),
                          p10: Double(vals[vals.count / 10]),
                          p90: Double(vals[vals.count * 9 / 10]),
                          validFraction: Double(vals.count) / Double(total))
    }

    /// cos of the angle between the back camera's optical axis (−z_device) and gravity.
    static func cosTilt(_ g: SIMD3<Double>?) -> Double {
        guard let g else { return 1 }
        let n = (g * g).sum().squareRoot()
        guard n > 0 else { return 1 }
        return max(0.5, min(1, abs(g.z) / n))
    }

    private func openLog() {
        let dir = Self.logDirectory()
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        let f = DateFormatter()
        f.dateFormat = "yyyy-MM-dd_HH-mm-ss"
        let url = dir.appendingPathComponent("\(f.string(from: Date())).csv")
        FileManager.default.createFile(atPath: url.path, contents: nil)
        log = try? FileHandle(forWritingTo: url)
        writeLog("t,depth_median,depth_p10,depth_p90,valid_frac,cos_tilt,h,accuracy,quality")
    }

    private func writeLog(_ line: String) {
        log?.write(Data((line + "\n").utf8))
    }

    private func finish(height: Double? = nil, message: String) {
        session?.stopRunning()
        session = nil
        writeLog("# result \(height.map { CSVRow.real($0) } ?? "none"): \(message)")
        try? log?.close()
        log = nil
        let c = completion
        completion = nil
        DispatchQueue.main.async { c?(height, message) }
    }
}
