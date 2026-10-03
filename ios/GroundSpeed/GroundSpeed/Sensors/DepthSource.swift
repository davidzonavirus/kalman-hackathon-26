import AVFoundation
import CoreVideo
import Foundation
import PhoneRuntime

/// LiDAR height measurement in a SEPARATE, short-lived capture session.
///
/// iOS can't run two capture sessions at once, and the LiDAR depth camera can't deliver
/// 120 fps video, so height is *measured* (cart still, flow session paused): ~1.5 s of depth
/// frames, median of the centre patch of each, median over frames. Every depth sample is
/// also forwarded (so it lands in depth.csv and LIDAR_OK lights while measuring). If the
/// device has no LiDAR or the session fails, the completion gets nil and the app keeps
/// using the manual height.
final class DepthSource: NSObject, AVCaptureDepthDataOutputDelegate, @unchecked Sendable {
    typealias Sink = (_ t: Double, _ h: Double) -> Void

    var onSample: Sink?

    private let queue = DispatchQueue(label: "gsk.depth", qos: .userInitiated)
    private var session: AVCaptureSession?
    private var depthOutput = AVCaptureDepthDataOutput()
    private var samples: [Double] = []
    private var completion: ((Double?, String) -> Void)?
    private var deadline = 0.0

    static var isAvailable: Bool {
        AVCaptureDevice.default(.builtInLiDARDepthCamera, for: .video, position: .back) != nil
    }

    /// Measures camera height (m). Completion runs on the main queue with (height, message).
    func measure(duration: Double = 1.5, completion: @escaping (Double?, String) -> Void) {
        queue.async {
            self.samples = []
            self.completion = completion
            do {
                try self.startSession()
            } catch {
                self.finish(message: "LiDAR unavailable: \(error)")
                return
            }
            self.deadline = Clock.now() + duration
            weak let me = self
            self.queue.asyncAfter(deadline: .now() + duration + 3.0) {
                // Safety net if no frames arrive.
                guard let me, me.completion != nil else { return }
                me.finish(message: "LiDAR: no depth frames")
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
        depthOutput.isFilteringEnabled = true
        depthOutput.alwaysDiscardsLateDepthData = true
        depthOutput.setDelegate(self, callbackQueue: queue)

        // Pick a video format that offers a float depth format, then the largest depth format.
        let isFloatDepth: (AVCaptureDevice.Format) -> Bool = { f in
            let st = CMFormatDescriptionGetMediaSubType(f.formatDescription)
            return st == kCVPixelFormatType_DepthFloat32 || st == kCVPixelFormatType_DepthFloat16
        }
        guard let format = dev.formats.last(where: { $0.supportedDepthDataFormats.contains(where: isFloatDepth) }),
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
        session = s
        s.startRunning()
    }

    func depthDataOutput(_ output: AVCaptureDepthDataOutput, didOutput depthData: AVDepthData,
                         timestamp: CMTime, connection: AVCaptureConnection) {
        guard completion != nil else { return }
        let depth = depthData.depthDataType == kCVPixelFormatType_DepthFloat32
            ? depthData : depthData.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32)
        guard let h = Self.centreMedian(depth.depthDataMap) else { return }
        let t = Clock.fromHostClockSeconds(CMTimeGetSeconds(timestamp))
        onSample?(t, h)
        samples.append(h)
        if Clock.now() >= deadline, samples.count >= 5 {
            let sorted = samples.sorted()
            let med = sorted[sorted.count / 2]
            finish(height: med, message: String(format: "LiDAR h = %.3f m (%d frames)", med, samples.count))
        }
    }

    /// Median of finite positive depths in the central 20% × 20% of the map (metres).
    static func centreMedian(_ map: CVPixelBuffer) -> Double? {
        CVPixelBufferLockBaseAddress(map, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(map, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(map) else { return nil }
        let w = CVPixelBufferGetWidth(map), h = CVPixelBufferGetHeight(map)
        let bpr = CVPixelBufferGetBytesPerRow(map)
        let x0 = w * 2 / 5, x1 = w * 3 / 5, y0 = h * 2 / 5, y1 = h * 3 / 5
        var vals: [Float] = []
        vals.reserveCapacity((x1 - x0) * (y1 - y0))
        for y in y0..<y1 {
            let row = (base + y * bpr).assumingMemoryBound(to: Float32.self)
            for x in x0..<x1 {
                let v = row[x]
                if v.isFinite, v > 0.02, v < 5 { vals.append(v) }
            }
        }
        guard vals.count >= 10 else { return nil }
        vals.sort()
        return Double(vals[vals.count / 2])
    }

    private func finish(height: Double? = nil, message: String) {
        session?.stopRunning()
        session = nil
        let c = completion
        completion = nil
        DispatchQueue.main.async { c?(height, message) }
    }
}
