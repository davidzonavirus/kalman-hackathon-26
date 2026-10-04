import Accelerate
import Foundation

/// Result of one phase-correlation step.
///
/// Sign convention: `dx > 0` means image *content* moved toward +x (right, increasing
/// column index) between the previous and the current frame; `dy > 0` means content
/// moved toward +y (down, increasing row index). Units are pixels of the *downsampled*
/// N×N grid (multiply by `downsample` for full-resolution pixels).
public struct FlowShift: Sendable, Equatable {
    public var dx: Double
    public var dy: Double
    /// Peak-to-sidelobe ratio: (peak − mean(sidelobe)) / std(sidelobe), sidelobe =
    /// surface minus an 11×11 box around the peak. ~3–6 for unrelated/noise frames,
    /// tens to hundreds for good texture.
    public var psr: Double

    public init(dx: Double, dy: Double, psr: Double) {
        self.dx = dx; self.dy = dy; self.psr = psr
    }
}

/// Frame-to-frame translation estimator by phase correlation (vDSP 2-D FFT).
///
/// Pipeline per frame (all buffers preallocated; no allocation in `ingest`):
/// 1. Luma: centre crop of `N·downsample` square, box-downsampled by `downsample` → N×N.
/// 2. Subtract mean, multiply by a 2-D Hann window (suppresses edge wrap-around).
/// 3. F_cur = FFT2(patch). Cross-power R = F_cur · conj(F_prev) / |F_cur · conj(F_prev)|.
///    If cur(x) = prev(x − d) then R = e^{−i2πk·d/N} and IFFT2(R) = δ(x − d): the peak sits at +d.
/// 4. R is multiplied by a Gaussian low-pass G(k) = exp(−|k|²/(2σ_k²)), which turns the
///    ideal delta into a small Gaussian blob of σ = N/(2π σ_k) px. That both suppresses noisy
///    high frequencies and makes a 3-point Gaussian (log-parabola) fit essentially unbiased
///    for subpixel refinement.
/// 5. Integer peak (wrap to ±N/2), subpixel offset per axis
///    δ = (ln l − ln r) / (2(ln l − 2 ln c + ln r)), then PSR.
/// The previous spectrum is cached, so each frame costs one forward + one inverse FFT.
public final class PhaseCorrelator {
    public let n: Int
    public let downsample: Int
    /// Spatial σ (pixels) of the correlation peak after spectral low-pass.
    public let peakSigma: Double

    private let log2n: vDSP_Length
    private let setup: FFTSetup
    private let count: Int

    private let window: UnsafeMutablePointer<Float>
    private let lowpass: UnsafeMutablePointer<Float>
    private let patch: UnsafeMutablePointer<Float>
    private let curRe: UnsafeMutablePointer<Float>, curIm: UnsafeMutablePointer<Float>
    private let prevRe: UnsafeMutablePointer<Float>, prevIm: UnsafeMutablePointer<Float>
    private let xRe: UnsafeMutablePointer<Float>, xIm: UnsafeMutablePointer<Float>
    private let tmp: UnsafeMutablePointer<Float>
    private var hasPrev = false

    /// Half-size of the sidelobe exclusion box (11×11 → 5).
    private let excl = 5

    /// - Parameters:
    ///   - n: correlation size (power of two), default 128.
    ///   - downsample: box-downsample factor applied to the centre crop (crop = n·downsample).
    ///   - peakSigma: spatial σ of the smoothed correlation peak in px (0.6–1.5 sensible).
    public init(n: Int = 128, downsample: Int = 2, peakSigma: Double = 0.85) {
        precondition(n >= 16 && n & (n - 1) == 0, "n must be a power of two ≥ 16")
        precondition(downsample >= 1)
        self.n = n
        self.downsample = downsample
        self.peakSigma = peakSigma
        self.count = n * n
        self.log2n = vDSP_Length(n.trailingZeroBitCount)
        guard let s = vDSP_create_fftsetup(log2n, FFTRadix(kFFTRadix2)) else {
            fatalError("vDSP_create_fftsetup failed")
        }
        setup = s
        func alloc() -> UnsafeMutablePointer<Float> {
            let p = UnsafeMutablePointer<Float>.allocate(capacity: n * n)
            p.initialize(repeating: 0, count: n * n)
            return p
        }
        window = alloc(); lowpass = alloc(); patch = alloc()
        curRe = alloc(); curIm = alloc(); prevRe = alloc(); prevIm = alloc()
        xRe = alloc(); xIm = alloc(); tmp = alloc()

        // Separable 2-D Hann window.
        var w1 = [Float](repeating: 0, count: n)
        for i in 0..<n { w1[i] = Float(0.5 - 0.5 * cos(2 * Double.pi * Double(i) / Double(n - 1))) }
        // Gaussian low-pass on the (wrapped) frequency grid. Spatial σ_s ↔ σ_k = N/(2π σ_s).
        let sk = Double(n) / (2 * Double.pi * peakSigma)
        var g1 = [Float](repeating: 0, count: n)
        for i in 0..<n {
            let k = Double(i <= n / 2 ? i : i - n)
            g1[i] = Float(exp(-k * k / (2 * sk * sk)))
        }
        for r in 0..<n { for c in 0..<n {
            window[r * n + c] = w1[r] * w1[c]
            lowpass[r * n + c] = g1[r] * g1[c]
        } }
    }

    deinit {
        vDSP_destroy_fftsetup(setup)
        for p in [window, lowpass, patch, curRe, curIm, prevRe, prevIm, xRe, xIm, tmp] { p.deallocate() }
    }

    /// Forget the previous frame (next ingest returns nil).
    public func reset() { hasPrev = false }

    /// Full-resolution crop side length required from the camera image.
    public var cropSize: Int { n * downsample }

    /// Ingest an 8-bit luma plane (e.g. CVPixelBuffer plane 0 of a 420f/420v buffer).
    /// Crops a `cropSize`² square centred `offsetX`/`offsetY` full-res px from the image centre
    /// and box-downsamples. Returns nil for the first frame or if the crop leaves the image.
    ///
    /// `predictX`/`predictY` (full-res px): expected content motion since the previous frame.
    /// The current frame is then also cropped at offset + prediction, so the correlation only
    /// has to find the residual and the speed range is no longer limited to ±n/2 per frame
    /// (only by the image size). The previous-frame reference is always the un-predicted
    /// crop, so each frame's prediction is independent. The returned shift is the total.
    public func ingest(lumaBase: UnsafeRawPointer, width: Int, height: Int, bytesPerRow: Int,
                       offsetX: Int = 0, offsetY: Int = 0,
                       predictX: Int = 0, predictY: Int = 0) -> FlowShift? {
        let crop = cropSize, ds = downsample
        let x0 = (width - crop) / 2 + offsetX, y0 = (height - crop) / 2 + offsetY
        guard x0 >= 0, y0 >= 0, x0 + crop <= width, y0 + crop <= height else { return nil }
        // Prediction in whole downsampled px, clamped so the crop stays inside the image.
        let px = max(-x0, min(width - crop - x0, Int((Double(predictX) / Double(ds)).rounded()) * ds))
        let py = max(-y0, min(height - crop - y0, Int((Double(predictY) / Double(ds)).rounded()) * ds))
        let base = lumaBase.assumingMemoryBound(to: UInt8.self)
        guard (px != 0 || py != 0) && hasPrev else {
            loadPatch(base, bytesPerRow: bytesPerRow, x0: x0, y0: y0)
            return process()
        }
        loadPatch(base, bytesPerRow: bytesPerRow, x0: x0 + px, y0: y0 + py)
        forward()
        let pdx = px / ds, pdy = py / ds
        // A peak at total motion 0 is the static pattern (sensor noise, vignetting, lens dirt),
        // which can outshine motion-blurred texture; mask it once the prediction is clear of it.
        let mask: (Int, Int)? = abs(pdx) + abs(pdy) >= 4 ? (-pdx, -pdy) : nil
        var shift = correlate(mask: mask)
        shift.dx += Double(pdx); shift.dy += Double(pdy)
        loadPatch(base, bytesPerRow: bytesPerRow, x0: x0, y0: y0)
        forward()
        storePrev()
        return shift
    }

    private func loadPatch(_ base: UnsafePointer<UInt8>, bytesPerRow: Int, x0: Int, y0: Int) {
        let ds = downsample
        let scale = 1 / Float(ds * ds)
        for r in 0..<n {
            let dst = patch + r * n
            for c in 0..<n { dst[c] = 0 }
            for sy in 0..<ds {
                let row = base + (y0 + r * ds + sy) * bytesPerRow + x0
                for c in 0..<n {
                    var acc: Float = 0
                    let p = row + c * ds
                    for sx in 0..<ds { acc += Float(p[sx]) }
                    dst[c] += acc
                }
            }
            for c in 0..<n { dst[c] *= scale }
        }
    }

    /// Ingest an already-downsampled N×N row-major float patch (any intensity scale).
    public func ingest(patch values: [Float]) -> FlowShift? {
        precondition(values.count == count, "patch must be n×n")
        values.withUnsafeBufferPointer { patch.update(from: $0.baseAddress!, count: count) }
        return process()
    }

    // MARK: - Core

    private func process() -> FlowShift? {
        forward()
        defer { storePrev() }
        guard hasPrev else { return nil }
        return correlate(mask: nil)
    }

    /// patch → zero-mean, Hann-windowed spectrum in curRe/curIm.
    private func forward() {
        let len = vDSP_Length(count)
        var mean: Float = 0
        vDSP_meanv(patch, 1, &mean, len)
        var negMean = -mean
        vDSP_vsadd(patch, 1, &negMean, curRe, 1, len)
        vDSP_vmul(curRe, 1, window, 1, curRe, 1, len)
        vDSP_vclr(curIm, 1, len)
        var cur = DSPSplitComplex(realp: curRe, imagp: curIm)
        vDSP_fft2d_zip(setup, &cur, 1, 0, log2n, log2n, FFTDirection(kFFTDirection_Forward))
    }

    private func storePrev() {
        prevRe.update(from: curRe, count: count)
        prevIm.update(from: curIm, count: count)
        hasPrev = true
    }

    /// Correlates the current spectrum with the previous one. `mask` (downsampled px, wrapped)
    /// suppresses a 5×5 neighbourhood of the surface before the peak search.
    private func correlate(mask: (Int, Int)?) -> FlowShift {
        let len = vDSP_Length(count)
        var cur = DSPSplitComplex(realp: curRe, imagp: curIm)
        // X = F_cur · conj(F_prev)
        var prev = DSPSplitComplex(realp: prevRe, imagp: prevIm)
        var cross = DSPSplitComplex(realp: xRe, imagp: xIm)
        vDSP_zvmul(&prev, 1, &cur, 1, &cross, 1, len, -1)   // conjugate = -1 conjugates first arg
        // |X| → tmp, normalise, apply low-pass (G / |X|), guarding tiny magnitudes.
        vDSP_zvabs(&cross, 1, tmp, 1, len)
        var maxMag: Float = 0
        vDSP_maxv(tmp, 1, &maxMag, len)
        guard maxMag > 0, maxMag.isFinite else { return FlowShift(dx: 0, dy: 0, psr: 0) }
        let floor = maxMag * 1e-7
        for i in 0..<count {
            let m = tmp[i]
            tmp[i] = m > floor ? lowpass[i] / m : 0
        }
        vDSP_vmul(xRe, 1, tmp, 1, xRe, 1, len)
        vDSP_vmul(xIm, 1, tmp, 1, xIm, 1, len)
        vDSP_fft2d_zip(setup, &cross, 1, 0, log2n, log2n, FFTDirection(kFFTDirection_Inverse))
        let surf = xRe   // real part = correlation surface (unnormalised, scale irrelevant)
        func wrapDist(_ v: Int) -> Int { let w = ((v % n) + n) % n; return min(w, n - w) }
        // Skip when the wrapped mask lands on the expected residual (≈ 0): at multiples of
        // n·downsample of predicted motion the static and moving peaks coincide.
        if let (mx, my) = mask, max(wrapDist(mx), wrapDist(my)) > 8 {
            let mx = ((mx % n) + n) % n, my = ((my % n) + n) % n
            var lo: Float = 0
            vDSP_minv(surf, 1, &lo, len)
            for dr in -2...2 { for dc in -2...2 {
                surf[((my + dr + n) % n) * n + ((mx + dc + n) % n)] = lo
            } }
        }

        var peak: Float = 0
        var peakIdx: vDSP_Length = 0
        vDSP_maxvi(surf, 1, &peak, &peakIdx, len)
        let pr = Int(peakIdx) / n, pc = Int(peakIdx) % n

        // Subpixel: 3-point Gaussian fit per axis (falls back to parabola if any ≤ 0).
        func at(_ r: Int, _ c: Int) -> Float { surf[((r + n) % n) * n + ((c + n) % n)] }
        let subX = Self.subpixel(at(pr, pc - 1), peak, at(pr, pc + 1))
        let subY = Self.subpixel(at(pr - 1, pc), peak, at(pr + 1, pc))

        // PSR over the surface excluding an 11×11 box around the peak (wrap-aware).
        var total: Float = 0, totalSq: Float = 0
        vDSP_sve(surf, 1, &total, len)
        vDSP_svesq(surf, 1, &totalSq, len)
        var exSum: Float = 0, exSq: Float = 0
        for dr in -excl...excl { for dc in -excl...excl {
            let v = at(pr + dr, pc + dc); exSum += v; exSq += v * v
        } }
        let side = Double(count - (2 * excl + 1) * (2 * excl + 1))
        let sMean = Double(total - exSum) / side
        let sVar = max(0, Double(totalSq - exSq) / side - sMean * sMean)
        let sStd = sVar.squareRoot()
        let psr = sStd > 0 ? (Double(peak) - sMean) / sStd : 0

        var dx = Double(pc) + subX, dy = Double(pr) + subY
        let half = Double(n / 2)
        if dx >= half { dx -= Double(n) }
        if dy >= half { dy -= Double(n) }
        return FlowShift(dx: dx, dy: dy, psr: psr)
    }

    private static func subpixel(_ l: Float, _ c: Float, _ r: Float) -> Double {
        if l > 0, c > 0, r > 0 {
            let ll = log(Double(l)), lc = log(Double(c)), lr = log(Double(r))
            let den = 2 * (ll - 2 * lc + lr)
            if den < 0 { return max(-0.5, min(0.5, (ll - lr) / den)) }
        }
        let den = 2 * Double(l - 2 * c + r)
        guard den < 0 else { return 0 }
        return max(-0.5, min(0.5, Double(l - r) / den))
    }
}
