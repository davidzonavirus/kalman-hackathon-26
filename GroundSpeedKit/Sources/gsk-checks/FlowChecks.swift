import Foundation
import KalmanCore
import OpticalFlow

/// Random-dot texture: sum of Gaussian dots evaluated at continuous coordinates, so a
/// subpixel shift is exact (no interpolation bias). Content is moved by (+sx, +sy).
func dotTexture(width: Int, height: Int, dots: [(x: Double, y: Double, a: Double)],
                sigma: Double, shift: (Double, Double)) -> [Float] {
    var img = [Float](repeating: 0, count: width * height)
    let rad = Int((4 * sigma).rounded(.up))
    let inv = 1 / (2 * sigma * sigma)
    for d in dots {
        let cx = d.x + shift.0, cy = d.y + shift.1
        let x0 = max(0, Int(cx) - rad), x1 = min(width - 1, Int(cx) + rad)
        let y0 = max(0, Int(cy) - rad), y1 = min(height - 1, Int(cy) + rad)
        if x0 > x1 || y0 > y1 { continue }
        for y in y0...y1 {
            let dy = Double(y) - cy
            for x in x0...x1 {
                let dx = Double(x) - cx
                img[y * width + x] += Float(d.a * exp(-(dx * dx + dy * dy) * inv))
            }
        }
    }
    return img
}

func randomDots(_ n: Int, width: Int, height: Int, rng: inout SeededRandom) -> [(x: Double, y: Double, a: Double)] {
    (0..<n).map { _ in (rng.uniform(-20, Double(width) + 20), rng.uniform(-20, Double(height) + 20), rng.uniform(0.3, 1)) }
}

func flowChecks(_ r: inout CheckRunner) {
    r.section("OpticalFlow")
    var rng = SeededRandom(seed: 11)
    let N = 128

    // [Float] entry point, subpixel shift in the N×N grid.
    let pc = PhaseCorrelator()
    let dots = randomDots(900, width: N, height: N, rng: &rng)
    let a = dotTexture(width: N, height: N, dots: dots, sigma: 1.2, shift: (0, 0))
    let b = dotTexture(width: N, height: N, dots: dots, sigma: 1.2, shift: (3.4, -5.7))
    r.check("first frame returns nil", pc.ingest(patch: a) == nil)
    if let s = pc.ingest(patch: b) {
        r.check("recovers (3.4, −5.7) px within 0.15", abs(s.dx - 3.4) < 0.15 && abs(s.dy + 5.7) < 0.15,
                String(format: "dx=%.3f dy=%.3f psr=%.1f", s.dx, s.dy, s.psr))
        r.check("PSR high on textured match", s.psr > 20, String(format: "%.1f", s.psr))
    } else { r.check("second frame returns a shift", false) }

    // A sweep of fractional shifts, with sensor noise.
    var worst = 0.0, minPSR = Double.infinity
    let shifts: [(Double, Double)] = [(0.25, 0.5), (-1.75, 2.1), (7.6, -0.3), (-12.45, -9.9), (0, 0), (15.3, 4.8)]
    for sh in shifts {
        pc.reset()
        var n1 = dotTexture(width: N, height: N, dots: dots, sigma: 1.2, shift: (0, 0))
        var n2 = dotTexture(width: N, height: N, dots: dots, sigma: 1.2, shift: sh)
        for i in 0..<n1.count { n1[i] += Float(rng.gaussian(0.03)); n2[i] += Float(rng.gaussian(0.03)) }
        _ = pc.ingest(patch: n1)
        if let s = pc.ingest(patch: n2) {
            worst = max(worst, abs(s.dx - sh.0), abs(s.dy - sh.1))
            minPSR = min(minPSR, s.psr)
        } else { worst = .infinity }
    }
    r.check("shift sweep (noisy) max error < 0.15 px", worst < 0.15, String(format: "worst=%.3f minPSR=%.1f", worst, minPSR))

    // Low PSR: pure noise vs unrelated noise frame.
    pc.reset()
    var psrNoise = 0.0
    for _ in 0..<6 {
        let nf = (0..<(N * N)).map { _ in Float(rng.gaussian(1)) }
        if let s = pc.ingest(patch: nf) { psrNoise = max(psrNoise, s.psr) }
    }
    // Unrelated textures (different dot fields)
    pc.reset()
    var psrUnrel = 0.0
    for _ in 0..<6 {
        let d2 = randomDots(900, width: N, height: N, rng: &rng)
        if let s = pc.ingest(patch: dotTexture(width: N, height: N, dots: d2, sigma: 1.2, shift: (0, 0))) {
            psrUnrel = max(psrUnrel, s.psr)
        }
    }
    r.check("PSR low on noise / unrelated frames", psrNoise < 8 && psrUnrel < 8,
            String(format: "noise max %.2f, unrelated max %.2f", psrNoise, psrUnrel))
    pc.reset()
    let flat = [Float](repeating: 0.5, count: N * N)
    _ = pc.ingest(patch: flat)
    let sFlat = pc.ingest(patch: flat)
    r.check("flat (lens covered) frame → PSR 0, finite", sFlat.map { $0.psr == 0 && $0.dx.isFinite } ?? false)

    // 8-bit luma path: centre crop 256 of a 300×280 image (bytesPerRow 320), 2× box downsample.
    let W = 300, H = 280, BPR = 320
    let lumaDots = randomDots(3500, width: W, height: H, rng: &rng)
    func luma(_ shift: (Double, Double)) -> [UInt8] {
        let f = dotTexture(width: W, height: H, dots: lumaDots, sigma: 2.0, shift: shift)
        var out = [UInt8](repeating: 0, count: BPR * H)
        for y in 0..<H { for x in 0..<W {
            let v = 40 + 150 * Double(f[y * W + x]) + rng.gaussian(2)
            out[y * BPR + x] = UInt8(max(0, min(255, v.rounded())))
        } }
        return out
    }
    let pl = PhaseCorrelator()
    let l1 = luma((0, 0)), l2 = luma((2.6, 1.3))
    _ = l1.withUnsafeBytes { pl.ingest(lumaBase: $0.baseAddress!, width: W, height: H, bytesPerRow: BPR) }
    let sl = l2.withUnsafeBytes { pl.ingest(lumaBase: $0.baseAddress!, width: W, height: H, bytesPerRow: BPR) }
    r.check("luma path: full-res (2.6, 1.3) → grid (1.3, 0.65) within 0.15",
            sl.map { abs($0.dx - 1.3) < 0.15 && abs($0.dy - 0.65) < 0.15 } ?? false,
            sl.map { String(format: "dx=%.3f dy=%.3f psr=%.1f", $0.dx, $0.dy, $0.psr) } ?? "nil")
    r.check("luma path: image smaller than crop → nil",
            l1.withUnsafeBytes { pl.ingest(lumaBase: $0.baseAddress!, width: 200, height: 200, bytesPerRow: BPR) } == nil)

    // Timing (informational; debug builds are ~10× slower than release).
    let t0 = Date()
    for k in 0..<60 {
        let buf = k % 2 == 0 ? l1 : l2
        _ = buf.withUnsafeBytes { pl.ingest(lumaBase: $0.baseAddress!, width: W, height: H, bytesPerRow: BPR) }
    }
    print(String(format: "      PhaseCorrelator: %.2f ms/frame (128², luma path)", Date().timeIntervalSince(t0) / 60 * 1000))

    // FlowConverter
    let fc = FlowConverter(focalPx: 1000, downsample: 2)
    let v = fc.velocity(dx: 2, dy: -1, dt: 1.0 / 120, h: 0.3)
    r.near("FlowConverter vx = −dx·ds/dt·h/f", v.vx, -2 * 2 * 120 * 0.3 / 1000, tol: 1e-12)
    r.near("FlowConverter vy sign", v.vy, 1 * 2 * 120 * 0.3 / 1000, tol: 1e-12)
    let g = SIMD3<Double>(0, 9.81 * sin(Double.pi / 6), 9.81 * cos(Double.pi / 6))
    let vt = fc.velocity(dx: 2, dy: 0, dt: 1.0 / 120, h: 0.3, gravityCam: g)
    r.near("FlowConverter tilt 30° → ×1/cos30", vt.vx, v.vx / cos(Double.pi / 6), tol: 1e-12)
    r.near("FlowConverter tilt clamp", fc.cosTilt(gravityCam: SIMD3(9.81, 0, 0)), 0.5, tol: 0)

    // MountMapping
    let m = MountMapping(swapXY: true, flipX: true, flipY: false)
    let mv = m.apply(PlanarVelocity(vx: 0.2, vy: -0.7))
    r.check("MountMapping swap then flip", mv.vx == 0.7 && mv.vy == 0.2)
    var allProper = true
    for (vx, vy) in [(1.0, 0.05), (-1.0, 0.1), (0.1, 1.0), (0.05, -1.0)] {
        let mm = MountMapping.fromForwardPush(vxCam: vx, vyCam: vy)
        let out = mm.apply(PlanarVelocity(vx: vx, vy: vy))
        if !(mm.isProper && out.vx > 0.9) { allProper = false }
    }
    r.check("MountMapping.fromForwardPush → forward +x, proper", allProper)
    r.check("MountMapping.standard is proper, identity is not", MountMapping.standard.isProper && !MountMapping.identity.isProper)
    let mj = (try? JSONEncoder().encode(MountMapping.standard)) ?? Data()
    r.check("MountMapping JSON keys", jsonKeys(mj) == ["swap_xy", "flip_x", "flip_y"])
}
