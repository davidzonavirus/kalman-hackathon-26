/**
 ******************************************************************************
 * @file : main.swift
 * @brief : Speed-range benchmark for PhaseCorrelator (shift sweep with motion blur)
 * @author : David Nguyen
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 MRacing. All rights reserved.
 * MRacing is a trademark of MRacing FSAE.
 *
 * Written by David Nguyen.
 *
 * This firmware is the property of MRacing FSAE. Unauthorized use, copying,
 * or distribution is prohibited.
 *
 ******************************************************************************
 */

// Usage: swift run -c release flowbench [--noise σ] [--exposure s] [--fps f] [--h m] [--focal px]
//        [--predict [--predict-err px]]   (tracking: crop shifted by the predicted motion)
//
// Feeds frame pairs of a synthetic floor texture, shifted by a known amount along x with the
// motion blur that shift implies (blur = shift × exposure × fps), plus Gaussian sensor noise,
// through the real PhaseCorrelator (n 128, downsample 2). Reports PSR and shift error per
// speed, and the highest speed where ≥ 99 % of pairs are within 0.5 px with PSR ≥ psr_min.

import Foundation
import OpticalFlow

var args = Array(CommandLine.arguments.dropFirst())
func opt(_ name: String, _ d: Double) -> Double {
    guard let i = args.firstIndex(of: name), i + 1 < args.count, let v = Double(args[i + 1]) else { return d }
    return v
}
let noise = opt("--noise", 6)
let exposure = opt("--exposure", 1.0 / 1000)
let fps = opt("--fps", 240)
let h = opt("--h", 0.235)
let focal = opt("--focal", 923.5)
let psrMin = opt("--psr-min", 8)
let pairs = Int(opt("--pairs", 60))
/// Static-in-frame artefacts (they move with the camera, not the floor):
/// torch hotspot depth (0…1, multiplicative Gaussian), fixed-pattern noise σ (DN), texture gain.
let hotspot = opt("--hotspot", 0)
let fpn = opt("--fpn", 0)
let contrast = opt("--contrast", 1)

struct RNG {
    var s: UInt64
    mutating func next() -> Double {
        s = s &* 6364136223846793005 &+ 1442695040888963407
        return Double(s >> 11) / Double(1 << 53)
    }
    mutating func gauss() -> Double {
        let u = max(1e-12, next()), v = next()
        return (-2 * log(u)).squareRoot() * cos(2 * .pi * v)
    }
}
var rng = RNG(s: 7)

// Texture: white noise box-blurred at a few scales (speckle + mottling, like concrete/carpet).
let W = args.contains("--predict") ? 3600 : 1600, H = 600
func boxBlur(_ a: [Float], _ r: Int) -> [Float] {
    var t = a, o = a
    for y in 0..<H {
        var acc: Float = 0
        for x in -r...r { acc += a[y * W + min(W - 1, max(0, x))] }
        for x in 0..<W {
            t[y * W + x] = acc / Float(2 * r + 1)
            acc += a[y * W + min(W - 1, x + r + 1)] - a[y * W + max(0, x - r)]
        }
    }
    for x in 0..<W {
        var acc: Float = 0
        for y in -r...r { acc += t[min(H - 1, max(0, y)) * W + x] }
        for y in 0..<H {
            o[y * W + x] = acc / Float(2 * r + 1)
            acc += t[min(H - 1, y + r + 1) * W + x] - t[max(0, y - r) * W + x]
        }
    }
    return o
}
var texture = [Float](repeating: 0, count: W * H)
for (r, gain) in [(1, 1.0), (3, 1.2), (8, 1.5)] {
    let n = (0..<(W * H)).map { _ in Float(rng.gauss()) }
    let b = boxBlur(n, r)
    var sq: Float = 0; for v in b { sq += v * v }
    let std = (sq / Float(b.count)).squareRoot()
    for i in 0..<texture.count { texture[i] += b[i] / std * Float(gain) * 12 }
}

/// Horizontal motion blur over `len` px (fractional, linear interpolation).
func motionBlur(_ a: [Float], _ len: Double) -> [Float] {
    guard len > 0.5 else { return a }
    let taps = max(2, Int(len.rounded(.up)) * 2)
    var o = [Float](repeating: 0, count: a.count)
    for y in 0..<H {
        for x in 0..<W {
            var acc: Float = 0
            for k in 0..<taps {
                let fx = Double(x) + len * Double(k) / Double(taps - 1)
                let x0 = min(W - 1, Int(fx)), x1 = min(W - 1, x0 + 1)
                let w = Float(fx - Double(Int(fx)))
                acc += a[y * W + x0] * (1 - w) + a[y * W + x1] * w
            }
            o[y * W + x] = acc / Float(taps)
        }
    }
    return o
}

let crop = 256
let step = opt("--step", 4), maxShift = opt("--max", 128)
/// Tracking mode: frames are wide enough for the predicted crop, and the prediction is the
/// true motion plus a uniform error of up to ±`--predict-err` full-res px.
let predict = args.contains("--predict")
let predictErr = opt("--predict-err", 20)
let frameW = predict ? crop + 2 * Int(maxShift + predictErr) + 8 : crop
let fpnPattern = (0..<(frameW * crop)).map { _ in Float(rng.gauss() * fpn) }
let illum: [Float] = (0..<(frameW * crop)).map { i in
    let x = Double(i % frameW - frameW / 2) + 28, y = Double(i / frameW) - 140   // off-centre hotspot
    return Float(1 - hotspot + hotspot * exp(-(x * x + y * y) / (2 * 90 * 90)) * 1.6)
}
func frame(_ img: [Float], x0: Double, y0: Int) -> [UInt8] {
    var out = [UInt8](repeating: 0, count: frameW * crop)
    let xi = Int(x0), w = Float(x0 - Double(xi))
    for y in 0..<crop {
        for x in 0..<frameW {
            let a = img[(y0 + y) * W + xi + x], b = img[(y0 + y) * W + xi + x + 1]
            let i = y * frameW + x
            let v = (128 + (a * (1 - w) + b * w) * Float(contrast)) * illum[i] + fpnPattern[i] + Float(rng.gauss() * noise)
            out[i] = UInt8(max(0, min(255, v.rounded())))
        }
    }
    return out
}

let corr = PhaseCorrelator(n: 128, downsample: 2)
let mPerPx = h / focal            // m per full-res px on the floor
print(String(format: "h %.3f m, f %.1f px, %.0f fps, exposure 1/%.0f s, noise σ %.1f DN, psr_min %.0f",
             h, focal, fps, 1 / exposure, noise, psrMin))
print("shift(ds px)  speed m/s  blur(px)  psr p10/med   |err| p99 (ds px)  bias %  ok%")
var lastGood = 0.0
var blurCache: [Int: [Float]] = [:]
for sFull in stride(from: step, through: maxShift, by: step) {   // full-res px per frame
    let truth = sFull / 2                        // downsampled px
    let speed = sFull * mPerPx * fps
    let blur = sFull * exposure * fps
    let key = Int((blur * 4).rounded())
    let img = blurCache[key] ?? motionBlur(texture, blur)
    blurCache[key] = img
    var psrs: [Double] = [], errs: [Double] = []
    var ok = 0, biasSum = 0.0, biasN = 0
    for _ in 0..<pairs {
        let x0 = 4 + rng.next() * Double(W - frameW - Int(sFull) - 10)
        let y0 = 4 + Int(rng.next() * Double(H - crop - 10))
        corr.reset()
        let a = frame(img, x0: x0, y0: y0), b = frame(img, x0: x0 + sFull, y0: y0)
        let p = predict ? Int((-sFull + (rng.next() * 2 - 1) * predictErr).rounded()) : 0
        _ = a.withUnsafeBytes { corr.ingest(lumaBase: $0.baseAddress!, width: frameW, height: crop, bytesPerRow: frameW) }
        guard let r = b.withUnsafeBytes({
            corr.ingest(lumaBase: $0.baseAddress!, width: frameW, height: crop, bytesPerRow: frameW, predictX: p)
        }) else { continue }
        // Camera moves +x over the floor → image content moves −x.
        let err = hypot(abs(r.dx) - truth, r.dy)
        psrs.append(r.psr); errs.append(err)
        if err < 0.5 && r.psr >= psrMin { ok += 1; biasSum += abs(r.dx) / truth - 1; biasN += 1 }
    }
    psrs.sort(); errs.sort()
    let pct = 100 * Double(ok) / Double(max(1, psrs.count))
    print(String(format: "%6.1f        %6.2f    %5.1f    %4.0f/%4.0f      %7.2f          %+5.2f  %5.1f",
                 truth, speed, blur, psrs[psrs.count / 10], psrs[psrs.count / 2],
                 errs[min(errs.count - 1, errs.count * 99 / 100)],
                 biasN > 0 ? 100 * biasSum / Double(biasN) : .nan, pct))
    if pct >= 99 { lastGood = speed } else if pct < 50 { break }
}
print(String(format: "max reliable speed ≈ %.2f m/s", lastGood))
