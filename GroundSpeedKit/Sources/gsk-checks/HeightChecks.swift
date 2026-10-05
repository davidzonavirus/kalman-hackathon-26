/**
 ******************************************************************************
 * @file : HeightChecks.swift
 * @brief : Image-expansion height tracking on rendered frames of a tilted floor.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 MRacing. All rights reserved.
 * MRacing is a trademark of MRacing FSAE.
 *
 * This firmware is the property of MRacing FSAE. Unauthorized use, copying,
 * or distribution is prohibited.
 *
 ******************************************************************************
 */

import Foundation
import OpticalFlow

/// Pinhole camera over the floor plane z = 0, rendering multi-octave value noise.
private struct FloorRenderer {
    let width = 512, height = 320
    let focal = 450.0

    private func hash(_ i: Int, _ j: Int, _ o: Int) -> Float {
        var h = UInt64(bitPattern: Int64(i &* 73_856_093 ^ j &* 19_349_663 ^ o &* 83_492_791))
        h = (h ^ (h >> 33)) &* 0xff51afd7ed558ccd
        h = (h ^ (h >> 33)) &* 0xc4ceb9fe1a85ec53
        return Float(h >> 40) / Float(1 << 24)
    }

    private func noise(_ x: Double, _ y: Double, _ o: Int) -> Float {
        let fx = x.rounded(.down), fy = y.rounded(.down)
        let i = Int(fx), j = Int(fy)
        var tx = Float(x - fx), ty = Float(y - fy)
        tx = tx * tx * (3 - 2 * tx); ty = ty * ty * (3 - 2 * ty)
        let a = hash(i, j, o), b = hash(i + 1, j, o), c = hash(i, j + 1, o), d = hash(i + 1, j + 1, o)
        return (a + (b - a) * tx) + ((c + (d - c) * tx) - (a + (b - a) * tx)) * ty
    }

    /// Floor texture at world (X, Y) metres.
    private func texture(_ x: Double, _ y: Double) -> Float {
        var v: Float = 0
        for (o, cell) in [0.0025, 0.005, 0.011].enumerated() { v += noise(x / cell, y / cell, o) * Float(1 << o) }
        return v
    }

    /// Camera at (px, py, h); `pitch` tilts the optical axis along image +x, `roll` along +y.
    func render(px: Double, py: Double, h: Double, pitch: Double, roll: Double) -> [UInt8] {
        let cp = cos(pitch), sp = sin(pitch), cr = cos(roll), sr = sin(roll)
        var img = [UInt8](repeating: 0, count: width * height)
        let cx = Double(width) / 2 - 0.5, cy = Double(height) / 2 - 0.5
        for r in 0..<height {
            for c in 0..<width {
                // Camera ray; camera looks down -Z, image x = world X, image y = world Y.
                let x = (Double(c) - cx) / focal, y = (Double(r) - cy) / focal
                // Pitch about image y, then roll about image x.
                let x1 = x * cp + sp, z1 = -x * sp + cp
                let y2 = y * cr + z1 * sr, z2 = -y * sr + z1 * cr
                let lambda = h / z2
                let v = texture(px + lambda * x1, py + lambda * y2)
                img[r * width + c] = UInt8(max(0, min(255, 20 + v * 33)))
            }
        }
        return img
    }
}

private struct ExpansionRig {
    let renderer = FloorRenderer()
    let offX = 176, offY = 88
    let patches = (0..<4).map { _ in PhaseCorrelator(n: 64, downsample: 2) }

    func shifts(_ img: [UInt8]) -> [FlowShift]? {
        let offs = [(-offX, 0), (offX, 0), (0, -offY), (0, offY)]
        // Every patch must see every frame, or they'd compare different frame pairs.
        let out = offs.enumerated().map { k, o in
            img.withUnsafeBytes {
                patches[k].ingest(lumaBase: $0.baseAddress!, width: renderer.width, height: renderer.height,
                                  bytesPerRow: renderer.width, offsetX: o.0, offsetY: o.1)
            }
        }
        let shifts = out.compactMap { $0 }
        return shifts.count == 4 ? shifts : nil
    }

    /// Runs a trajectory; returns (tracked h, naive isotropic h) at the end.
    func run(frames: Int, h0: Double, pose: (Int) -> (x: Double, y: Double, h: Double),
             pitch: Double, roll: Double) -> (tracked: Double, naive: Double) {
        var tracker = HeightTracker(height: h0)
        var naive = h0
        let ox = Double(offX) / 2, oy = Double(offY) / 2   // downsampled px
        for f in 0...frames {
            let p = pose(f)
            let img = renderer.render(px: p.x, py: p.y, h: p.h, pitch: pitch, roll: roll)
            guard let s = shifts(img) else { continue }
            tracker.update(left: s[0], right: s[1], top: s[2], bottom: s[3], offsetX: ox, offsetY: oy)
            let iso = -((s[1].dx - s[0].dx) / (2 * ox) + (s[3].dy - s[2].dy) / (2 * oy)) / 2
            naive *= 1 + iso
        }
        return (tracker.height, naive)
    }
}

func heightChecks(_ r: inout CheckRunner) {
    r.section("HeightTracker")
    let deg = Double.pi / 180
    let step = 0.0012   // m per frame ≈ 2.7 px at h = 0.2

    // Constant height on a tilted mount: tracked h must not drift with distance travelled.
    for (name, dir) in [("along x", (1.0, 0.0)), ("along y", (0.0, 1.0)), ("diagonal", (0.8, 0.6))] {
        let res = ExpansionRig().run(frames: 30, h0: 0.20, pose: { f in
            (Double(f) * step * dir.0, Double(f) * step * dir.1, 0.20)
        }, pitch: 5 * deg, roll: -3 * deg)
        r.check("tilted 5°/−3°, constant h, moving \(name): tracked h within 0.5 %",
                abs(res.tracked / 0.20 - 1) < 0.005,
                String(format: "tracked %+.2f %%, isotropic-only %+.2f %%",
                       (res.tracked / 0.20 - 1) * 100, (res.naive / 0.20 - 1) * 100))
    }
    // Ride height rising 0.20 -> 0.24 m while moving on a tilted mount.
    let rise = ExpansionRig().run(frames: 30, h0: 0.20, pose: { f in
        (Double(f) * step * 0.9, Double(f) * step * 0.3, 0.20 + 0.04 * Double(f) / 30)
    }, pitch: 4 * deg, roll: 2 * deg)
    r.check("h 0.20 -> 0.24 m while moving: tracked within 1 %", abs(rise.tracked / 0.24 - 1) < 0.01,
            String(format: "tracked %.4f m", rise.tracked))
    // Lowering at rest.
    let drop = ExpansionRig().run(frames: 20, h0: 0.25, pose: { f in (0, 0, 0.25 - 0.06 * Double(f) / 20) },
                                  pitch: 3 * deg, roll: 0)
    r.check("h 0.25 -> 0.19 m at rest: tracked within 1 %", abs(drop.tracked / 0.19 - 1) < 0.01,
            String(format: "tracked %.4f m", drop.tracked))

    var t = HeightTracker(height: 0.2)
    let weak = FlowShift(dx: 0, dy: 0, psr: 3)
    let ok = FlowShift(dx: 0, dy: 0, psr: 50)
    r.check("HeightTracker skips weak patches", !t.update(left: weak, right: ok, top: ok, bottom: ok, offsetX: 50, offsetY: 50))
    r.check("HeightTracker rejects a >2 % jump",
            !t.update(left: FlowShift(dx: -20, dy: 0, psr: 50), right: FlowShift(dx: 20, dy: 0, psr: 50),
                      top: FlowShift(dx: 0, dy: -20, psr: 50), bottom: FlowShift(dx: 0, dy: 20, psr: 50),
                      offsetX: 50, offsetY: 50) && t.height == 0.2)
    let fast = { (dx: Double) in FlowShift(dx: 30 + dx, dy: 0, psr: 50) }
    r.check("HeightTracker skips frames at speed (mean shift > maxShift)",
            !t.update(left: fast(-0.1), right: fast(0.1), top: fast(0), bottom: fast(0),
                      offsetX: 50, offsetY: 50) && t.height == 0.2)
}
