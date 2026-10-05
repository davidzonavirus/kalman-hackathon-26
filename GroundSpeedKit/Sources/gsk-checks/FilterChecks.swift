import Foundation
import KalmanCore

/// Replays the default synthetic run; returns (summary, rows).
func replaySynthetic(_ filter: any GroundSpeedFilter, run: SyntheticRun) -> (ReplaySummary, [EstRow]) {
    var rows: [EstRow] = []
    let s = Replay.run(filter: filter, imu: run.imu, flow: run.flow, gnss: run.gnss, events: run.events,
                       zuptDetector: ZuptDetector(), onRow: { rows.append($0) })
    return (s, rows)
}

func filterChecks(_ r: inout CheckRunner) {
    r.section("KalmanCore")

    r.check("registry has ReferenceKF4 + DecoupledKF2x2",
            FilterRegistry.make(name: "ReferenceKF4") is ReferenceKF4
            && FilterRegistry.make(name: "DecoupledKF2x2") is DecoupledKF2x2
            && FilterRegistry.make(name: "nope") == nil
            && FilterRegistry.make(name: FilterRegistry.defaultName) != nil)

    // FilterConfig JSON keys
    let cfgJSON = (try? JSONEncoder().encode(FilterConfig.default)) ?? Data()
    r.check("FilterConfig snake_case keys",
            jsonKeys(cfgJSON) == ["q_accel", "q_bias", "r_flow_base", "psr_ref", "psr_min", "r_zupt", "gate", "p0_v", "p0_b", "gate_reset_count"])
    let partial = try? JSONDecoder().decode(FilterConfig.self, from: Data(#"{"gate":16}"#.utf8))
    r.check("FilterConfig partial JSON falls back to defaults", partial?.gate == 16 && partial?.qAccel == FilterConfig.default.qAccel)

    // Uninitialized behaviour
    let f0 = ReferenceKF4()
    r.check("updates before first predict are skipped",
            f0.updateFlow(t: 0, vx: 1, vy: 0, quality: 20) == .skipped(reason: "not initialized"))
    f0.predict(t: 5, ax: 1, ay: 0, r: 0)
    r.check("first predict initializes (no propagation)",
            f0.state.initialized && f0.state.t == 5 && f0.state.vx == 0 && f0.state.pDiag[0] == FilterConfig.default.p0V)

    // Synthetic 10 m run
    let run = SyntheticRun()
    let truthD = run.truth.last!.d
    r.near("synthetic truth distance = 10 m", truthD, 10.0, tol: 1e-9)

    let (s4, rows4) = replaySynthetic(ReferenceKF4(), run: run)
    r.near("ReferenceKF4 synthetic distance within 2%", s4.finalDistance, 10.0, tol: 0.2)
    print(String(format: "      ReferenceKF4: d=%.4f m, flow acc/gated/skipped=%d/%d/%d, zupts=%d, max σv=%.3f",
                 s4.finalDistance, s4.flowAccepted, s4.flowGated, s4.flowSkipped, s4.zuptCount, s4.maxSigmaV))
    r.check("injected flow outliers gated", s4.flowGated >= 2, "\(s4.flowGated)")
    r.check("lens-covered flow skipped", s4.flowSkipped >= 200, "\(s4.flowSkipped)")
    r.check("ZUPT detector fired at rest", s4.zuptCount > 100, "\(s4.zuptCount)")

    if let drop = run.dropoutWindow {
        func sigma(at t: Double) -> Double {
            rows4.min(by: { abs($0.t - t) < abs($1.t - t) })!.sigmaVx
        }
        let before = sigma(at: drop.lowerBound - 0.05)
        let atEnd = sigma(at: drop.upperBound - 0.01)
        let after = sigma(at: drop.upperBound + 0.5)
        r.check("σ_vx grows during dropout", atEnd > 3 * before, String(format: "%.4f -> %.4f", before, atEnd))
        r.check("σ_vx shrinks after flow returns", after < 0.3 * atEnd, String(format: "%.4f -> %.4f", atEnd, after))
        let maxErr = zip(rows4, run.truth).filter { drop.contains($0.0.t) }.map { abs($0.0.vx - $0.1.v) }.max() ?? 0
        r.check("velocity error during dropout < 0.15 m/s", maxErr < 0.15, String(format: "%.3f", maxErr))
    }
    let cruiseErr = zip(rows4, run.truth).filter { $0.1.v > 0.99 }.map { abs($0.0.vx - $0.1.v) }
    let rms = (cruiseErr.map { $0 * $0 }.reduce(0, +) / Double(max(1, cruiseErr.count))).squareRoot()
    r.check("cruise velocity RMS error < 0.05 m/s", rms < 0.05, String(format: "%.4f", rms))
    r.near("final velocity ≈ 0 (rest)", rows4.last!.vx, 0, tol: 0.02)

    let (s2, _) = replaySynthetic(DecoupledKF2x2(), run: run)
    r.near("DecoupledKF2x2 synthetic distance within 3%", s2.finalDistance, 10.0, tol: 0.3)
    print(String(format: "      DecoupledKF2x2: d=%.4f m", s2.finalDistance))

    // Determinism (replay must be reproducible bit-for-bit)
    let (s4b, rows4b) = replaySynthetic(ReferenceKF4(), run: SyntheticRun())
    r.check("replay deterministic", s4b == s4 && rows4b == rows4)

    // Gating: converge at 1 m/s then send a 10 m/s outlier.
    for (name, make) in [("ReferenceKF4", { ReferenceKF4() as any GroundSpeedFilter }),
                         ("DecoupledKF2x2", { DecoupledKF2x2() as any GroundSpeedFilter })] {
        let g = make()
        var t = 0.0
        g.predict(t: t, ax: 0, ay: 0, r: 0)
        for _ in 0..<300 {
            t += 0.01
            g.predict(t: t, ax: 0, ay: 0, r: 0)
            g.updateFlow(t: t, vx: 1.0, vy: 0, quality: 20)
        }
        let before = g.state
        let out = g.updateFlow(t: t, vx: 10, vy: 0, quality: 20)
        r.check("\(name) gates 10 m/s outlier", out.isGated && g.state == before, "\(out)")
        r.check("\(name) accepts consistent sample", g.updateFlow(t: t, vx: 1.02, vy: 0.01, quality: 20).isAccepted)
        r.check("\(name) skips low-PSR flow", g.updateFlow(t: t, vx: 0, vy: 0, quality: 1).isGated == false
                && g.updateFlow(t: t, vx: 0, vy: 0, quality: 1) == .skipped(reason: "low quality"))
    }

    // GNSS
    let gf = ReferenceKF4()
    gf.predict(t: 0, ax: 0, ay: 0, r: 0)
    r.check("GNSS skipped at low speed", gf.updateGNSS(t: 0, speed: 0.5, speedAccuracy: 0.3) == .skipped(reason: "speed < 1 m/s"))
    var tg = 0.0
    for _ in 0..<200 { tg += 0.01; gf.predict(t: tg, ax: 0, ay: 0, r: 0); gf.updateFlow(t: tg, vx: 2, vy: 0.1, quality: 20) }
    r.check("GNSS accepted when moving", gf.updateGNSS(t: tg, speed: 2.02, speedAccuracy: 0.2).isAccepted)

    // P symmetric, positive diagonal under aggressive random inputs (incl. yaw rate)
    let pf = ReferenceKF4()
    var rng = SeededRandom(seed: 7)
    var t = 0.0
    var symOK = true, posOK = true, psdOK = true
    pf.predict(t: 0, ax: 0, ay: 0, r: 0)
    for k in 0..<5000 {
        t += rng.uniform(0.002, 0.03)
        pf.predict(t: t, ax: rng.gaussian(2), ay: rng.gaussian(2), r: rng.gaussian(1.5))
        if k % 3 == 0 { pf.updateFlow(t: t, vx: rng.gaussian(1), vy: rng.gaussian(1), quality: rng.uniform(1, 40)) }
        if k % 50 == 0 { pf.updateZeroVelocity(t: t) }
        if k % 97 == 0 { pf.updateGNSS(t: t, speed: rng.uniform(0, 3), speedAccuracy: 0.3) }
        let P = pf.covariance
        for i in 0..<4 {
            if !(P[i][i] > 0) { posOK = false }
            for j in 0..<4 where P[i][j] != P[j][i] { symOK = false }
            for j in 0..<4 where i != j && P[i][j] * P[i][j] > P[i][i] * P[j][j] * (1 + 1e-9) { psdOK = false }
        }
    }
    r.check("P exactly symmetric throughout", symOK)
    r.check("P diagonal positive throughout", posOK)
    r.check("P 2×2 minors non-negative (|P_ij|² ≤ P_ii P_jj)", psdOK)

    // dt handling
    let df = ReferenceKF4()
    df.predict(t: 0, ax: 0, ay: 0, r: 0)
    df.predict(t: 2.0, ax: 1, ay: 0, r: 0)
    r.near("dt clamped to 0.1 s", df.state.vx, 0.1, tol: 1e-12)
    df.predict(t: 1.0, ax: 1, ay: 0, r: 0)
    r.check("non-positive dt ignored", df.state.t == 2.0 && abs(df.state.vx - 0.1) < 1e-12)

    // Yaw coupling: pure rotation of the velocity vector, no noise.
    let yf = ReferenceKF4(config: FilterConfig(qAccel: 0, qBias: 0))
    yf.predict(t: 0, ax: 0, ay: 0, r: 0)
    for _ in 0..<50 { yf.updateFlow(t: 0, vx: 1, vy: 0, quality: 1000) }
    let v0 = yf.state.speed
    var ty = 0.0
    for _ in 0..<100 { ty += 0.001; yf.predict(t: ty, ax: 0, ay: 0, r: 0.5) }
    r.check("yaw term rotates velocity (vy < 0 for r > 0, |v| ≈ const)",
            yf.state.vy < 0 && abs(yf.state.speed - v0) < 1e-3 * v0,
            String(format: "vy=%.5f |v|=%.5f->%.5f", yf.state.vy, v0, yf.state.speed))

    // DistanceIntegrator
    var dp = DistanceIntegrator(mode: .pathLength)
    for k in 0...500 { dp.add(t: Double(k) * 0.01, vx: 0.6, vy: 0.8) }
    r.near("DistanceIntegrator pathLength 1 m/s × 5 s = 5 m", dp.distance, 5.0, tol: 1e-9)
    var di = DistanceIntegrator(mode: .forward)
    for k in 0...500 { di.add(t: Double(k) * 0.01, vx: 1.0, vy: 0.8) }
    r.near("DistanceIntegrator forward integrates v_x only", di.distance, 5.0, tol: 1e-9)
    var dd = DistanceIntegrator(deadband: 0.05)
    for k in 0...1000 { dd.add(t: Double(k) * 0.01, vx: 0.03, vy: -0.02) }
    r.check("DistanceIntegrator deadband: 3.6 cm/s creep for 10 s adds 0 [\(dd.distance)]", dd.distance == 0)
    var dl = DistanceIntegrator(deadband: 0.05, smoothing: 0.5)
    for k in 0...300 { dl.add(t: Double(k) * 0.01, vx: 1.0, vy: 0) }
    r.check("DistanceIntegrator lead compensation: 1 m/s × 3 s reads ≈ 3 m mid-motion [plain \(dl.distance), lead \(dl.leadCompensated)]",
            abs(dl.leadCompensated - 3.0) < 0.03 && dl.distance < 2.6)
    var dj = DistanceIntegrator(mode: .forward)
    for k in 0...1000 { let t = Double(k) * 0.01; dj.add(t: t, vx: 0.4 * sin(2 * .pi * 3 * t), vy: 0.3 * cos(2 * .pi * 5 * t)) }
    r.check("DistanceIntegrator forward: jostling (±0.4 m/s, 3 Hz) nets ≈ 0 m [\(dj.distance)]", abs(dj.distance) < 0.01)
    var wobRaw = DistanceIntegrator(deadband: 0.05)
    var wobLP = DistanceIntegrator(deadband: 0.05, smoothing: 0.5)
    for k in 0...1000 {
        let t = Double(k) * 0.01
        let (x, y) = (0.4 * sin(2 * .pi * 3 * t), 0.3 * cos(2 * .pi * 5 * t))
        wobRaw.add(t: t, vx: x, vy: y); wobLP.add(t: t, vx: x, vy: y)
    }
    r.check("DistanceIntegrator smoothing: 10 s of wobble (±0.4 m/s, 3 Hz) adds < 0.15 m [raw \(wobRaw.distance), smoothed \(wobLP.distance)]",
            wobLP.distance < 0.15)
    var steadyLP = DistanceIntegrator(deadband: 0.05, smoothing: 0.5)
    for k in 0...700 { let t = Double(k) * 0.01; steadyLP.add(t: t, vx: t <= 5 ? 1 : 0, vy: 0) }
    r.near("DistanceIntegrator smoothing: 5 m push then 2 s still reads 5 m (deadband trims < 1% of the tail)", steadyLP.distance, 5.0, tol: 0.04)
    di = DistanceIntegrator()
    for k in 0...500 { di.add(t: Double(k) * 0.01, vx: 1, vy: 0) }
    di.reset()
    di.add(t: 5.01, vx: 1, vy: 0)
    r.near("DistanceIntegrator reset keeps integrating", di.distance, 0.01, tol: 1e-9)

    // ZuptDetector
    var zd = ZuptDetector()
    var zr = SeededRandom(seed: 3)
    var still = false
    for k in 0..<100 { still = zd.update(t: Double(k) * 0.01, ax: zr.gaussian(0.02), ay: zr.gaussian(0.02), az: zr.gaussian(0.02), flowSpeed: 0.005) }
    r.check("ZuptDetector: still -> true", still)
    var moving = false
    for k in 100..<200 { moving = zd.update(t: Double(k) * 0.01, ax: zr.gaussian(0.15), ay: zr.gaussian(0.15), az: zr.gaussian(0.25), flowSpeed: 1.0) || moving }
    r.check("ZuptDetector: vibrating & moving -> false", !moving)
    var slowRoll = false
    zd.reset()
    for k in 0..<100 { slowRoll = zd.update(t: Double(k) * 0.01, ax: zr.gaussian(0.02), ay: zr.gaussian(0.02), az: zr.gaussian(0.02), flowSpeed: 0.3) || slowRoll }
    r.check("ZuptDetector: quiet IMU but flow 0.3 m/s -> false", !slowRoll)
}
