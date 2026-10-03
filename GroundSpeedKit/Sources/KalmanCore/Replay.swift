import Foundation

/// In-memory sensor samples (same columns as the run CSVs, see PROTOCOL.md §3).
public struct ImuSample: Sendable, Equatable {
    public var t, ax, ay, az, gx, gy, gz: Double
    public init(t: Double, ax: Double, ay: Double, az: Double, gx: Double, gy: Double, gz: Double) {
        self.t = t; self.ax = ax; self.ay = ay; self.az = az; self.gx = gx; self.gy = gy; self.gz = gz
    }
}

public struct FlowSample: Sendable, Equatable {
    public var t, vx, vy, quality, h: Double
    public init(t: Double, vx: Double, vy: Double, quality: Double, h: Double) {
        self.t = t; self.vx = vx; self.vy = vy; self.quality = quality; self.h = h
    }
}

public struct GnssSample: Sendable, Equatable {
    public var t, speed, speedAcc, course: Double
    public init(t: Double, speed: Double, speedAcc: Double, course: Double) {
        self.t = t; self.speed = speed; self.speedAcc = speedAcc; self.course = course
    }
}

public struct RunEvent: Sendable, Equatable {
    public var t: Double
    public var event: String
    public var value: String
    public init(t: Double, event: String, value: String = "") {
        self.t = t; self.event = event; self.value = value
    }
}

/// One est.csv row (status kept as booleans so KalmanCore needn't know the bit layout;
/// `kfreplay` maps them onto `StatusFlags`).
public struct EstRow: Sendable, Equatable {
    public var t, vx, vy, sigmaVx, sigmaVy, distance: Double
    public var flowQuality, h: Double
    public var initialized: Bool
    public var flowOK: Bool        // last flow accepted and < 100 ms old
    public var flowGated: Bool     // last flow update was gated
    public var gnssOK: Bool        // GNSS sample < 2 s old with speed_acc ≥ 0
    public var gnssGated: Bool     // last GNSS update was gated
    public var zupt: Bool          // ZUPT applied in the last 100 ms
}

public struct ReplaySummary: Sendable, Equatable {
    public var rows: Int
    public var finalDistance: Double
    public var maxSigmaV: Double
    public var flowAccepted = 0, flowGated = 0, flowSkipped = 0
    public var gnssAccepted = 0, gnssGated = 0, gnssSkipped = 0
    public var zuptCount = 0
}

/// Deterministic log replay through any `GroundSpeedFilter`.
///
/// Merge rule: all samples sorted by (t, kind) with kind order IMU(predict) < flow < GNSS <
/// ZUPT, stable within a kind. After each predict the `DistanceIntegrator` is advanced and
/// one `EstRow` is emitted immediately (post-predict state, before any update at the same t —
/// this matches the phone's `SensorFusionEngine`). ZUPT sources: `events` rows with
/// event == "zupt" (the app logs every ZUPT it applies), and, if `zuptDetector` != nil, the
/// detector evaluated on every IMU sample (fed flow speeds of samples with quality ≥ psr_min).
/// A ZUPT at time t is applied after any flow/GNSS sample with the same t; several ZUPT
/// sources at the same t apply once.
public enum Replay {
    public static func run(filter: any GroundSpeedFilter,
                           imu: [ImuSample], flow: [FlowSample], gnss: [GnssSample],
                           events: [RunEvent] = [],
                           zuptDetector: ZuptDetector? = nil,
                           onRow: ((EstRow) -> Void)? = nil) -> ReplaySummary {
        enum Kind: Int { case imu = 0, flow = 1, gnss = 2, zupt = 3 }
        struct Ev { var t: Double; var kind: Kind; var idx: Int }
        var evs: [Ev] = []
        evs.reserveCapacity(imu.count + flow.count + gnss.count + events.count)
        for (i, s) in imu.enumerated() { evs.append(Ev(t: s.t, kind: .imu, idx: i)) }
        for (i, s) in flow.enumerated() { evs.append(Ev(t: s.t, kind: .flow, idx: i)) }
        for (i, s) in gnss.enumerated() { evs.append(Ev(t: s.t, kind: .gnss, idx: i)) }
        for (i, e) in events.enumerated() where e.event == "zupt" { evs.append(Ev(t: e.t, kind: .zupt, idx: i)) }
        evs.sort { a, b in
            if a.t != b.t { return a.t < b.t }
            if a.kind != b.kind { return a.kind.rawValue < b.kind.rawValue }
            return a.idx < b.idx
        }

        var detector = zuptDetector
        var dist = DistanceIntegrator()
        var summary = ReplaySummary(rows: 0, finalDistance: 0, maxSigmaV: 0)
        var lastFlowQ = 0.0, lastH = 0.0
        var lastFlowAcceptT = -Double.infinity, lastFlowGated = false
        var lastGnssT = -Double.infinity, lastGnssGated = false
        var lastZuptT = -Double.infinity
        var pendingT = -Double.infinity     // timestamp of the group being processed
        var pendingZupt = false

        func flushZupt() {
            guard pendingZupt else { return }
            filter.updateZeroVelocity(t: pendingT)
            lastZuptT = pendingT
            summary.zuptCount += 1
            pendingZupt = false
        }

        func emitRow() {
            let s = filter.state
            let row = EstRow(t: s.t, vx: s.vx, vy: s.vy, sigmaVx: s.sigmaVx, sigmaVy: s.sigmaVy,
                             distance: dist.distance, flowQuality: lastFlowQ, h: lastH,
                             initialized: s.initialized,
                             flowOK: !lastFlowGated && s.t - lastFlowAcceptT < 0.1,
                             flowGated: lastFlowGated,
                             gnssOK: s.t - lastGnssT < 2, gnssGated: lastGnssGated,
                             zupt: s.t - lastZuptT < 0.1)
            summary.rows += 1
            summary.maxSigmaV = max(summary.maxSigmaV, s.sigmaVx, s.sigmaVy)
            onRow?(row)
        }

        for ev in evs {
            if ev.t > pendingT { flushZupt(); pendingT = ev.t }
            switch ev.kind {
            case .imu:
                let s = imu[ev.idx]
                filter.predict(t: s.t, ax: s.ax, ay: s.ay, r: s.gz)
                dist.add(filter.state)
                emitRow()
                if detector?.addIMU(t: s.t, ax: s.ax, ay: s.ay, az: s.az) == true { pendingZupt = true }
            case .flow:
                let s = flow[ev.idx]
                lastFlowQ = s.quality; lastH = s.h
                if s.quality >= filter.config.psrMin {
                    detector?.addFlow(t: s.t, speed: (s.vx * s.vx + s.vy * s.vy).squareRoot())
                }
                switch filter.updateFlow(t: s.t, vx: s.vx, vy: s.vy, quality: s.quality) {
                case .accepted: summary.flowAccepted += 1; lastFlowAcceptT = s.t; lastFlowGated = false
                case .gated: summary.flowGated += 1; lastFlowGated = true
                case .skipped: summary.flowSkipped += 1
                }
            case .gnss:
                let s = gnss[ev.idx]
                if s.speedAcc >= 0 { lastGnssT = s.t }
                switch filter.updateGNSS(t: s.t, speed: s.speed, speedAccuracy: s.speedAcc) {
                case .accepted: summary.gnssAccepted += 1; lastGnssGated = false
                case .gated: summary.gnssGated += 1; lastGnssGated = true
                case .skipped: summary.gnssSkipped += 1
                }
            case .zupt:
                pendingZupt = true
            }
        }
        flushZupt()
        summary.finalDistance = dist.distance
        return summary
    }
}
