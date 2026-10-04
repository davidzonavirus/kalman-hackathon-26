import Foundation
import KalmanCore
import SpeedProtocol

/// What the UI / telemetry sees after every filter step.
public struct FusionSnapshot: Sendable, Equatable {
    public var t: Double = 0
    public var vx: Double = 0
    public var vy: Double = 0
    public var sigmaVx: Double = 0
    public var sigmaVy: Double = 0
    public var bx: Double = 0
    public var by: Double = 0
    /// Total distance travelled (∫|v|, deadband) since zero / run start, m.
    public var distance: Double = 0
    /// Net signed displacement along vehicle x (∫v_x) since zero / run start, m.
    public var netForward: Double = 0
    public var flowQuality: Double = 0
    /// Camera height currently used (LiDAR if fresh, else manual), m.
    public var h: Double = 0
    public var status: StatusFlags = []
    public var filterName: String = ""
    /// Last raw flow velocity (vehicle frame), for the orientation check.
    public var flowVx: Double = 0
    public var flowVy: Double = 0
    /// Last IMU sample (vehicle frame, calibrated), for the orientation check.
    public var imuAx: Double = 0
    public var imuAy: Double = 0
    public var imuGz: Double = 0
    public var gnssSpeed: Double = -1
    public var gnssSpeedAcc: Double = -1
    public var lidarH: Double = 0
    /// Measured input rates (Hz), smoothed.
    public var imuRateHz: Double = 0
    public var flowRateHz: Double = 0
    public var zuptCount: Int = 0
    public var flowGatedCount: Int = 0

    public init() {}

    public var speed: Double { (vx * vx + vy * vy).squareRoot() }
}

/// Owns the filter, distance integrator and ZUPT detector on its own serial queue.
///
/// All `ingest*` methods are thread-safe and non-blocking. Samples are put in a short
/// re-order buffer (default 50 ms, driven by IMU time) and processed strictly in timestamp
/// order with the replay tie-break rule (IMU, flow, GNSS at equal t), so that replaying the
/// logged CSVs reproduces `est.csv`. Every input is quantised to its CSV text representation
/// before the filter sees it, so the log is bit-exact with what the filter consumed.
///
/// Per IMU sample (= one filter step):
///   predict(t, ax, ay, r: gz) → distance.add(state) → est.csv row → [ZUPT update + "zupt" event]
/// The first IMU sample after launch / a filter change calls `reset(t:)` instead of predict
/// (identical to a fresh filter's first predict). Starting a run does NOT touch the filter
/// (the estimate stays continuous); it only restarts the distance integrator at the run's
/// first IMU sample and logs the filter state there as a `filter_state` event.
public final class SensorFusionEngine: @unchecked Sendable {
    // MARK: Configuration (set before / while running; thread-safe through the queue)

    public struct Settings: Sendable, Equatable {
        /// Extra PSR floor for FLOW_OK / ZUPT flow speed. The effective threshold is
        /// max(this, filter.config.psrMin), so by default it follows the filter's psr_min
        /// (the correlator's noise floor is PSR ≈ 7; covered lens must not read FLOW_OK).
        public var flowQualityThreshold: Double = 0
        /// Manual camera height (m) used when LiDAR is not fresh.
        public var manualHeight: Double = 0.30
        /// Reorder latency (s).
        public var reorderLatency: Double = 0.05
        public var zuptEnabled: Bool = true
        /// When flow is stale, ZUPT additionally requires |v̂| below this (m/s)...
        public var zuptMaxSpeedWithoutFlow: Double = 0.1
        /// ...until flow has been gone this long (s). After that a still IMU always zeroes v̂:
        /// IMU-only velocity drifts without bound (seen: 0.9 m/s after 30 s, 5.8 m/s after
        /// 4 min with the phone at rest), so it must not block its own correction.
        public var zuptAnySpeedAfterFlowLoss: Double = 2.0
        /// Duration of the calibrate command's averaging window (s).
        public var calibrationDuration: Double = 2.0
        /// Remove rotation-induced image motion from flow using the gyro:
        ///   v_x ← v_x − s·h·ω_y,  v_y ← v_y + s·h·ω_x   (vehicle frame, ω averaged over
        /// the flow interval). Tilting/jostling the phone sweeps the image across the floor
        /// at h·ω without any translation; without this it is counted as speed. Sign s = +1
        /// was fitted on device (flow vs gyro corr 0.89); flip it if a mount mapping change
        /// makes jostling read worse. flow.csv logs the CORRECTED velocity (what the filter used).
        public var flowDerotation: Bool = true
        public var flowDerotationSign: Double = 1
        /// Fraction of h·|ω| left after de-rotation, added as flow noise (0 = off).
        public var wobbleResidual: Double = 0.5
        public init() {}
    }

    public let queue = DispatchQueue(label: "gsk.engine", qos: .userInteractive)

    /// Receives raw samples and estimates. May be swapped at any time.
    public var recorder: RunRecorder? {
        get { lock.withLock { _recorder } }
        set { lock.withLock { _recorder = newValue } }
    }

    /// Called on the engine queue after every filter step (IMU rate).
    public var onSnapshot: (@Sendable (FusionSnapshot) -> Void)?

    /// Wall clock used for staleness checks of the published snapshot.
    public var timeSource: @Sendable () -> Double = { Clock.now() }

    // MARK: Private state (engine queue only unless noted)

    private enum Kind: Int { case imu = 0, flow = 1, gnss = 2, depth = 3 }
    private struct Sample {
        var kind: Kind
        var t: Double
        var a: Double, b: Double, c: Double, d: Double, e: Double, f: Double
    }

    private var pending: [Sample] = []
    private var filter: any GroundSpeedFilter
    /// Speeds below this don't add to the total (filter noise at rest must not creep).
    public static let distanceDeadband = 0.05
    /// Low-pass τ (s) on velocity before ∫|v|, so phone wobble doesn't add distance.
    public static let distanceSmoothing = 0.5
    private var distance = DistanceIntegrator(deadband: SensorFusionEngine.distanceDeadband,
                                              smoothing: SensorFusionEngine.distanceSmoothing)
    private var netForward = DistanceIntegrator(mode: .forward)
    private var zupt = ZuptDetector()
    private var settings = Settings()
    private var needsReset = true
    private var runStartPending = false
    private var lastProcessedT = -Double.infinity
    private var maxIMUT = -Double.infinity
    private var lastIMUArrivalWall = -Double.infinity

    private var lastIMUT = -Double.infinity
    private var lastFlowT = -Double.infinity
    /// Gyro ω_x, ω_y accumulated since the previous flow sample (for de-rotation).
    private var gyroSum = (x: 0.0, y: 0.0, n: 0)
    private var lastGyro = (x: 0.0, y: 0.0)
    private var lastFlowGoodT = -Double.infinity
    private var lastFlowQuality = 0.0
    /// Height the flow source used (tracks ride height between LiDAR fixes).
    private var lastFlowH = 0.0
    private var lastFlowAccepted = false
    private var lastFlowGated = false
    private var lastGNSST = -Double.infinity
    private var lastGNSSAcc = -1.0
    private var lastGNSSGated = false
    private var lastDepthT = -Double.infinity
    private var lidarH = 0.0
    private var lastZuptT = -Double.infinity
    private var pendingZuptT: Double?
    private var torchOn = false
    private var holdStill = false
    private var imuRate = RateMeter()
    private var flowRate = RateMeter()

    // Calibration (IMU bias removal in the vehicle frame).
    private var calibrating = false
    private var calibEnd = 0.0
    private var calibSum = [Double](repeating: 0, count: 6)
    private var calibN = 0
    private var bias = [Double](repeating: 0, count: 6)   // ax, ay, az, gx, gy, gz
    private var calibSumSq = [Double](repeating: 0, count: 6)
    /// Max accel std (m/s²) over the calibration window; above this the phone was moving.
    /// Measured: phone resting still ~0.01, phone being handled ~0.07.
    public static let calibrationMaxAccStd = 0.04

    private var snap = FusionSnapshot()

    // Cross-thread state.
    private let lock = NSLock()
    private var _recorder: RunRecorder?
    private var _latest = FusionSnapshot()
    private var _latestHeight = 0.30
    private var _arrivals = (imu: -Double.infinity, flow: -Double.infinity, gnss: -Double.infinity, depth: -Double.infinity)
    private var _recording = false

    public init(filterName: String = FilterRegistry.defaultName, config: FilterConfig = .default,
                settings: Settings = Settings()) {
        self.filter = FilterRegistry.make(name: filterName, config: config)
            ?? ReferenceKF4(config: config)
        self.settings = settings
        self._latestHeight = settings.manualHeight
        self.snap.filterName = filter.filterName
        self.snap.h = settings.manualHeight
        self._latest = snap
    }

    // MARK: - Inputs (any thread)

    /// IMU sample, vehicle frame, gravity removed (m/s², rad/s). `gz` is used as yaw rate r.
    public func ingestIMU(t: Double, ax: Double, ay: Double, az: Double, gx: Double, gy: Double, gz: Double) {
        markArrival(.imu)
        enqueue(Sample(kind: .imu, t: t, a: ax, b: ay, c: az, d: gx, e: gy, f: gz))
    }

    /// Optical-flow ground velocity, vehicle frame (m/s); quality = PSR; h = height used (m).
    public func ingestFlow(t: Double, vx: Double, vy: Double, quality: Double, h: Double) {
        markArrival(.flow)
        enqueue(Sample(kind: .flow, t: t, a: vx, b: vy, c: quality, d: h, e: 0, f: 0))
    }

    /// GNSS speed (m/s), 1σ speed accuracy (m/s, < 0 invalid), course (deg, < 0 invalid).
    public func ingestGNSS(t: Double, speed: Double, speedAcc: Double, course: Double) {
        markArrival(.gnss)
        enqueue(Sample(kind: .gnss, t: t, a: speed, b: speedAcc, c: course, d: 0, e: 0, f: 0))
    }

    /// LiDAR height of the camera above the floor (m).
    public func ingestDepth(t: Double, h: Double) {
        markArrival(.depth)
        enqueue(Sample(kind: .depth, t: t, a: h, b: 0, c: 0, d: 0, e: 0, f: 0))
    }

    // MARK: - Control (any thread)

    /// Start-of-run: distance restarts from 0 at the next IMU sample. The filter keeps running
    /// (no velocity discontinuity). Replaying the run from a fresh filter therefore matches
    /// est.csv exactly only if the filter was fresh at run start (e.g. first run after launch
    /// or a filter change); otherwise it converges within the first flow updates. The state
    /// at run start is logged (`filter_state` event) for seeding a replay.
    public func beginRun() {
        queue.async { self.runStartPending = true }
    }

    public func setRecording(_ on: Bool) {
        lock.withLock { _recording = on }
    }

    public func resetDistance() {
        queue.async {
            self.distance.reset()
            self.netForward.reset()
            self.snap.distance = 0
            self.snap.netForward = 0
            self.lock.withLock { self._latest.distance = 0; self._latest.netForward = 0 }
            self.recorder?.recordEvent(t: self.eventTime(), event: CSVSchema.Event.resetDistance)
        }
    }

    /// Averages raw IMU for `settings.calibrationDuration` s (cart must be still) and removes
    /// that bias from subsequent samples (the logged imu.csv is bias-corrected, so replay
    /// needs no knowledge of it). The old bias stays applied during the window and a still
    /// IMU gets ZUPTs, so zeroing never lets velocity drift. The filter is not reset (that
    /// would break replay).
    public func calibrate() {
        queue.async {
            self.calibrating = true
            self.calibEnd = self.lastIMUT + self.settings.calibrationDuration
            if !self.calibEnd.isFinite { self.calibEnd = -Double.infinity }
            self.calibSum = [Double](repeating: 0, count: 6)
            self.calibSumSq = [Double](repeating: 0, count: 6)
            self.calibN = 0
            self.recorder?.recordEvent(t: self.eventTime(), event: CSVSchema.Event.calibrate, value: "begin")
        }
    }

    public func clearCalibration() {
        queue.async {
            self.calibrating = false
            self.bias = [Double](repeating: 0, count: 6)
        }
    }

    /// While on, every IMU step applies a ZUPT (logged as usual, so replay matches). Used
    /// while the flow camera is paused for a LiDAR height measurement: the cart is still by
    /// definition, and IMU-only dead-reckoning would otherwise drift for seconds.
    public func setHoldStill(_ on: Bool) {
        queue.async {
            self.holdStill = on
            self.recorder?.recordEvent(t: self.eventTime(), event: "hold_still", value: on ? "on" : "off")
        }
    }

    public func setTorchOn(_ on: Bool) {
        queue.async { self.torchOn = on }
    }

    /// Records a free-form event at the current engine time.
    public func recordEvent(_ event: String, value: String = "") {
        queue.async { self.recorder?.recordEvent(t: self.eventTime(), event: event, value: value) }
    }

    /// Swap the filter variant / config. Takes effect with a reset at the next IMU sample.
    public func setFilter(name: String, config: FilterConfig) {
        queue.async {
            if let f = FilterRegistry.make(name: name, config: config) {
                self.filter = f
            } else {
                self.filter.config = config
            }
            self.snap.filterName = self.filter.filterName
            self.needsReset = true
        }
    }

    public func updateSettings(_ s: Settings) {
        queue.async {
            self.settings = s
            self.lock.withLock {
                if !(self.lastDepthFresh(at: self.lastProcessedT)) { self._latestHeight = s.manualHeight }
            }
        }
    }

    public var currentFilterName: String { queue.sync { filter.filterName } }
    public var currentFilterConfig: FilterConfig { queue.sync { filter.config } }
    public var currentSettings: Settings { queue.sync { settings } }
    public var currentBias: [Double] { queue.sync { bias } }

    /// Height the camera pipeline should use right now (LiDAR if fresh, else manual).
    public var currentHeight: Double { lock.withLock { _latestHeight } }

    /// Latest snapshot, with live-ness flags downgraded if a sensor stopped arriving
    /// (the event-time flags can't notice a sensor that went silent).
    public func snapshot() -> FusionSnapshot {
        let now = timeSource()
        return lock.withLock {
            var s = _latest
            if now - _arrivals.imu > 0.25 { s.status.remove(.imuOK) }
            if now - _arrivals.flow > 0.25 { s.status.remove(.flowOK) }
            if now - _arrivals.gnss > 2.5 { s.status.remove(.gnssOK) }
            if now - _arrivals.depth > 0.75 { s.status.remove(.lidarOK) }
            if _recording { s.status.insert(.recording) } else { s.status.remove(.recording) }
            return s
        }
    }

    /// Process everything still in the reorder buffer (e.g. before stopping a run).
    public func drain() {
        queue.sync {
            self.process(upTo: .infinity)
            self.applyPendingZupt(before: .infinity)
        }
    }

    // MARK: - Internals

    private func markArrival(_ k: Kind) {
        let now = timeSource()
        lock.withLock {
            switch k {
            case .imu: _arrivals.imu = now
            case .flow: _arrivals.flow = now
            case .gnss: _arrivals.gnss = now
            case .depth: _arrivals.depth = now
            }
        }
    }

    private func eventTime() -> Double {
        lastProcessedT.isFinite ? lastProcessedT : timeSource()
    }

    private func enqueue(_ s: Sample) {
        queue.async {
            var s = s
            guard s.t.isFinite else { return }
            // Too late for the reorder window: re-stamp so the log stays monotone and replayable.
            if s.t < self.lastProcessedT { s.t = self.lastProcessedT }
            // Insert keeping (t, kind) order; most samples go at the end.
            var i = self.pending.count
            while i > 0 {
                let p = self.pending[i - 1]
                if p.t < s.t || (p.t == s.t && p.kind.rawValue <= s.kind.rawValue) { break }
                i -= 1
            }
            self.pending.insert(s, at: i)
            let wall = self.timeSource()
            if s.kind == .imu {
                self.maxIMUT = max(self.maxIMUT, s.t)
                self.lastIMUArrivalWall = wall
            }
            // Without IMU there is nothing to drive the watermark: process immediately.
            let imuAlive = wall - self.lastIMUArrivalWall < 0.2
            let watermark = imuAlive ? self.maxIMUT - self.settings.reorderLatency : .infinity
            self.process(upTo: watermark)
        }
    }

    private func process(upTo watermark: Double) {
        var n = 0
        while n < pending.count, pending[n].t <= watermark {
            handle(pending[n])
            n += 1
        }
        if n > 0 { pending.removeFirst(n) }
    }

    @inline(__always) private func qt(_ t: Double) -> Double { Double(CSVRow.time(t)) ?? t }
    @inline(__always) private func qv(_ v: Double) -> Double { Double(CSVRow.real(v)) ?? v }

    private func applyPendingZupt(before t: Double) {
        guard let zt = pendingZuptT, t > zt else { return }
        pendingZuptT = nil
        filter.updateZeroVelocity(t: zt)
        lastZuptT = zt
        snap.zuptCount += 1
        recorder?.recordEvent(t: zt, event: CSVSchema.Event.zupt)
    }

    private func handle(_ s: Sample) {
        let t = qt(s.t)
        applyPendingZupt(before: t)
        lastProcessedT = max(lastProcessedT, t)
        let rec = recorder
        switch s.kind {
        case .imu:
            var raw = [s.a, s.b, s.c, s.d, s.e, s.f]
            if calibrating {
                if !calibEnd.isFinite { calibEnd = t + settings.calibrationDuration }
                for k in 0..<6 { calibSum[k] += raw[k]; calibSumSq[k] += raw[k] * raw[k] }
                calibN += 1
                if t >= calibEnd, calibN > 0 {
                    let n = Double(calibN)
                    let mean = calibSum.map { $0 / n }
                    let accStd = (0..<3).map { max(0, calibSumSq[$0] / n - mean[$0] * mean[$0]).squareRoot() }.max() ?? 0
                    calibrating = false
                    if accStd <= Self.calibrationMaxAccStd {
                        bias = mean
                        rec?.recordEvent(t: t, event: CSVSchema.Event.calibrate,
                                         value: "end bias_ax=\(CSVRow.real(bias[0])) bias_ay=\(CSVRow.real(bias[1])) bias_gz=\(CSVRow.real(bias[5]))")
                    } else {
                        rec?.recordEvent(t: t, event: CSVSchema.Event.calibrate,
                                         value: "rejected acc_std=\(CSVRow.real(accStd)) (moving) kept previous bias")
                    }
                }
            }
            for k in 0..<6 { raw[k] = qv(raw[k] - bias[k]) }
            let (ax, ay, az, gx, gy, gz) = (raw[0], raw[1], raw[2], raw[3], raw[4], raw[5])
            rec?.recordIMU(t: t, ax: ax, ay: ay, az: az, gx: gx, gy: gy, gz: gz)
            imuRate.tick(t)
            gyroSum.x += gx; gyroSum.y += gy; gyroSum.n += 1
            lastGyro = (gx, gy)

            if needsReset {
                filter.reset(t: t)
                distance.clear()
                netForward.clear()
                zupt.reset()
                pendingZuptT = nil
                needsReset = false
            } else {
                filter.predict(t: t, ax: ax, ay: ay, r: gz)
            }
            let st = filter.state
            if runStartPending {
                runStartPending = false
                distance.clear()
                netForward.clear()
                snap.distance = 0
                snap.netForward = 0
                let p = st.pDiag.map { CSVRow.real($0) }.joined(separator: " ")
                rec?.recordEvent(t: t, event: "filter_state",
                                 value: "vx=\(CSVRow.real(st.vx)) vy=\(CSVRow.real(st.vy)) bx=\(CSVRow.real(st.bx)) by=\(CSVRow.real(st.by)) p=\(p)")
            }
            distance.add(st)
            netForward.add(st)
            lastIMUT = t
            snap.imuAx = ax; snap.imuAy = ay; snap.imuGz = gz

            // ZUPT decision uses this sample; applied after the est row (replay order).
            let still = zupt.addIMU(t: t, ax: ax, ay: ay, az: az)
            let flowAge = t - lastFlowGoodT
            let flowFresh = flowAge <= zupt.config.flowMaxAge
            let applyZupt = holdStill || (settings.zuptEnabled && still
                && (calibrating || flowFresh || st.speed < settings.zuptMaxSpeedWithoutFlow
                    || flowAge > settings.zuptAnySpeedAfterFlowLoss))

            publish(t: t, state: st, recorder: rec)

            // Applied once every other sample with the same timestamp has been processed
            // (replay order: IMU < flow < GNSS < ZUPT at equal t).
            if applyZupt { pendingZuptT = t }

        case .flow:
            var vx = s.a, vy = s.b
            let rawQ = qv(s.c), h = qv(s.d)
            let w = gyroSum.n > 0
                ? (x: gyroSum.x / Double(gyroSum.n), y: gyroSum.y / Double(gyroSum.n))
                : lastGyro
            if settings.flowDerotation {
                let k = settings.flowDerotationSign * h
                vx -= k * w.y
                vy += k * w.x
            }
            gyroSum = (0, 0, 0)
            vx = qv(vx); vy = qv(vy)
            let q = qv(wobbleQuality(rawQ, h: h, omega: (w.x * w.x + w.y * w.y).squareRoot()))
            rec?.recordFlow(t: t, vx: vx, vy: vy, quality: q, h: h)
            flowRate.tick(t)
            lastFlowT = t
            lastFlowQuality = rawQ
            if h.isFinite, h > 0 { lastFlowH = h }
            snap.flowVx = vx; snap.flowVy = vy
            if q >= flowThreshold {
                lastFlowGoodT = t
                zupt.addFlow(t: t, speed: (vx * vx + vy * vy).squareRoot())
            }
            let outcome = filter.updateFlow(t: t, vx: vx, vy: vy, quality: q)
            lastFlowAccepted = outcome.isAccepted
            lastFlowGated = outcome.isGated
            if outcome.isGated { snap.flowGatedCount += 1 }

        case .gnss:
            let speed = qv(s.a), acc = qv(s.b), course = qv(s.c)
            rec?.recordGNSS(t: t, speed: speed, speedAcc: acc, courseDeg: course)
            snap.gnssSpeed = speed
            snap.gnssSpeedAcc = acc
            if acc >= 0 { lastGNSST = t; lastGNSSAcc = acc }
            let outcome = filter.updateGNSS(t: t, speed: speed, speedAccuracy: acc)
            lastGNSSGated = outcome.isGated

        case .depth:
            let h = qv(s.a)
            rec?.recordDepth(t: t, h: h)
            if h.isFinite, h > 0.02, h < 5 {
                lastDepthT = t
                lidarH = h
                snap.lidarH = h
                lock.withLock { _latestHeight = h }
            }
        }
        if s.kind != .depth, !lastDepthFresh(at: t) {
            let m = settings.manualHeight
            lock.withLock { _latestHeight = m }
        }
    }

    /// Lowers the flow quality the filter sees while the phone rotates. De-rotation leaves
    /// ≈ `wobbleResidual`·h·|ω| of real lens swing (the lens isn't on the rotation axis;
    /// fitted 0.1–0.6 on device), so that variance is added to the flow noise:
    /// R' = R(q) + (c·h·|ω|)², returned as the quality q' with R(q') = R'. flow.csv logs q'.
    private func wobbleQuality(_ q: Double, h: Double, omega: Double) -> Double {
        let c = settings.wobbleResidual
        guard c > 0, omega > 0, q > 0 else { return q }
        let cfg = filter.config
        let r = cfg.flowVariance(quality: q) + (c * h * omega) * (c * h * omega)
        return cfg.psrRef * (cfg.rFlowBase / r).squareRoot()
    }

    /// Effective PSR threshold for FLOW_OK and ZUPT flow confirmation.
    private var flowThreshold: Double { max(settings.flowQualityThreshold, filter.config.psrMin) }

    private func lastDepthFresh(at t: Double) -> Bool { t - lastDepthT < 0.5 }

    private func statusFlags(at t: Double) -> StatusFlags {
        var f: StatusFlags = []
        if t - lastIMUT < 0.05 { f.insert(.imuOK) }
        if t - lastFlowT < 0.1, lastFlowQuality >= flowThreshold, lastFlowAccepted { f.insert(.flowOK) }
        if t - lastGNSST < 2.0, lastGNSSAcc >= 0 { f.insert(.gnssOK) }
        if lastDepthFresh(at: t) { f.insert(.lidarOK) }
        if lock.withLock({ _recording }) { f.insert(.recording) }
        if t - lastZuptT < 0.1 { f.insert(.zupt) }
        if lastFlowGated { f.insert(.flowGated) }
        if lastGNSSGated { f.insert(.gnssGated) }
        if torchOn { f.insert(.torchOn) }
        if filter.state.initialized { f.insert(.filterInit) }
        if calibrating { f.insert(.calibrating) }
        return f
    }

    private func publish(t: Double, state st: FilterState, recorder rec: RunRecorder?) {
        let h = lastDepthFresh(at: t) ? lidarH
            : (t - lastFlowT < 0.5 && lastFlowH > 0 ? lastFlowH : settings.manualHeight)
        snap.t = t
        snap.vx = st.vx; snap.vy = st.vy
        snap.sigmaVx = st.sigmaVx; snap.sigmaVy = st.sigmaVy
        snap.bx = st.bx; snap.by = st.by
        snap.distance = distance.leadCompensated
        snap.netForward = netForward.distance
        snap.flowQuality = t - lastFlowT < 0.5 ? lastFlowQuality : 0
        snap.h = h
        snap.status = statusFlags(at: t)
        snap.imuRateHz = imuRate.hz
        snap.flowRateHz = t - lastFlowT < 0.5 ? flowRate.hz : 0
        rec?.recordEstimate(t: t, vx: st.vx, vy: st.vy, sigmaVx: st.sigmaVx, sigmaVy: st.sigmaVy,
                            distance: snap.distance, status: snap.status.rawValue,
                            flowQuality: snap.flowQuality, h: h)
        let s = snap
        lock.withLock { _latest = s }
        onSnapshot?(s)
    }
}

/// Exponentially smoothed sample-rate estimate.
struct RateMeter: Sendable {
    private var lastT = -Double.infinity
    private(set) var hz = 0.0
    mutating func tick(_ t: Double) {
        let dt = t - lastT
        lastT = t
        guard dt > 0, dt < 1 else { return }
        let inst = 1 / dt
        hz = hz == 0 ? inst : 0.95 * hz + 0.05 * inst
    }
}
