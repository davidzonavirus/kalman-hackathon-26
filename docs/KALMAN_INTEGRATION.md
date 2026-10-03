# Incorporating Joseph's Kalman filter

The app never talks to a concrete filter. It talks to the `GroundSpeedFilter`
protocol in `GroundSpeedKit/Sources/KalmanCore`. We ship a working reference
filter (`ReferenceKF4`) so the app runs end-to-end today; Joseph's filter
replaces it by conforming to the same protocol. Nothing else in the app changes.

## The contract (Swift)

```swift
public struct FilterState: Sendable, Equatable {
    public var t: Double                 // time of last predict/update (s)
    public var vx, vy: Double            // m/s, vehicle frame
    public var bx, by: Double            // accel biases, m/s²
    public var pDiag: [Double]           // [P_vx, P_vy, P_bx, P_by]
    public var sigmaVx: Double { pDiag[0].squareRoot() }
    public var sigmaVy: Double { pDiag[1].squareRoot() }
    public var initialized: Bool
}

public enum UpdateOutcome: Sendable, Equatable {
    case accepted(nis: Double)           // nis = ν²/S
    case gated(nis: Double)              // rejected by innovation gate
    case skipped(reason: String)         // e.g. GNSS speed < 1 m/s, not initialized
}

public protocol GroundSpeedFilter: AnyObject {
    static var name: String { get }      // goes into meta.json "filter_name"
    var state: FilterState { get }
    var config: FilterConfig { get set } // Q, R, gate, etc. (Codable → meta.json)

    func reset(t: Double)

    /// 100 Hz, on every IMU sample. Vehicle frame, gravity removed.
    /// ax, ay in m/s²; r = yaw rate in rad/s. dt derived from t − state.t.
    func predict(t: Double, ax: Double, ay: Double, r: Double)

    /// Optical flow velocity (vehicle frame, m/s). quality = PSR.
    /// Applied as two scalar updates (z = v_x, z = v_y) with R scaled by quality.
    @discardableResult
    func updateFlow(t: Double, vx: Double, vy: Double, quality: Double) -> UpdateOutcome

    /// GNSS speed |v| with reported accuracy (1σ m/s). Skip below 1 m/s.
    @discardableResult
    func updateGNSS(t: Double, speed: Double, speedAccuracy: Double) -> UpdateOutcome

    /// Zero-velocity pseudo-measurement z = 0 on both velocities.
    @discardableResult
    func updateZeroVelocity(t: Double) -> UpdateOutcome
}
```

Distance is NOT the filter's job: `DistanceIntegrator` (KalmanCore) integrates
`hypot(vx, vy)` from the filter state after each predict, so every filter variant gets
the same distance logic.

## Model the reference implements (matches the design doc)

State `x = [v_x, v_y, b_x, b_y]`, inputs `u = [a_x, a_y, r]`, Δt from timestamps (nominal 10 ms):

```
v_x' = v_x + Δt·( a_x − b_x + r·v_y )
v_y' = v_y + Δt·( a_y − b_y − r·v_x )
b'   = b                       (random walk, Q_b)
F = I + Δt·[[0,  r, −1,  0],
            [−r, 0,  0, −1],
            [0,  0,  0,  0],
            [0,  0,  0,  0]]
```

- Covariance predict written out by hand (sparsity), symmetric enforced.
- Sequential scalar updates, one division each, **Joseph form** `P = (I−KH)P(I−KH)ᵀ + K R Kᵀ`.
- Flow: `H = e_x` then `e_y`; `R_flow = r_flow_base · (psr_ref / max(quality, psr_min))²`.
- GNSS: `H = [v_x/|v|, v_y/|v|, 0, 0]`, `R = speed_acc²`, only when `|v̂| > 1` and `speed > 1`.
- ZUPT: `z = 0` on both velocities with `R_zupt`.
- Gate: reject when `ν²/S > gate` (default 9 = 3σ). Gating disabled while uninitialized.

## How Joseph plugs in (pick one)

**A. Replace the reference (recommended once his matches `kf_ref.py`).**
1. Add `JosephKF.swift` in `GroundSpeedKit/Sources/KalmanCore/` with
   `public final class JosephKF: GroundSpeedFilter`.
2. Register it in `FilterRegistry.all` (one line).
3. In the app: Settings → Filter → pick "JosephKF". (Default is set in `FilterRegistry.defaultName`.)

**B. Ship variants side by side** (full, decoupled 2×2, steady-state gain): register
each; the app's picker and `meta.json` record which ran, and the replay tool runs
all of them on the same log for the variant table.

## Verifying a filter (do this before the demo)

```bash
cd GroundSpeedKit
swift run kfreplay path/to/runs/<run_id> --filter JosephKF --out est_swift.csv
swift run gsk-checks          # unit checks: protocol CRC, filter sanity, flow math
```

`kfreplay` merges `imu.csv`/`flow.csv`/`gnss.csv`/`events.csv` by timestamp and
writes `est.csv` with the frozen header. Compare against `kf_ref.py` output; the
pass bar is max |Δv| ≤ 1e-6 m/s. Ordering rule at equal timestamps:
predict (IMU) first, then flow, then GNSS, then ZUPT.

## Knobs the app exposes at runtime

`FilterConfig` (Codable) is saved per run in `meta.json` and editable in the app's
Settings screen: `q_accel`, `q_bias`, `r_flow_base`, `psr_ref`, `psr_min`,
`r_zupt`, `gate`, `p0_v`, `p0_b`. Q/R tuned from the stationary log go here.

## Behaviour details `kf_ref.py` must match (as implemented in `ReferenceKF4`)

The full step-by-step algorithm is in the doc comment at the top of
`GroundSpeedKit/Sources/KalmanCore/ReferenceKF4.swift`. The non-obvious rules:

1. **Timing.** The first `predict` (or `reset(t:)`) initializes and does not propagate.
   `dt ≤ 0` → ignored; `dt > 0.1` → clamped to 0.1. `state.t` = time of last predict only.
2. **Low-quality flow is skipped**, not down-weighted: `quality < psr_min` → `.skipped`.
   (A covered lens otherwise reports a confident zero.)
3. **Gate arming.** After init/reset the gate is off until the first accepted
   flow/GNSS/ZUPT update, so a filter started while already moving can lock on.
4. **Flow gating uses the prior** x and P on both axes; if either axis's NIS > gate the
   whole sample is rejected, else update x then y. ZUPT is never gated.
5. **Defaults** (tuned to this correlator's PSR scale: noise ≤ ~7, textured floor 90–650;
   q_accel raised 0.02 → 0.1 after the first on-device run, halving gated flow during jolts):
   `q_accel 0.1, q_bias 1e-5, r_flow_base 1.6e-3, psr_ref 20, psr_min 8, r_zupt 1e-4,
   gate 9, p0_v 0.25, p0_b 0.01`.
6. **Replay order** at equal t: IMU predict → est row emitted → flow → GNSS → ZUPT.
   `kfreplay --zupt-detector auto` (default) uses the ZUPT rows the phone logged in
   `events.csv`; it runs the detector only if there are none (synthetic/raw logs).

Generate a test log any time: `swift run kfreplay --synth /tmp/run1` (16 s, exactly
10.000 m rest-to-rest, 2 s lens-covered dropout, two 5 m/s flow outliers, `truth.csv`).
`swift run kfreplay /tmp/run1 --all` prints every registered variant side by side.
7. **Gate lockout recovery** (`gate_reset_count`, default 12): after 12 consecutive gated
   flow samples (~0.1 s) the filter assumes ITS estimate diverged, sets P_vv cross terms to 0
   and P_ii = max(P_ii, p0_v) for both velocities, and accepts the sample. Exact rule is in
   the `ReferenceKF4.swift` header comment.
8. **Distance** is the signed forward odometer ∫v_x (trapezoid), not ∫|v|.
   `DistanceIntegrator(mode: .pathLength)` keeps the old behaviour.
9. **Flow arrives already de-rotated** (engine subtracts h·gyro before the filter, and logs the
   corrected value), so the filter itself needs no gyro term for flow.
