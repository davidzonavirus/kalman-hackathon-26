import CoreMotion
import Foundation
import PhoneRuntime

/// CoreMotion device motion at 100 Hz -> vehicle-frame IMU samples.
///
/// userAcceleration (g, gravity removed) × 9.80665 -> m/s²; rotationRate (rad/s).
/// Both are device-frame and rotated to the vehicle frame with `IMUMountMapping`
/// (default: phone flat, screen up, top of phone pointing forward).
final class MotionSource: @unchecked Sendable {
    typealias Sink = (_ t: Double, _ ax: Double, _ ay: Double, _ az: Double,
                      _ gx: Double, _ gy: Double, _ gz: Double) -> Void

    private let manager = CMMotionManager()
    private let queue: OperationQueue = {
        let q = OperationQueue()
        q.name = "gsk.motion"
        q.maxConcurrentOperationCount = 1
        q.qualityOfService = .userInteractive
        return q
    }()
    private let lock = NSLock()
    private var _mapping = IMUMountMapping()
    private var _gravity: SIMD3<Double>?
    private var _running = false

    /// Receives every sample (on the motion queue).
    var onSample: Sink?

    var mapping: IMUMountMapping {
        get { lock.withLock { _mapping } }
        set { lock.withLock { _mapping = newValue } }
    }

    /// Latest gravity vector in DEVICE coordinates (unit g), for the flow tilt correction.
    var gravityDevice: SIMD3<Double>? { lock.withLock { _gravity } }

    var isAvailable: Bool { manager.isDeviceMotionAvailable }
    var isRunning: Bool { lock.withLock { _running } }

    @discardableResult
    func start() -> Bool {
        guard manager.isDeviceMotionAvailable else { return false }
        guard !isRunning else { return true }
        manager.deviceMotionUpdateInterval = 1.0 / 100.0
        manager.showsDeviceMovementDisplay = false
        manager.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: queue) { [weak self] motion, _ in
            guard let self, let m = motion else { return }
            let g0 = 9.80665
            let ua = m.userAcceleration
            let rr = m.rotationRate
            let map = self.mapping
            let a = map.apply(x: ua.x * g0, y: ua.y * g0, z: ua.z * g0)
            let w = map.apply(x: rr.x, y: rr.y, z: rr.z)
            let grav = SIMD3<Double>(m.gravity.x, m.gravity.y, m.gravity.z)
            self.lock.withLock { self._gravity = grav }
            // CMLogItem.timestamp is seconds since boot on the mach clock (same as Clock.now()).
            self.onSample?(Clock.fromMotionTimestamp(m.timestamp), a.x, a.y, a.z, w.x, w.y, w.z)
        }
        lock.withLock { _running = true }
        return true
    }

    func stop() {
        manager.stopDeviceMotionUpdates()
        lock.withLock { _running = false }
    }
}
