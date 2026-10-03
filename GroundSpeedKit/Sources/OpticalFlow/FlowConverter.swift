import Foundation

/// Velocity in a 2-D frame (m/s).
public struct PlanarVelocity: Sendable, Equatable {
    public var vx: Double
    public var vy: Double
    public init(vx: Double, vy: Double) { self.vx = vx; self.vy = vy }
    public var speed: Double { (vx * vx + vy * vy).squareRoot() }
}

/// Converts a pixel shift between frames into the camera's velocity over the floor.
///
/// Pinhole model, camera looking at a plane at depth Z: a ground displacement ΔX moves the
/// image by Δp = f·ΔX/Z pixels, so ΔX = Δp·Z/f. With the shift measured on a grid
/// downsampled by `downsample`: v = (Δp · downsample / Δt) · (Z / f_px).
///
/// Sign: floor content moving +x in the image means the camera moved −x over the floor, so
/// the returned velocity is **the camera's own velocity, v = −(content motion)**, expressed in
/// image axes (x = image right / increasing column, y = image down / increasing row).
/// Map to vehicle axes with `MountMapping`.
///
/// Tilt: `h` is the camera's *vertical* height above the floor. If the optical axis is
/// tilted by θ from straight down, the slant depth along the axis is h/cos θ, where
/// cos θ = |ĝ·ẑ_cam| from the gravity vector in camera coordinates (z = optical axis).
/// (This is the first-order correction; foreshortening across the tilt direction is
/// ignored, fine for θ ≲ 15°.) If `h` already is the LiDAR range along the optical axis,
/// pass `gravityCam: nil`.
public struct FlowConverter: Sendable, Equatable, Codable {
    /// Focal length in full-resolution pixels (camera intrinsics fx ≈ fy).
    public var focalPx: Double
    /// Downsample factor between full-res pixels and the correlator grid.
    public var downsample: Double
    /// Lower bound on cos θ used by the tilt correction (avoid blow-up; 0.5 = 60°).
    public var minCosTilt: Double
    /// Empirical scale fudge (from a measured-distance calibration push), default 1.
    public var scaleFactor: Double

    public init(focalPx: Double, downsample: Double = 2, minCosTilt: Double = 0.5, scaleFactor: Double = 1) {
        self.focalPx = focalPx; self.downsample = downsample
        self.minCosTilt = minCosTilt; self.scaleFactor = scaleFactor
    }

    enum CodingKeys: String, CodingKey {
        case focalPx = "focal_px", downsample, minCosTilt = "min_cos_tilt", scaleFactor = "scale_factor"
    }

    /// cos of the angle between the optical axis (+z_cam) and gravity.
    public func cosTilt(gravityCam g: SIMD3<Double>) -> Double {
        let norm = (g * g).sum().squareRoot()
        guard norm > 0 else { return 1 }
        return max(minCosTilt, min(1, abs(g.z) / norm))
    }

    /// Metres on the floor per correlator-grid pixel.
    public func metresPerPixel(h: Double, gravityCam: SIMD3<Double>? = nil) -> Double {
        let depth = gravityCam.map { h / cosTilt(gravityCam: $0) } ?? h
        return downsample * depth / focalPx * scaleFactor
    }

    /// Camera velocity (image axes, m/s) from a correlator shift over `dt` seconds.
    public func velocity(dx: Double, dy: Double, dt: Double, h: Double,
                         gravityCam: SIMD3<Double>? = nil) -> PlanarVelocity {
        guard dt > 0 else { return PlanarVelocity(vx: 0, vy: 0) }
        let k = metresPerPixel(h: h, gravityCam: gravityCam) / dt
        return PlanarVelocity(vx: -dx * k, vy: -dy * k)
    }

    public func velocity(_ s: FlowShift, dt: Double, h: Double, gravityCam: SIMD3<Double>? = nil) -> PlanarVelocity {
        velocity(dx: s.dx, dy: s.dy, dt: dt, h: h, gravityCam: gravityCam)
    }
}

/// Maps camera image axes (x right, y down) to vehicle axes (x forward, y left).
/// Applied as: optional swap of (x, y), then optional sign flips of each output axis.
///
/// Physically valid (non-mirrored) mappings for a downward-looking camera have
/// det = −1 (because z_cam points down while z_vehicle points up); `isProper` reports that.
/// Fix on site with `fromForwardPush` or by toggling the flags in Settings.
public struct MountMapping: Sendable, Equatable, Codable, Hashable {
    public var swapXY: Bool
    public var flipX: Bool
    public var flipY: Bool

    public init(swapXY: Bool = false, flipX: Bool = false, flipY: Bool = true) {
        self.swapXY = swapXY; self.flipX = flipX; self.flipY = flipY
    }

    enum CodingKeys: String, CodingKey { case swapXY = "swap_xy", flipX = "flip_x", flipY = "flip_y" }

    /// Default: image-right = forward, image-down = vehicle right (vehicle y = −image y).
    /// A proper mapping, but which way the phone sensor faces depends on the mount: verify
    /// with a forward push (vx should come out positive).
    public static let standard = MountMapping()
    /// No swap, no flips (a mirror mapping for a down-looking camera; for debugging).
    public static let identity = MountMapping(swapXY: false, flipX: false, flipY: false)

    public func apply(_ v: PlanarVelocity) -> PlanarVelocity {
        let (a, b) = swapXY ? (v.vy, v.vx) : (v.vx, v.vy)
        return PlanarVelocity(vx: flipX ? -a : a, vy: flipY ? -b : b)
    }

    public func apply(x: Double, y: Double) -> (x: Double, y: Double) {
        let r = apply(PlanarVelocity(vx: x, vy: y))
        return (r.vx, r.vy)
    }

    /// Determinant of the 2×2 map.
    public var determinant: Int {
        let s = (flipX ? -1 : 1) * (flipY ? -1 : 1)
        return swapXY ? -s : s
    }

    /// True for rotations of a downward-looking camera (det = −1), false for mirror images.
    public var isProper: Bool { determinant == -1 }

    /// Infers the mapping from the mean camera-axes velocity measured during a straight
    /// forward push: the dominant axis becomes vehicle x with positive sign, and the y flip is
    /// chosen so the mapping is proper (no mirror).
    public static func fromForwardPush(vxCam: Double, vyCam: Double) -> MountMapping {
        let swap = abs(vyCam) > abs(vxCam)
        let fwd = swap ? vyCam : vxCam
        let flipX = fwd < 0
        var m = MountMapping(swapXY: swap, flipX: flipX, flipY: false)
        if !m.isProper { m.flipY = true }
        return m
    }
}
