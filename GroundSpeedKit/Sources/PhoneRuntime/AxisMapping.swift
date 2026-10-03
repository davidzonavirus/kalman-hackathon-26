import Foundation

/// Maps the phone's device axes (CoreMotion: x = right edge, y = top edge, z = out of screen)
/// to the vehicle frame (x = forward, y = left, z = up).
///
/// Base mounting (all toggles off): phone flat, screen up, top of the phone pointing forward:
///   vehicle x = device +y,  vehicle y = device −x,  vehicle z = device +z.
/// The toggles are then applied in the horizontal plane (swap first, then flips). An odd
/// number of toggles is a reflection, which physically means the phone is screen-down,
/// so z (accel z and yaw rate) flips sign to keep the frame right-handed.
public struct IMUMountMapping: Codable, Sendable, Equatable {
    public var swapXY: Bool
    public var flipX: Bool
    public var flipY: Bool

    public init(swapXY: Bool = false, flipX: Bool = false, flipY: Bool = false) {
        self.swapXY = swapXY; self.flipX = flipX; self.flipY = flipY
    }

    enum CodingKeys: String, CodingKey { case swapXY = "swap_xy", flipX = "flip_x", flipY = "flip_y" }

    /// +1 for a proper rotation, −1 for a reflection (phone screen-down).
    public var zSign: Double {
        var s = 1.0
        if swapXY { s = -s }
        if flipX { s = -s }
        if flipY { s = -s }
        return s
    }

    /// Device-frame vector → vehicle-frame vector.
    @inline(__always)
    public func apply(x dx: Double, y dy: Double, z dz: Double) -> (x: Double, y: Double, z: Double) {
        var x = dy
        var y = -dx
        if swapXY { swap(&x, &y) }
        if flipX { x = -x }
        if flipY { y = -y }
        return (x, y, dz * zSign)
    }
}
