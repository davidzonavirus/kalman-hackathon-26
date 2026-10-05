/**
 ******************************************************************************
 * @file : HeightTracker.swift
 * @brief : Tracks camera height between LiDAR fixes from image expansion.
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

/// Frame-to-frame relative height change from the shifts of four patches placed
/// symmetrically about the image centre (left/right at ∓`offsetX`, top/bottom at ∓`offsetY`).
///
/// The differences give the flow gradient A = ∂(u,v)/∂(x,y). For a camera over a plane,
/// moving by t with plane normal n (camera frame) and height d, the flow is the homography
/// u ∝ (t_x − x t_z)(n·p)/d. Moving towards the floor expands the image isotropically, but
/// so does moving *along* a floor the camera is tilted towards: that adds 2·n_x·t_x/d to A
/// along the motion and n_x·t_x/d across it (it would integrate to tens of % per metre).
/// With ∥ = flow direction and ⊥ = across it,
///
///     Δh/h = A∥ − 2·A⊥
///
/// cancels the tilt term exactly (also the quadratic terms, since only opposite-patch
/// differences enter) and leaves the true height change. Roll and pitch/yaw rotation and
/// symmetric lens distortion contribute nothing to these differences at first order.
public enum ImageExpansion {
    /// Δh/h between the two frames (negative = camera moved towards the floor).
    /// Shifts and offsets in the same pixel units.
    public static func relativeHeightChange(left: FlowShift, right: FlowShift, top: FlowShift, bottom: FlowShift,
                                            offsetX: Double, offsetY: Double) -> Double {
        let a11 = (right.dx - left.dx) / (2 * offsetX)
        let a21 = (right.dy - left.dy) / (2 * offsetX)
        let a12 = (bottom.dx - top.dx) / (2 * offsetY)
        let a22 = (bottom.dy - top.dy) / (2 * offsetY)
        let ux = (left.dx + right.dx + top.dx + bottom.dx) / 4
        let uy = (left.dy + right.dy + top.dy + bottom.dy) / 4
        let speed = (ux * ux + uy * uy).squareRoot()
        // At rest the direction is noise; the tilt term scales with the shift and is negligible.
        guard speed > 0.02 else { return -(a11 + a22) / 2 }
        let cx = ux / speed, cy = uy / speed
        let sym = a12 + a21
        let along = a11 * cx * cx + sym * cx * cy + a22 * cy * cy
        let across = a11 * cy * cy - sym * cx * cy + a22 * cx * cx
        return along - 2 * across
    }
}

/// Integrates `ImageExpansion` into a camera height, anchored by LiDAR / manual height.
public struct HeightTracker: Sendable {
    public private(set) var height: Double
    /// Height the tracker was last anchored to (LiDAR or manual).
    public private(set) var anchor: Double
    /// Every patch needs at least this PSR for a frame to count.
    public var minPSR = 12.0
    /// Larger per-frame changes are treated as correlation glitches. Real ride-height
    /// changes are well under 1 % per frame at 120–240 fps.
    public var maxStep = 0.02
    /// Tracked height stays within anchor × [1/maxRatio, maxRatio]. Car test (2026-10-04):
    /// with 3 the tracker collapsed to anchor/3 at 4–9 m/s and speed read 3× low.
    public var maxRatio = 1.5
    /// Frames whose mean patch shift exceeds this (patch px) are skipped: the side patches
    /// cannot follow a motion prediction, so their expansion is unreliable at speed.
    public var maxShift = 24.0

    public init(height: Double) {
        self.height = height
        self.anchor = height
    }

    public mutating func reset(to h: Double) {
        height = h
        anchor = h
    }

    /// Applies one frame; returns false if the frame was skipped (weak texture, glitch).
    @discardableResult
    public mutating func update(left: FlowShift, right: FlowShift, top: FlowShift, bottom: FlowShift,
                                offsetX: Double, offsetY: Double) -> Bool {
        guard min(left.psr, right.psr, top.psr, bottom.psr) >= minPSR else { return false }
        let ux = (left.dx + right.dx + top.dx + bottom.dx) / 4
        let uy = (left.dy + right.dy + top.dy + bottom.dy) / 4
        guard (ux * ux + uy * uy).squareRoot() <= maxShift else { return false }
        let k = ImageExpansion.relativeHeightChange(left: left, right: right, top: top, bottom: bottom,
                                                    offsetX: offsetX, offsetY: offsetY)
        guard k.isFinite, abs(k) < maxStep else { return false }
        height = min(anchor * maxRatio, max(anchor / maxRatio, height * (1 + k)))
        return true
    }
}
