import FlyFreeCore
import simd

/// Slider-driven stand-in for real hands, so the full pipeline runs in the visionOS simulator.
/// Defaults to both hands held out flat, so calibration completes on its own.
struct SimulatedHands {
    var present = true
    /// Stick hand closed: reset orientation.
    var stickClosed = false
    /// Degrees, relative to the calibrated flat pose.
    var pitch = 0.0
    var yaw = 0.0
    var roll = 0.0
    /// Throttle pedal, degrees of fingertip push: positive = fingertips down = accelerate.
    var pedal = 0.0
    /// Throttle hand closed: brake to a stop.
    var throttleClosed = false
    /// Throttle hand moved from its calibrated spot (cm): up, and forward (drone).
    var handUp = 0.0
    var handForward = 0.0

    /// - Parameter poses: the craft's starting pose per hand; slider motions apply on top of it,
    ///   in the pilot's frame, exactly as real hand motions would.
    func frames(dominant: FlyFreeCore.Chirality, poses: [FlyFreeCore.Chirality: HandPose] = [:]) -> (left: HandFrame?, right: HandFrame?) {
        guard present else { return (nil, nil) }
        let radians = Double.pi / 180
        func curl(_ amount: Double) -> [Finger: Double] {
            [.index: amount, .middle: amount, .ring: amount, .little: amount]
        }
        let stick = SyntheticHand.make(
            chirality: dominant,
            orientation: Quat.fromYawPitchRoll(yaw: yaw * radians, pitch: pitch * radians, roll: roll * radians)
                * (poses[dominant] ?? .flat).orientation(for: dominant),
            position: Vec3(dominant == .right ? 0.2 : -0.2, 1.0, -0.35),
            curl: curl(stickClosed ? 0.9 : 0)
        )
        let throttle = SyntheticHand.make(
            chirality: dominant.opposite,
            orientation: Quat(angle: -pedal * radians, axis: BodyAxis.right)
                * (poses[dominant.opposite] ?? .flat).orientation(for: dominant.opposite),
            position: Vec3(dominant == .right ? -0.2 : 0.2, 1.0 + handUp / 100, -0.35 - handForward / 100),
            curl: curl(throttleClosed ? 0.9 : 0)
        )
        return dominant == .right ? (throttle, stick) : (stick, throttle)
    }
}
