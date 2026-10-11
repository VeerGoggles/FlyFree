import FlyFreeCore

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

    func frames(dominant: FlyFreeCore.Chirality) -> (left: HandFrame?, right: HandFrame?) {
        guard present else { return (nil, nil) }
        let radians = Double.pi / 180
        func curl(_ amount: Double) -> [Finger: Double] {
            [.index: amount, .middle: amount, .ring: amount, .little: amount]
        }
        let stick = SyntheticHand.make(
            chirality: dominant,
            orientation: .fromYawPitchRoll(yaw: yaw * radians, pitch: pitch * radians, roll: roll * radians),
            position: Vec3(dominant == .right ? 0.2 : -0.2, 1.0, -0.35),
            curl: curl(stickClosed ? 0.9 : 0)
        )
        let throttle = SyntheticHand.make(
            chirality: dominant.opposite,
            orientation: Quat(angle: -pedal * radians, axis: BodyAxis.right),
            position: Vec3(dominant == .right ? -0.2 : 0.2, 1.0, -0.35),
            curl: curl(throttleClosed ? 0.9 : 0)
        )
        return dominant == .right ? (throttle, stick) : (stick, throttle)
    }
}
