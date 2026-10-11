import FlyFreeCore

/// Slider-driven stand-in for real hands, so the full pipeline runs in the visionOS simulator.
struct SimulatedHands {
    var grip = false
    /// Degrees, relative to the pose at the moment of grip.
    var pitch = 0.0
    var yaw = 0.0
    var roll = 0.0
    /// 0...1 finger curl on the throttle hand.
    var throttleCurl = 0.0

    func frames(dominant: FlyFreeCore.Chirality, attitudeMode: AttitudeInputMode) -> (left: HandFrame?, right: HandFrame?) {
        let gripCurl = grip ? 0.9 : 0.0
        let indexCurl = attitudeMode == .pointer ? 0 : gripCurl
        let radians = Double.pi / 180
        let stick = SyntheticHand.make(
            chirality: dominant,
            orientation: .fromYawPitchRoll(yaw: yaw * radians, pitch: pitch * radians, roll: roll * radians),
            position: Vec3(dominant == .right ? 0.2 : -0.2, 1.0, -0.35),
            curl: [.index: indexCurl, .middle: gripCurl, .ring: gripCurl, .little: gripCurl]
        )
        let throttle = SyntheticHand.make(
            chirality: dominant.opposite,
            position: Vec3(dominant == .right ? -0.2 : 0.2, 1.0, -0.35),
            curl: [.index: throttleCurl, .middle: throttleCurl, .ring: throttleCurl, .little: throttleCurl]
        )
        return dominant == .right ? (throttle, stick) : (stick, throttle)
    }
}
