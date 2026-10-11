/// Stage 2: `PilotInput` (normalized, craft-agnostic) → `DesiredCraftState` (physical targets),
/// using the selected craft's `ControlProfile`.
public struct IntentMapper: Sendable {
    public var profile: ControlProfile
    public var assist: FlightAssistMode

    public init(profile: ControlProfile, assist: FlightAssistMode) {
        self.profile = profile
        self.assist = assist
    }

    public func map(_ input: PilotInput) -> DesiredCraftState {
        var rate = Vec3.zero
        if input.engaged {
            for axis in 0..<3 {
                rate[axis] = profile.rotationShaping.apply(input.rotation[axis]) * profile.maxAngularRate[axis]
            }
            if profile.invertPitch { rate.x = -rate.x }
        }
        let throttle = profile.throttleShaping.apply(input.throttle)

        switch assist {
        case .off:
            return DesiredCraftState(angularRate: rate, assist: .off, thrustFraction: throttle)
        case .velocityHold:
            var velocity = input.translation * profile.maxStrafeSpeed
            velocity.z = -throttle * profile.maxForwardSpeed
            return DesiredCraftState(angularRate: rate, assist: .velocityHold, bodyVelocity: velocity)
        }
    }
}
