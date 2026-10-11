import simd

/// Stage 2: `PilotInput` (normalized, craft-agnostic) → `DesiredCraftState` (physical targets),
/// applied in the craft's *current* frame of reference.
///
/// Rotation is rate-command / attitude-hold: a deflected stick commands a body rate; when the
/// stick is centered or released, the attitude at that moment becomes the target to hold.
public struct IntentMapper: Sendable {
    public var profile: ControlProfile
    public var assist: FlightAssistMode
    /// Fraction of max forward speed / thrust available in reverse.
    public var reverseFraction = 0.25
    /// Body rate (rad/s) below which a centered stick locks in attitude hold.
    public var settleRate = 0.02
    private var heldAttitude: Quat?

    public init(profile: ControlProfile, assist: FlightAssistMode) {
        self.profile = profile
        self.assist = assist
    }

    public mutating func map(_ input: PilotInput, state: RigidBodyState) -> DesiredCraftState {
        var rate = Vec3.zero
        if input.engaged {
            for axis in 0..<3 {
                rate[axis] = profile.rotationShaping.apply(input.rotation[axis]) * profile.maxAngularRate[axis]
            }
            if profile.invertPitch { rate.x = -rate.x }
        }

        // Capture the hold target only once rotation has settled, so a stick released mid-turn
        // decelerates to a stop instead of being yanked back to where it was released.
        if simd_length(rate) > 0 {
            heldAttitude = nil
        } else if heldAttitude == nil, simd_length(state.angularVelocity) < settleRate {
            heldAttitude = state.orientation
        }

        var throttle = profile.throttleShaping.apply(input.throttle)
        if throttle < 0 { throttle *= reverseFraction }

        switch assist {
        case .off:
            return DesiredCraftState(angularRate: rate, attitude: heldAttitude, assist: .off,
                                     thrustFraction: throttle)
        case .velocityHold:
            var velocity = input.translation * profile.maxStrafeSpeed
            velocity.z = -throttle * profile.maxForwardSpeed
            return DesiredCraftState(angularRate: rate, attitude: heldAttitude, assist: .velocityHold,
                                     bodyVelocity: velocity)
        }
    }
}
