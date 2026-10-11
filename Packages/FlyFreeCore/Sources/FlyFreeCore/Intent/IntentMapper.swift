import simd

/// Stage 2: `PilotInput` (normalized, craft-agnostic) → `DesiredCraftState` (physical targets),
/// applied in the craft's *current* frame of reference.
///
/// The mapper owns smooth *reference* signals that the controller then tracks tightly:
/// - **Rotation:** stick → target body rate, slewed by the profile's angular acceleration
///   (spin-up) and deceleration (spin-down) limits. Centered or released → attitude hold.
///   Closed stick hand → rotate back to the reset attitude.
/// - **Translation:** the throttle is a pedal. Tipping the hand down accelerates, level keeps
///   the current speed, tipping up brakes (then reverses). A closed throttle hand brakes to a
///   full stop. Speed changes are limited by the profile's linear acceleration / deceleration.
public struct IntentMapper: Sendable {
    public var profile: ControlProfile
    public var assist: FlightAssistMode
    /// Attitude a closed stick hand returns the craft to (the launch attitude).
    public var resetAttitude: Quat
    /// Fraction of max forward speed / thrust available in reverse.
    public var reverseFraction = 0.25
    /// Body rate (rad/s) below which a centered stick locks in attitude hold.
    public var settleRate = 0.02

    private var rateReference = Vec3.zero
    private var velocityReference = Vec3.zero
    private var heldAttitude: Quat?

    public init(profile: ControlProfile, assist: FlightAssistMode, resetAttitude: Quat = Quat(ix: 0, iy: 0, iz: 0, r: 1)) {
        self.profile = profile
        self.assist = assist
        self.resetAttitude = resetAttitude
    }

    public mutating func map(_ input: PilotInput, state: RigidBodyState, dt: Double) -> DesiredCraftState {
        let previousRate = rateReference
        let previousVelocity = velocityReference
        var desired = reference(input, state: state, dt: dt)
        if dt > 0 {
            desired.angularAcceleration = (rateReference - previousRate) / dt
            if desired.assist == .velocityHold {
                desired.linearAcceleration = (velocityReference - previousVelocity) / dt
            }
        }
        return desired
    }

    private mutating func reference(_ input: PilotInput, state: RigidBodyState, dt: Double) -> DesiredCraftState {
        let bodyVelocity = state.orientation.inverse.act(state.velocity)

        // Rotation.
        var targetRate = Vec3.zero
        if input.engaged {
            for axis in 0..<3 {
                targetRate[axis] = profile.rotationShaping.apply(input.rotation[axis]) * profile.maxAngularRate[axis]
            }
            if profile.invertPitch { targetRate.x = -targetRate.x }
        } else if input.stickClosed {
            let error = HandMetrics.rotationVector(current: resetAttitude, neutral: state.orientation)
            targetRate = simd_clamp(error * profile.levelingGain, -profile.maxAngularRate, profile.maxAngularRate)
        }
        for axis in 0..<3 {
            rateReference[axis] = Self.slew(rateReference[axis], toward: targetRate[axis],
                                            up: profile.angularAcceleration[axis],
                                            down: profile.angularDeceleration[axis], dt: dt)
        }

        // Hold attitude once rotation has settled with nothing commanded, so a stick released
        // mid-turn decelerates to a stop instead of being yanked back.
        if input.stickClosed || simd_length(targetRate) > 0 || simd_length(rateReference) > 0 {
            heldAttitude = nil
        } else if heldAttitude == nil, simd_length(state.angularVelocity) < settleRate {
            heldAttitude = state.orientation
        }

        // Translation.
        let throttle = profile.throttleShaping.apply(input.throttle)
        let brakeStep = profile.linearDeceleration * dt
        if input.throttleClosed {
            velocityReference = Self.slew(velocityReference, toward: .zero, step: brakeStep)
            return DesiredCraftState(angularRate: rateReference, attitude: heldAttitude,
                                     assist: .velocityHold, bodyVelocity: velocityReference)
        }

        switch assist {
        case .off:
            velocityReference = bodyVelocity
            return DesiredCraftState(angularRate: rateReference, attitude: heldAttitude, assist: .off,
                                     thrustFraction: throttle < 0 ? throttle * reverseFraction : throttle)
        case .velocityHold:
            // Forward is -Z. Accelerate (or brake) along it; bleed off any sideways drift.
            var forwardSpeed = -velocityReference.z
            let rate = throttle * (throttle * forwardSpeed >= 0 ? profile.linearAcceleration : profile.linearDeceleration)
            forwardSpeed = (forwardSpeed + rate * dt)
                .clamped(to: -profile.maxForwardSpeed * reverseFraction...profile.maxForwardSpeed)
            let lateral = Self.slew(Vec3(velocityReference.x, velocityReference.y, 0),
                                    toward: input.translation * profile.maxStrafeSpeed, step: brakeStep)
            velocityReference = Vec3(lateral.x, lateral.y, -forwardSpeed)
            return DesiredCraftState(angularRate: rateReference, attitude: heldAttitude, assist: .velocityHold,
                                     bodyVelocity: velocityReference)
        }
    }

    /// Moves `value` toward `target`, limited to `up`·dt when moving away from zero and
    /// `down`·dt when moving toward (or through) zero.
    static func slew(_ value: Double, toward target: Double, up: Double, down: Double, dt: Double) -> Double {
        let slowing = abs(target) < abs(value) || target * value < 0
        let limit = (slowing ? down : up) * dt
        return value + (target - value).clamped(to: -limit...limit)
    }

    static func slew(_ value: Vec3, toward target: Vec3, step: Double) -> Vec3 {
        let delta = target - value
        let distance = simd_length(delta)
        return distance <= step ? target : value + delta * (step / distance)
    }
}
