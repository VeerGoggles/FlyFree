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
    public var frame: ControlFrame
    /// Attitude a closed stick hand returns the craft to (the launch attitude).
    public var resetAttitude: Quat
    /// Fraction of max forward speed / thrust available in reverse.
    public var reverseFraction = 0.25
    /// Body rate (rad/s) below which a centered stick locks in attitude hold.
    public var settleRate = 0.02

    /// Rate reference in body axes (what the controller tracks).
    private var rateReference = Vec3.zero
    /// Global-frame mode: the same reference held in world axes, so it stays fixed in space
    /// as the ship turns.
    private var worldRateReference = Vec3.zero
    private var velocityReference = Vec3.zero
    private var heldAttitude: Quat?

    public init(profile: ControlProfile, assist: FlightAssistMode, frame: ControlFrame = .global,
                resetAttitude: Quat = Quat(ix: 0, iy: 0, iz: 0, r: 1)) {
        self.profile = profile
        self.assist = assist
        self.frame = frame
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

        // Rotation. Attitude is never decomposed into angles: every frame change is a quaternion
        // rotation of a vector, so there is no gimbal lock at any attitude.
        let q = state.orientation
        var targetRate = Vec3.zero  // body axes
        if input.engaged {
            var pilotRate = Vec3.zero  // axes of the pilot's hand frame
            for axis in 0..<3 {
                pilotRate[axis] = profile.rotationShaping.apply(input.rotation[axis]) * profile.maxAngularRate[axis]
            }
            if profile.invertPitch { pilotRate.x = -pilotRate.x }
            switch frame {
            case .body:
                targetRate = pilotRate
            case .global:
                // Hand frame ≙ launch frame → world → the ship's *current* body frame.
                targetRate = Self.limit(q.inverse.act(resetAttitude.act(pilotRate)), to: profile.maxAngularRate)
            }
        } else if input.stickClosed {
            let error = HandMetrics.rotationVector(current: resetAttitude, neutral: q)
            targetRate = simd_clamp(error * profile.levelingGain, -profile.maxAngularRate, profile.maxAngularRate)
        }

        // Slew as a vector so the rotation axis never bends while rates build up; the step size
        // respects each body axis's own limit (an ellipsoid of per-axis limits). In global mode the
        // reference lives in world axes between frames, so a world-fixed command stays world-fixed
        // while the ship rotates underneath it.
        var current = frame == .global ? q.inverse.act(worldRateReference) : rateReference
        let slowing = simd_length(targetRate) < simd_length(current) || simd_dot(targetRate, current) < 0
        current = Self.slew(current, toward: targetRate,
                            limits: slowing ? profile.angularDeceleration : profile.angularAcceleration, dt: dt)
        rateReference = current
        worldRateReference = q.act(current)

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

    /// Scales `rate` uniformly (keeping its axis) so no body component exceeds its limit.
    static func limit(_ rate: Vec3, to max: Vec3) -> Vec3 {
        var scale = 1.0
        for axis in 0..<3 where abs(rate[axis]) > max[axis] {
            scale = min(scale, max[axis] / abs(rate[axis]))
        }
        return rate * scale
    }

    /// Vector slew whose maximum step along any direction lies on the ellipsoid of per-axis
    /// limits — the direction of change is preserved exactly.
    static func slew(_ value: Vec3, toward target: Vec3, limits: Vec3, dt: Double) -> Vec3 {
        let delta = target - value
        let distance = simd_length(delta)
        guard distance > 0 else { return target }
        let d = delta / distance
        let limit = 1 / simd_length(d / limits)
        return slew(value, toward: target, step: limit * dt)
    }

    static func slew(_ value: Vec3, toward target: Vec3, step: Double) -> Vec3 {
        let delta = target - value
        let distance = simd_length(delta)
        return distance <= step ? target : value + delta * (step / distance)
    }
}
