import simd

public struct ControllerGains: Sendable, Equatable {
    /// Angular-rate loop gain (1/s) per body axis.
    public var rate: Vec3
    /// Attitude loop gain (1/s): attitude error (rad) → corrective rate (rad/s).
    public var attitude: Double
    /// Velocity loop gain (1/s) for velocity-hold assist.
    public var velocity: Double

    public init(rate: Vec3 = Vec3(6, 6, 6), attitude: Double = 1.2, velocity: Double = 2.0) {
        self.rate = rate
        self.attitude = attitude
        self.velocity = velocity
    }
}

/// Stage 3: closed-loop controller. `DesiredCraftState` + current state → body wrench demand.
public struct FlightController: Sendable {
    public var gains: ControllerGains
    public var mass: MassProperties
    /// Max main-engine thrust (N); used to scale `thrustFraction` in assist-off mode.
    public var maxForwardThrust: Double
    /// Cap on the corrective rate the attitude loop may command (rad/s).
    public var maxHoldRate: Vec3

    public init(gains: ControllerGains, mass: MassProperties, maxForwardThrust: Double, maxHoldRate: Vec3) {
        self.gains = gains
        self.mass = mass
        self.maxForwardThrust = maxForwardThrust
        self.maxHoldRate = maxHoldRate
    }

    /// - Parameters:
    ///   - airframeTorque: moments the airframe already produces (stability, damping); the
    ///     controller subtracts them so surfaces only supply the difference (dynamic inversion).
    ///   - bodyAirVelocity: for aircraft, the autothrottle tracks air-relative speed.
    public func demand(desired: DesiredCraftState, state: RigidBodyState,
                       airframeTorque: Vec3 = .zero, bodyAirVelocity: Vec3? = nil) -> Wrench {
        var targetRate = desired.angularRate
        if let attitude = desired.attitude {
            // Attitude error as a body-frame rotation vector.
            let error = HandMetrics.rotationVector(current: attitude, neutral: state.orientation)
            targetRate += simd_clamp(error * gains.attitude, -maxHoldRate, maxHoldRate)
        }

        let omega = state.angularVelocity
        let inertia = mass.inertia
        let torque = inertia * (desired.angularAcceleration + gains.rate * (targetRate - omega))
            + simd_cross(omega, inertia * omega) - airframeTorque

        let force: Vec3
        switch desired.assist {
        case .off:
            force = BodyAxis.forward * desired.thrustFraction * maxForwardThrust
        case .velocityHold:
            let bodyVelocity = bodyAirVelocity ?? state.orientation.inverse.act(state.velocity)
            var f = (desired.linearAcceleration + (desired.bodyVelocity - bodyVelocity) * gains.velocity) * mass.mass
            if desired.thrustAxisOnly { f = Vec3(0, 0, f.z) }
            force = f
        }
        return Wrench(force: force, torque: torque)
    }
}
