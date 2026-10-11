import simd

public struct ControllerGains: Sendable, Equatable {
    /// Angular-rate loop gain (1/s) per body axis.
    public var rate: Vec3
    /// Velocity loop gain (1/s) for velocity-hold assist.
    public var velocity: Double

    public init(rate: Vec3 = Vec3(2.5, 2.5, 3.0), velocity: Double = 0.8) {
        self.rate = rate
        self.velocity = velocity
    }
}

/// Stage 3: closed-loop controller. `DesiredCraftState` + current state → body wrench demand.
///
/// Rotation is rate-command / attitude-hold: commanded rate is tracked with a proportional
/// loop plus gyroscopic compensation; a zero command holds the current attitude.
public struct FlightController: Sendable {
    public var gains: ControllerGains
    public var mass: MassProperties
    /// Max main-engine thrust (N); used to scale `thrustFraction` in assist-off mode.
    public var maxForwardThrust: Double

    public init(gains: ControllerGains, mass: MassProperties, maxForwardThrust: Double) {
        self.gains = gains
        self.mass = mass
        self.maxForwardThrust = maxForwardThrust
    }

    public func demand(desired: DesiredCraftState, state: RigidBodyState) -> Wrench {
        let omega = state.angularVelocity
        let inertia = mass.inertia
        let rateError = desired.angularRate - omega
        let torque = inertia * (gains.rate * rateError) + simd_cross(omega, inertia * omega)

        let force: Vec3
        switch desired.assist {
        case .off:
            force = BodyAxis.forward * desired.thrustFraction * maxForwardThrust
        case .velocityHold:
            let bodyVelocity = state.orientation.inverse.act(state.velocity)
            force = (desired.bodyVelocity - bodyVelocity) * gains.velocity * mass.mass
        }
        return Wrench(force: force, torque: torque)
    }
}
