/// Loads applied by the environment rather than the craft's actuators.
public struct ExternalLoads: Sendable, Equatable {
    public var worldForce: Vec3
    public var bodyTorque: Vec3

    public init(worldForce: Vec3 = .zero, bodyTorque: Vec3 = .zero) {
        self.worldForce = worldForce
        self.bodyTorque = bodyTorque
    }

    public static let none = ExternalLoads()
}

/// The world the craft flies in. Extend with gravity, atmosphere, wind, etc.
public protocol FlightEnvironment: Sendable {
    func loads(on state: RigidBodyState, mass: MassProperties) -> ExternalLoads
}

/// No gravity, no atmosphere, no wind.
public struct DeepSpace: FlightEnvironment {
    public init() {}
    public func loads(on state: RigidBodyState, mass: MassProperties) -> ExternalLoads { .none }
}

/// Uniform gravity field — a first step toward landers and atmospheric craft.
public struct UniformGravity: FlightEnvironment {
    public var acceleration: Vec3
    public init(acceleration: Vec3 = Vec3(0, -9.81, 0)) { self.acceleration = acceleration }
    public func loads(on state: RigidBodyState, mass: MassProperties) -> ExternalLoads {
        ExternalLoads(worldForce: acceleration * mass.mass)
    }
}
