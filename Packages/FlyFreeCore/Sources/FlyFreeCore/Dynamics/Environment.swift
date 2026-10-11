import Foundation

/// Loads applied by the environment rather than the craft's actuators.
public struct ExternalLoads: Sendable, Equatable {
    public var worldForce: Vec3
    public var bodyTorque: Vec3

    public init(worldForce: Vec3 = .zero, bodyTorque: Vec3 = .zero) {
        self.worldForce = worldForce
        self.bodyTorque = bodyTorque
    }

    public static let none = ExternalLoads()

    public static func + (a: ExternalLoads, b: ExternalLoads) -> ExternalLoads {
        ExternalLoads(worldForce: a.worldForce + b.worldForce, bodyTorque: a.bodyTorque + b.bodyTorque)
    }
}

/// Air at a point: what the aerodynamic model needs.
public struct AirState: Sendable, Equatable {
    /// kg/m³
    public var density: Double
    /// World-frame wind velocity (m/s).
    public var wind: Vec3
}

/// The world the craft flies in.
public protocol FlightEnvironment: Sendable {
    func loads(on state: RigidBodyState, mass: MassProperties) -> ExternalLoads
    /// Air at the craft's position, or nil in vacuum.
    func air(at position: Vec3) -> AirState?
    /// Height of the ground (world Y), or nil if there is none.
    var groundHeight: Double? { get }
}

extension FlightEnvironment {
    public func air(at position: Vec3) -> AirState? { nil }
    public var groundHeight: Double? { nil }
}

/// No gravity, no atmosphere, no wind.
public struct DeepSpace: FlightEnvironment {
    public init() {}
    public func loads(on state: RigidBodyState, mass: MassProperties) -> ExternalLoads { .none }
}

/// Uniform gravity field — a first step toward landers.
public struct UniformGravity: FlightEnvironment {
    public var acceleration: Vec3
    public init(acceleration: Vec3 = Vec3(0, -9.81, 0)) { self.acceleration = acceleration }
    public func loads(on state: RigidBodyState, mass: MassProperties) -> ExternalLoads {
        ExternalLoads(worldForce: acceleration * mass.mass)
    }
}

/// Earth-like flight: gravity, an exponential atmosphere, steady wind, and flat ground at y = 0.
public struct Atmosphere: FlightEnvironment {
    public var gravity = 9.81
    public var seaLevelDensity = 1.225
    public var scaleHeight = 8500.0
    public var wind = Vec3.zero

    public init(wind: Vec3 = .zero) { self.wind = wind }

    public func loads(on state: RigidBodyState, mass: MassProperties) -> ExternalLoads {
        ExternalLoads(worldForce: Vec3(0, -gravity * mass.mass, 0))
    }

    public func air(at position: Vec3) -> AirState? {
        AirState(density: seaLevelDensity * exp(-max(0, position.y) / scaleHeight), wind: wind)
    }

    public var groundHeight: Double? { 0 }
}
