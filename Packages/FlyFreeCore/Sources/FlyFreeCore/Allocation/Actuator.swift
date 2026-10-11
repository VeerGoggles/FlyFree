import simd

/// A single force-producing device on the craft.
public struct Actuator: Sendable, Equatable, Identifiable {
    public enum Kind: String, Sendable {
        case mainEngine
        case rcsThruster
        // Future: aeroSurface, reactionWheel, gimbaledEngine, rotor …
    }

    public var id: String
    public var kind: Kind
    /// Point of force application in body frame, relative to the center of mass (m).
    public var position: Vec3
    /// Unit direction of the force on the craft (opposite the exhaust plume).
    public var direction: Vec3
    /// Force at command = 1 (N).
    public var maxForce: Double
    /// Allocation priority: lower groups are solved first, later groups cover the residual.
    public var priority: Int

    public init(id: String, kind: Kind, position: Vec3, direction: Vec3, maxForce: Double, priority: Int) {
        self.id = id
        self.kind = kind
        self.position = position
        self.direction = simd_normalize(direction)
        self.maxForce = maxForce
        self.priority = priority
    }

    /// Body wrench produced at command = 1.
    public var unitWrench: Wrench {
        let f = direction * maxForce
        return Wrench(force: f, torque: simd_cross(position, f))
    }
}
