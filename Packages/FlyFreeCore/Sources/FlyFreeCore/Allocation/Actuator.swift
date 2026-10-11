import simd

/// A single force- or moment-producing device on the craft.
public struct Actuator: Sendable, Equatable, Identifiable {
    public enum Kind: String, Sendable {
        case mainEngine
        case rcsThruster
        /// Aerodynamic control surface. Its wrench scales with dynamic pressure q̄, so its
        /// unit wrench is given *per pascal* and the allocator is told q̄ every step.
        case aeroSurface
    }

    /// How to draw a control surface in the instruments.
    public struct SurfaceVisual: Sendable, Equatable {
        public var hinge: Vec3
        public var span: Double
        /// Drawn as a mirrored pair deflecting in opposite directions (ailerons).
        public var mirrored: Bool
        public init(hinge: Vec3, span: Double, mirrored: Bool = false) {
            self.hinge = hinge
            self.span = span
            self.mirrored = mirrored
        }
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
    /// Command range: 0...1 for engines and thrusters, -1...1 for control surfaces.
    public var range: ClosedRange<Double>
    /// Explicit wrench at command = 1 (and effectiveness = 1), for devices that are not a single
    /// point force — e.g. a control surface modelled as a pure moment.
    public var customWrench: Wrench?
    public var surface: SurfaceVisual?

    public init(id: String, kind: Kind, position: Vec3, direction: Vec3, maxForce: Double, priority: Int,
                range: ClosedRange<Double> = 0...1, customWrench: Wrench? = nil, surface: SurfaceVisual? = nil) {
        self.id = id
        self.kind = kind
        self.position = position
        self.direction = simd_normalize(direction)
        self.maxForce = maxForce
        self.priority = priority
        self.range = range
        self.customWrench = customWrench
        self.surface = surface
    }

    /// A control surface producing a pure body moment of `momentPerPascal` (N·m/Pa) at full deflection.
    public static func surface(id: String, momentPerPascal: Vec3, position: Vec3, visual: SurfaceVisual) -> Actuator {
        Actuator(id: id, kind: .aeroSurface, position: position, direction: BodyAxis.up, maxForce: 0,
                 priority: 1, range: -1...1, customWrench: Wrench(torque: momentPerPascal), surface: visual)
    }

    /// Body wrench produced at command = 1 (and effectiveness = 1).
    public var unitWrench: Wrench {
        if let customWrench { return customWrench }
        let f = direction * maxForce
        return Wrench(force: f, torque: simd_cross(position, f))
    }
}
